"""swarmsim's reference swarm, with formation roles chosen from where the drones are.

Everything that changes is on the ground. The drones run swarmsim's ``DroneController``
unchanged.

* A formation's leader and slots go to the drones nearest to them
  (``planning.formation_roles``), not to the lowest ids.
* The formation forms up before it flies. The leader's path starts straight above
  where it stands, so its followers take their slots around a leader that is not yet
  moving sideways.
* A drone that is disarmed is on the ground and not moving, so the ground averages its
  position reports. The average, not the latest noisy report, is what the plan starts
  from.
* A leader low on battery is succeeded by one of its followers. The roles are
  re-planned among the drones still flying plus the idle drone that joins. The
  reference instead hands the lead to the idle drone on its pad, and the whole
  formation turns back towards that pad (EXPERIMENTS.md, E0).

The supervisor subclasses swarmsim's ``MissionSupervisor``. It re-plans after the
reference ``start()`` and replaces its private ``_hand_over()``, because swarmsim has
no planning hook (EXPERIMENTS.md, finding F8). Its class attributes are the lab's
decisions, and each mutant in mutants.py changes one of them.
"""

from __future__ import annotations

from collections.abc import Sequence
from dataclasses import dataclass

from swarm_coordination.mission_planning import FORMATIONS, MissionMessage, MissionPlan
from swarm_coordination.scenarios.sut import (
    DroneInfo,
    Message,
    ReferenceGround,
    ReferenceSwarm,
)
from swarm_coordination.supervisor import (
    ActiveMission,
    Assign,
    Command,
    MissionSupervisor,
    Slot,
)
from swarm_coordination.trajectory import Vector3

from .planning import Cost, Roles, distance, formation_roles


class FormationAwareSupervisor(MissionSupervisor):
    cost: Cost = staticmethod(distance)
    prefer = staticmethod(max)  # among equally cheap plans: the one with the most clearance
    form_up = True
    average_parked = True

    def __init__(self, drones: Sequence[str], battery_threshold_pct: float = 20.0) -> None:
        super().__init__(drones, battery_threshold_pct)
        self._positions: dict[str, Vector3] = {}
        self._parked: dict[str, tuple[Vector3, int]] = {}  # sum and count while disarmed

    def observe_position(self, drone: str, position: Vector3 | None, armed: bool) -> None:
        """A drone's reported position. One report is only as good as the GPS behind it.
        A disarmed drone is not moving, though, so its reports are averaged until it arms."""
        if position is None or drone not in self._battery:
            return
        if armed or not self.average_parked:
            self._parked.pop(drone, None)
            self._positions[drone] = position
            return
        total, count = self._parked.get(drone, (Vector3(0.0, 0.0, 0.0), 0))
        total, count = total + position, count + 1
        self._parked[drone] = (total, count)
        self._positions[drone] = total.scale(1.0 / count)

    def start(self, mission: MissionMessage) -> list:
        actions = super().start(mission)
        active = self._mission
        if (
            mission.mission_type != "formation"
            or active is None
            or active.mission_id != mission.mission_id
        ):
            return actions  # lanes, or a rejected mission: nothing to re-plan
        drones = active.plan.drones
        if any(d not in self._positions for d in drones):
            return actions  # not everyone has said where it is: keep the id-order plan
        offsets = FORMATIONS[mission.formation](len(drones) - 1, mission.spacing_m)
        roles = self._roles(drones, offsets)
        path = list(mission.waypoints)
        if self.form_up:
            here = self._positions[roles.leader]
            path.insert(0, Vector3(here.x, here.y, path[0].z))
        _apply(active.plan, roles, path)
        return self._actions_for(active.plan, keep=actions)

    def _hand_over(self, low: str, replacement: str) -> list:
        mission = self._mission
        plan = mission.plan
        members = [d for d in plan.drones if d != low] + [replacement]
        if low not in plan.paths or not plan.followers:
            # A follower's replacement takes its slot, as in the reference. Re-planning the
            # slots measured no better (EXPERIMENTS.md, E3). Lanes: the rest of the lane.
            return super()._hand_over(low, replacement)
        if any(d not in self._positions for d in members):
            return super()._hand_over(low, replacement)
        progress = mission.progress.get(low)
        path = plan.paths[low]
        start = progress.waypoint_index if progress and progress.waypoint_index else 0
        remaining = path[min(start, len(path) - 1) :]
        offsets = [offset for _, offset in plan.followers.values()]
        before = dict(plan.followers)
        roles = self._roles(members, offsets)
        _apply(plan, roles, remaining)
        actions: list = [Assign(roles.leader, mission.mission_id, tuple(remaining))]
        actions += [
            Slot(drone, mission.mission_id, roles.leader, offset)
            for drone, offset in roles.slots.items()
            if before.get(drone) != (roles.leader, offset)
        ]
        return actions

    def _roles(self, drones: list[str], offsets: list[Vector3]) -> Roles:
        positions = {d: self._positions[d] for d in drones}
        return formation_roles(positions, offsets, self.cost, prefer=self.prefer)

    @staticmethod
    def _actions_for(plan: MissionPlan, keep: list) -> list:
        """The reference ``start()``'s actions, with its assignments replaced by ``plan``'s."""
        commands = [a for a in keep if isinstance(a, Command)]
        active = [a for a in keep if isinstance(a, ActiveMission)]
        return (
            commands
            + [Assign(d, plan.mission_id, tuple(path)) for d, path in plan.paths.items()]
            + [
                Slot(d, plan.mission_id, leader, offset)
                for d, (leader, offset) in plan.followers.items()
            ]
            + active
        )


def _apply(plan: MissionPlan, roles: Roles, path: list[Vector3]) -> None:
    plan.paths = {roles.leader: list(path)}
    plan.followers = {d: (roles.leader, offset) for d, offset in roles.slots.items()}


class LabGround(ReferenceGround):
    """The reference ground software, which also tells the supervisor where drones are."""

    def step(self, now_s: float, inbox: Sequence[Message]) -> list[Message]:
        observe = getattr(self.supervisor, "observe_position", None)
        if observe is not None:
            for message in inbox:
                if message.topic == "telemetry":
                    observe(message.sender, message.body["position"], message.body["armed"])
        return super().step(now_s, inbox)


@dataclass
class LabSwarm(ReferenceSwarm):
    name: str = "lab"
    supervisor_cls: type[MissionSupervisor] = FormationAwareSupervisor

    def ground(self, fleet: Sequence[DroneInfo]) -> LabGround:
        supervisor = self.supervisor_cls([d.drone_id for d in fleet], self.battery_threshold_pct)
        return LabGround(fleet, supervisor)
