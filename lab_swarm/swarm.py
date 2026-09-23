"""swarmsim's reference swarm, with formation roles chosen from where the drones are.

Everything that changes is on the ground. The drones run swarmsim's ``DroneController``
unchanged.

* A formation's leader and slots go to the drones nearest to them
  (``planning.formation_roles``), not to the lowest ids.
* A low-battery drone's place is re-planned among the drones still flying plus the
  idle drone that replaces it. A leader is succeeded by one of its followers. The
  reference instead hands the lead to the idle drone on its pad, and the whole formation
  turns back towards that pad (EXPERIMENTS.md, E0).
* The ground passes every drone's reported position to the supervisor. The reference
  supervisor never needed positions.

The supervisor subclasses swarmsim's ``MissionSupervisor``. It re-plans after the
reference ``start()`` and replaces its private ``_hand_over()``, because swarmsim has no
planning hook (EXPERIMENTS.md, finding F8). This is the smallest seam there is.
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

from .planning import Roles, formation_roles, slot_assignment


class FormationAwareSupervisor(MissionSupervisor):
    def __init__(self, drones: Sequence[str], battery_threshold_pct: float = 20.0) -> None:
        super().__init__(drones, battery_threshold_pct)
        self._positions: dict[str, Vector3] = {}

    def observe_position(self, drone: str, position: Vector3 | None) -> None:
        if position is not None and drone in self._battery:
            self._positions[drone] = position

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
        roles = formation_roles({d: self._positions[d] for d in drones}, offsets)
        _apply(active.plan, roles, list(mission.waypoints))
        return self._actions_for(active.plan, keep=actions)

    def _hand_over(self, low: str, replacement: str) -> list:
        mission = self._mission
        plan = mission.plan
        members = [d for d in plan.drones if d != low] + [replacement]
        if not plan.followers or any(d not in self._positions for d in members):
            return super()._hand_over(low, replacement)  # lanes: the rest of the lane
        (leader,) = plan.paths
        offsets = [offset for _, offset in plan.followers.values()]
        before = dict(plan.followers)
        if low == leader:
            progress = mission.progress.get(low)
            path = plan.paths[low]
            start = progress.waypoint_index if progress and progress.waypoint_index else 0
            remaining = path[min(start, len(path) - 1) :]
            roles = formation_roles({d: self._positions[d] for d in members}, offsets)
        else:
            remaining = plan.paths[leader]
            followers = {d: self._positions[d] for d in members if d != leader}
            slots, cost = slot_assignment(followers, self._positions[leader], offsets)
            roles = Roles(leader, slots, cost)
        _apply(plan, roles, remaining)

        actions: list = []
        if roles.leader != leader:
            actions.append(Assign(roles.leader, mission.mission_id, tuple(remaining)))
        actions += [
            Slot(drone, mission.mission_id, roles.leader, offset)
            for drone, offset in roles.slots.items()
            if before.get(drone) != (roles.leader, offset)
        ]
        return actions

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
                    observe(message.sender, message.body["position"])
        return super().step(now_s, inbox)


@dataclass
class LabSwarm(ReferenceSwarm):
    name: str = "lab"
    supervisor_cls: type[MissionSupervisor] = FormationAwareSupervisor

    def ground(self, fleet: Sequence[DroneInfo]) -> LabGround:
        supervisor = self.supervisor_cls([d.drone_id for d in fleet], self.battery_threshold_pct)
        return LabGround(fleet, supervisor)
