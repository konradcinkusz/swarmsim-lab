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

The first two are swarmsim's ``Planner`` seam (``formation_planner``): the supervisor
asks it when a mission starts. Before that seam existed this module rewrote the
supervisor's private plan after its ``start()`` (EXPERIMENTS.md, finding F8). The last
two are a subclass of ``MissionSupervisor``. Averaging overrides ``observe_position``.
The hand-over has no seam in swarmsim, so ``_hand_over`` reads the supervisor's private
mission. Its class attributes are the lab's decisions, and each mutant in mutants.py
changes one of them.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from swarm_coordination.mission_planning import (
    FORMATIONS,
    MissionMessage,
    MissionPlan,
    plan_mission,
)
from swarm_coordination.scenarios.sut import ReferenceSwarm
from swarm_coordination.supervisor import Assign, MissionSupervisor, Planner, Slot
from swarm_coordination.trajectory import Vector3

from .planning import Cost, Roles, distance, formation_roles


def formation_planner(cost: Cost = distance, prefer=max, form_up: bool = True) -> Planner:
    """swarmsim's planner for the lab's formations: roles by assignment, from positions."""

    def plan(
        mission: MissionMessage, eligible: Sequence[str], positions: Mapping[str, Vector3]
    ) -> MissionPlan:
        planned = plan_mission(mission, eligible[: mission.drone_count])  # swarmsim's drones
        drones = planned.drones
        if mission.mission_type != "formation" or any(d not in positions for d in drones):
            return planned  # lanes, or not everyone has said where it is: the id-order plan
        offsets = FORMATIONS[mission.formation](len(drones) - 1, mission.spacing_m)
        roles = formation_roles({d: positions[d] for d in drones}, offsets, cost, prefer=prefer)
        path = list(mission.waypoints)
        if form_up:
            here = positions[roles.leader]
            path.insert(0, Vector3(here.x, here.y, path[0].z))
        _apply(planned, roles, path)
        return planned

    return plan


class FormationAwareSupervisor(MissionSupervisor):
    cost: Cost = staticmethod(distance)
    prefer = staticmethod(max)  # among equally cheap plans: the one with the most clearance
    form_up = True
    average_parked = True
    plans_roles = True  # False: swarmsim's own roles, the lowest id leading (a mutant)

    def __init__(self, drones: Sequence[str], battery_threshold_pct: float = 20.0) -> None:
        planner = (
            formation_planner(self.cost, self.prefer, self.form_up) if self.plans_roles else None
        )
        super().__init__(drones, battery_threshold_pct, planner=planner)
        self._parked: dict[str, tuple[Vector3, int]] = {}  # sum and count while disarmed

    def observe_position(self, drone: str, position: Vector3 | None, armed: bool) -> None:
        """A drone's reported position. One report is only as good as the GPS behind it.
        A disarmed drone is not moving, though, so its reports are averaged until it arms."""
        if position is None or drone not in self.drones:
            return
        if armed or not self.average_parked:
            self._parked.pop(drone, None)
            super().observe_position(drone, position, armed)
            return
        total, count = self._parked.get(drone, (Vector3(0.0, 0.0, 0.0), 0))
        total, count = total + position, count + 1
        self._parked[drone] = (total, count)
        super().observe_position(drone, total.scale(1.0 / count), armed)

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


def _apply(plan: MissionPlan, roles: Roles, path: list[Vector3]) -> None:
    plan.paths = {roles.leader: list(path)}
    plan.followers = {d: (roles.leader, offset) for d, offset in roles.slots.items()}


@dataclass
class LabSwarm(ReferenceSwarm):
    name: str = "lab"
    supervisor_cls: type[MissionSupervisor] = FormationAwareSupervisor
