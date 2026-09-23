"""FormationAwareSupervisor's decisions, driven directly (lab_swarm/swarm.py)."""

from __future__ import annotations

import uuid

import pytest
from swarm_coordination.mission_planning import MissionMessage
from swarm_coordination.supervisor import Assign, Command, Slot
from swarm_coordination.trajectory import Vector3

from lab_swarm.swarm import FormationAwareSupervisor

DRONES = ["drone_1", "drone_2", "drone_3", "drone_4"]


def _pad(drone: str) -> Vector3:
    return Vector3(0.0, 3.0 * (int(drone.rsplit("_", 1)[1]) - 1), 0.0)


def _supervisor(drones=DRONES) -> FormationAwareSupervisor:
    supervisor = FormationAwareSupervisor(drones)
    for d in drones:
        supervisor.observe_position(d, _pad(d), armed=False)
        supervisor.observe_battery(d, 90.0)
    return supervisor


def _mission(formation: str = "v", count: int = 3, kind: str = "formation") -> MissionMessage:
    waypoints = [Vector3(0.0, 0.0, 6.0), Vector3(50.0, 0.0, 6.0)]
    return MissionMessage(str(uuid.uuid4()), kind, formation, waypoints, count, 3.0)


def _by_type(actions, kind):
    return [a for a in actions if isinstance(a, kind)]


def test_a_formation_forms_up_above_its_leader_before_it_flies_the_route():
    actions = _supervisor().start(_mission())
    (assign,) = _by_type(actions, Assign)
    assert assign.drone == "drone_2"  # the middle of drone_1..drone_3
    assert assign.waypoints == (
        Vector3(0.0, 3.0, 6.0),  # straight up from its pad
        Vector3(0.0, 0.0, 6.0),
        Vector3(50.0, 0.0, 6.0),
    )
    slots = {s.drone: s.offset for s in _by_type(actions, Slot)}
    assert slots == {"drone_1": Vector3(-3.0, -3.0, 0.0), "drone_3": Vector3(-3.0, 3.0, 0.0)}


def test_lanes_are_left_to_the_reference():
    actions = _supervisor().start(_mission(kind="waypoint"))
    assert sorted(a.drone for a in _by_type(actions, Assign)) == ["drone_1", "drone_2", "drone_3"]
    assert _by_type(actions, Slot) == []


def test_without_every_position_it_keeps_the_id_order_plan():
    supervisor = FormationAwareSupervisor(DRONES)
    (assign,) = _by_type(supervisor.start(_mission()), Assign)
    assert assign.drone == "drone_1"


def test_a_disarmed_drone_is_averaged_and_an_armed_one_is_not():
    supervisor = FormationAwareSupervisor(DRONES)
    for y in (2.0, 4.0, 3.0):
        supervisor.observe_position("drone_2", Vector3(0.0, y, 0.0), armed=False)
    assert supervisor._positions["drone_2"] == Vector3(0.0, 3.0, 0.0)
    supervisor.observe_position("drone_2", Vector3(1.0, 5.0, 2.0), armed=True)
    assert supervisor._positions["drone_2"] == Vector3(1.0, 5.0, 2.0)


def _flying(supervisor, positions: dict[str, Vector3]) -> None:
    for d, p in positions.items():
        supervisor.observe_position(d, p, armed=True)


def test_a_leader_low_on_battery_is_succeeded_by_a_follower_not_by_the_drone_on_its_pad():
    supervisor = _supervisor()
    supervisor.start(_mission())  # drone_2 leads, drone_1 and drone_3 follow
    _flying(
        supervisor,
        {
            "drone_2": Vector3(30.0, 0.0, 6.0),
            "drone_1": Vector3(27.0, -3.0, 6.0),
            "drone_3": Vector3(27.0, 3.0, 6.0),
        },
    )
    # Past the form-up point and the first waypoint: heading for the second.
    supervisor.observe_progress("drone_2", supervisor.active_mission_id, 2, False)
    supervisor.observe_battery("drone_2", 15.0)
    actions = supervisor.reallocate()

    assert Command("drone_2", "rtl") in actions
    (assign,) = _by_type(actions, Assign)
    assert assign.drone in {"drone_1", "drone_3"}
    assert assign.waypoints == (Vector3(50.0, 0.0, 6.0),)  # the rest of the route
    slots = {s.drone: (s.leader, s.offset) for s in _by_type(actions, Slot)}
    assert "drone_4" in slots  # the idle drone joins as a follower
    assert all(leader == assign.drone for leader, _ in slots.values())


def test_a_follower_low_on_battery_is_replaced_in_its_slot_and_the_leader_keeps_the_lead():
    supervisor = _supervisor()
    supervisor.start(_mission())
    _flying(
        supervisor,
        {
            "drone_2": Vector3(30.0, 0.0, 6.0),
            "drone_1": Vector3(27.0, -3.0, 6.0),
            "drone_3": Vector3(27.0, 3.0, 6.0),
        },
    )
    supervisor.observe_battery("drone_1", 15.0)
    actions = supervisor.reallocate()

    assert Command("drone_1", "rtl") in actions
    assert _by_type(actions, Assign) == []
    slots = {s.drone: (s.leader, s.offset) for s in _by_type(actions, Slot)}
    assert slots == {"drone_4": ("drone_2", Vector3(-3.0, -3.0, 0.0))}


@pytest.mark.parametrize("formation", ["line", "v"])
def test_every_follower_gets_a_distinct_slot(formation):
    actions = _supervisor().start(_mission(formation, count=4))
    offsets = [s.offset for s in _by_type(actions, Slot)]
    assert len(offsets) == len(set(offsets)) == 3
