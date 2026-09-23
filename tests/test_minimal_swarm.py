"""MinimalSwarm against swarmsim's own scenarios: what a swarm written from scratch passes.

MinimalSwarm (lab_swarm/minimal.py) flies lanes and obeys operator commands. It has no
formations, no battery policy and no comms-loss policy, so swarmsim's scenarios for
those must fail it. This test pins exactly which assertions fail in which scenario. If
swarmsim changes a scenario, or MinimalSwarm grows a behaviour, the table must change
with it.
"""

from __future__ import annotations

import pytest
from swarm_coordination.scenarios import ReferenceSwarm, load_scenario, run_scenario

from lab_swarm.metrics import ever_airborne, travel_after
from lab_swarm.minimal import MinimalSwarm

EXPECTED_FAILURES = {
    "waypoint_lanes": set(),
    "scale_ten_lanes": set(),
    # A formation mission is ignored, so it is never reported complete.
    "formation_line": {"mission_completes"},
    "formation_line_in_wind": {"mission_completes"},
    "follower_comms_blip": {"mission_completes"},
    "v_formation_from_pads": {"mission_completes"},
    # Nobody takes off, and only the leader's destination notices (see below).
    "follower_jammed_goes_home": {"final_position"},
    # No battery policy: a low drone keeps flying its lane.
    "low_battery_handover": {"no_task_below_battery", "reaches", "final_position"},
    "plan_around_low_battery": {"never_mode", "final_position"},
    # It lands where it is, but that is not where the reference swarm is at t=15 s (F9).
    "operator_land_in_place": {"final_position"},
}


@pytest.mark.parametrize("seed", [1, 2])
def test_it_fails_exactly_what_it_lacks(swarmsim_scenarios, seed):
    found = {}
    for path in sorted(swarmsim_scenarios.glob("*.yaml")):
        verdict, _ = run_scenario(load_scenario(path), MinimalSwarm(), seed)
        found[verdict.scenario] = {o.assertion for o in verdict.outcomes if not o.passed}
    assert found == EXPECTED_FAILURES


def test_it_lands_where_the_command_found_it(swarmsim_scenarios):
    """operator_land_in_place's intent, measured from the moment of the command (F9).

    Every drone stops within 1 m of where "land" found it. MinimalSwarm cruises at PX4's
    5 m/s, so at t=15 s it is at x=42 m. swarmsim's final_position wants drone_1 within
    6 m of x=16 m, which is where the reference swarm, cruising at 2 m/s, has got to.
    """
    spec = load_scenario(swarmsim_scenarios / "operator_land_in_place.yaml")
    (command_at,) = [e.at_s for e in spec.events if e.kind == "command"]
    for sut in (MinimalSwarm(), ReferenceSwarm()):
        verdict, trace = run_scenario(spec, sut, 1)
        travel = travel_after(trace, command_at)
        assert max(travel.values()) < 1.0, (sut.name, travel)
    assert [o.assertion for o in verdict.outcomes if not o.passed] == []  # the reference


def test_a_swarm_that_never_flies_passes_most_of_follower_jammed_goes_home(swarmsim_scenarios):
    """Assertions about end states pass for a swarm that never took off (F9).

    Only drone_1's final_position catches it. Both "reaches its pad" checks, all_landed
    and min_separation pass, because nothing ever left the ground.
    """
    spec = load_scenario(swarmsim_scenarios / "follower_jammed_goes_home.yaml")
    verdict, trace = run_scenario(spec, MinimalSwarm(), 1)
    assert ever_airborne(trace) == set()
    passed = [o.assertion for o in verdict.outcomes if o.passed]
    assert passed == ["reaches", "reaches", "all_landed", "min_separation"]
