"""MinimalSwarm against swarmsim's own scenarios: what a swarm written from scratch passes.

MinimalSwarm (lab_swarm/minimal.py) flies lanes and obeys operator commands. It has no
formations, no battery policy and no comms-loss policy, so swarmsim's scenarios for
those must fail it. This test pins exactly which assertions fail in which scenario. If
swarmsim changes a scenario, or MinimalSwarm grows a behaviour, the table must change
with it.
"""

from __future__ import annotations

import pytest
from swarm_coordination.scenarios import load_scenario, run_scenario

from lab_swarm.minimal import MinimalSwarm

EXPECTED_FAILURES = {
    "waypoint_lanes": set(),
    "scale_ten_lanes": set(),
    # A formation mission is ignored, so it is never reported complete, and no formation
    # ever flies: the distance checks have nothing to measure and say so (F10).
    "formation_line": {"formation_error", "min_separation", "mission_completes"},
    "formation_line_in_wind": {"formation_error", "min_separation", "mission_completes"},
    "follower_comms_blip": {"formation_error", "min_separation", "mission_completes"},
    "v_formation_from_pads": {"min_separation", "mission_completes"},
    # Nobody takes off: the leader's destination and the separation notice (see below).
    "follower_jammed_goes_home": {"final_position", "min_separation"},
    # No battery policy: a low drone keeps flying its lane.
    "low_battery_handover": {"no_task_below_battery", "reaches", "final_position"},
    "plan_around_low_battery": {"never_mode", "final_position"},
    # It lands where "land" found it, and that is all the scenario asks (F9, fixed).
    "operator_land_in_place": set(),
}


@pytest.mark.parametrize("seed", [1, 2])
def test_it_fails_exactly_what_it_lacks(swarmsim_scenarios, seed):
    found = {}
    for path in sorted(swarmsim_scenarios.glob("*.yaml")):
        verdict, _ = run_scenario(load_scenario(path), MinimalSwarm(), seed)
        found[verdict.scenario] = {o.assertion for o in verdict.outcomes if not o.passed}
    assert found == EXPECTED_FAILURES


def test_it_lands_where_the_command_found_it(swarmsim_scenarios):
    """operator_land_in_place asks for exactly that, measured from the moment of the command.

    MinimalSwarm cruises at PX4's 5 m/s, so at t=15 s it is at x=42 m, where the reference
    swarm, cruising at 2 m/s, has got to x=18 m. The scenario used to want drone_1 within
    6 m of x=16 m, which failed MinimalSwarm for flying faster (F9). It asks for travel
    after the command now, and MinimalSwarm moves 0.5 m.
    """
    spec = load_scenario(swarmsim_scenarios / "operator_land_in_place.yaml")
    verdict, _ = run_scenario(spec, MinimalSwarm(), 1)

    travel = next(o for o in verdict.outcomes if o.assertion == "travel_after")
    assert travel.passed and travel.measured < 1.0
    assert verdict.passed


def _ever_airborne(trace) -> set[str]:
    return {s.drone_id for f in trace.frames for s in f.drones if s.airborne}


def test_a_swarm_that_never_flies_still_passes_the_assertions_that_ask_for_nothing_flown(
    swarmsim_scenarios,
):
    """follower_jammed_goes_home against a swarm that never took off (F9, F10).

    The two "reaches its pad" checks and all_landed are true of a swarm that never left the
    ground. The leader's final position and the separation, which has nothing to measure
    with nobody airborne, are not, so the scenario fails it.
    """
    spec = load_scenario(swarmsim_scenarios / "follower_jammed_goes_home.yaml")
    verdict, trace = run_scenario(spec, MinimalSwarm(), 1)
    assert _ever_airborne(trace) == set()
    passed = [o.assertion for o in verdict.outcomes if o.passed]
    assert passed == ["reaches", "reaches", "all_landed"]
    assert not verdict.passed
