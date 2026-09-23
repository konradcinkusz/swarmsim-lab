"""LabSwarm against swarmsim's own scenarios, and what that shows about the scenarios.

LabSwarm is the reference swarm with formation roles chosen differently. So where it
fails swarmsim's suite, either it regressed or the scenario assumes the reference's
roles. This test pins which is which. Each assumption is a finding in EXPERIMENTS.md.
"""

from __future__ import annotations

import pytest
from swarm_coordination.scenarios import ReferenceSwarm, load_scenario, run_scenario, run_suite
from swarm_coordination.scenarios.assertions import formation_error
from swarm_coordination.trajectory import Vector3

from lab_swarm.metrics import formation_shape_error
from lab_swarm.swarm import LabSwarm

EXPECTED_FAILURES = {
    "waypoint_lanes": set(),
    "scale_ten_lanes": set(),
    "low_battery_handover": set(),
    "plan_around_low_battery": set(),
    "operator_land_in_place": set(),
    "follower_comms_blip": set(),  # its formation check measures nothing (F10)
    "v_formation_from_pads": set(),  # an expect: fail that LabSwarm fixed (F4)
    "formation_line": {"formation_error"},  # assumes drone_1 leads (F3)
    "formation_line_in_wind": {"formation_error"},  # likewise (F3)
    "follower_jammed_goes_home": {"reaches", "final_position"},  # jams LabSwarm's leader (F11)
}


def _outcome(verdict, kind):
    return next(o for o in verdict.outcomes if o.assertion == kind)


@pytest.mark.parametrize("seed", [1, 2])
def test_lab_swarm_fails_only_where_a_scenario_assumes_the_reference_roles(
    swarmsim_scenarios, seed
):
    found = {}
    for path in sorted(swarmsim_scenarios.glob("*.yaml")):
        verdict, _ = run_scenario(load_scenario(path), LabSwarm(), seed)
        found[verdict.scenario] = {o.assertion for o in verdict.outcomes if not o.passed}
    assert found == EXPECTED_FAILURES


def test_f3_formation_error_assumes_drone_1_leads_while_the_lab_line_holds_its_shape(
    swarmsim_scenarios,
):
    spec = load_scenario(swarmsim_scenarios / "formation_line.yaml")
    limit = next(a.params for a in spec.assertions if a.kind == "formation_error")

    verdict, trace = run_scenario(spec, ReferenceSwarm(), 1)
    shape, _ = formation_shape_error(trace, limit["from_s"])
    # With the reference's roles, the role-free measure agrees with swarmsim's.
    assert shape == pytest.approx(_outcome(verdict, "formation_error").measured, abs=1e-3)

    verdict, trace = run_scenario(spec, LabSwarm(), 1)
    shape, _ = formation_shape_error(trace, limit["from_s"])
    assert _outcome(verdict, "formation_error").measured > 10.0  # drone_3 leads, not drone_1
    assert shape <= limit["max_m"]


def test_f4_a_known_limitation_that_is_fixed_fails_swarmsims_suite(swarmsim_scenarios):
    report = run_suite([swarmsim_scenarios / "v_formation_from_pads.yaml"], LabSwarm(), [1])
    (scenario,) = report.scenarios
    assert scenario.outcome == "xpass"
    assert not report.ok


def test_f10_follower_comms_blip_has_never_measured_its_formation(swarmsim_scenarios):
    """The formation lands at about 37 s, and the check starts at 40 s. It passes having
    looked at nothing, for the reference too. A window from 31 s (3 s after the blip)
    would measure the reference at 2.42 m, inside the 2.5 m limit."""
    spec = load_scenario(swarmsim_scenarios / "follower_comms_blip.yaml")
    for sut in (ReferenceSwarm(), LabSwarm()):
        verdict, trace = run_scenario(spec, sut, 1)
        check = _outcome(verdict, "formation_error")
        assert check.passed and check.measured is None
    reference_trace = run_scenario(spec, ReferenceSwarm(), 1)[1]
    after_the_blip = formation_error(spec, reference_trace, {"max_m": 2.5, "from_s": 31})
    assert after_the_blip.passed
    assert after_the_blip.measured == pytest.approx(2.42, abs=0.01)


def test_f11_follower_jammed_goes_home_jams_the_lab_swarms_leader(swarmsim_scenarios):
    """The scenario jams drone_2 and drone_3, meaning the followers. LabSwarm's line from
    three pads is led by drone_2, which finishes the route alone, as a leader does.
    drone_1, the follower nobody jammed, can no longer hear its leader, so it goes home.
    The scenario expects the opposite of both."""
    spec = load_scenario(swarmsim_scenarios / "follower_jammed_goes_home.yaml")
    _, trace = run_scenario(spec, LabSwarm(), 1)
    end = {s.drone_id: s.position for s in trace.frames[-1].drones}
    assert end["drone_2"].distance_to(Vector3(60.0, 0.0, 0.0)) < 1.0
    assert end["drone_1"].distance_to(trace.homes["drone_1"]) < 1.0
