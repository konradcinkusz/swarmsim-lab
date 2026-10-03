"""swarmsim's own scenarios on LabSwarm: what LabSwarm is expected to fail, and why.

They are the scenarios swarmsim wrote for its reference swarm, and LabSwarm is that swarm
with its formation roles chosen differently. So where LabSwarm fails one, either it
regressed or the scenario assumes the reference's roles. With the lab's expectations
(expectations/lab.yaml, passed with --expect) the suite holds LabSwarm to everything but
the one scenario that does (finding F11).

Findings F3 (``formation_error`` assumed the reference's roles), F4 (a known limitation
belonged to the scenario file), F10 (``follower_comms_blip`` measured nothing) and F9 (an
absolute end position) were fixed in swarmsim, and what they changed is in EXPERIMENTS.md.
"""

from __future__ import annotations

from swarm_coordination.scenarios import load_scenario, run_scenario, run_suite
from swarm_coordination.trajectory import Vector3

from lab_swarm.swarm import LabSwarm

SEEDS = [1, 2, 3]


def test_lab_swarm_meets_swarmsims_suite_but_for_the_scenario_that_jams_its_leader(
    swarmsim_scenarios, lab_expectations
):
    report = run_suite([swarmsim_scenarios], LabSwarm(), SEEDS, expectations=lab_expectations)

    assert report.ok
    outcomes = {s.spec.name: s.outcome for s in report.scenarios}
    assert {n for n, outcome in outcomes.items() if outcome != "passed"} == {
        "follower_jammed_goes_home"
    }
    assert outcomes["follower_jammed_goes_home"] == "xfail"
    # lab.yaml also names the lab's own scenario, which is not in this run.
    assert report.unused_expectations() == ["line_five_noisy_gps"]


def test_under_the_scenario_files_own_expectations_a_fixed_gap_fails_swarmsims_suite(
    swarmsim_scenarios,
):
    """F4: the files' ``expect`` is the reference swarm's. LabSwarm passes the V from the
    pads that the reference fails, which the files call an xpass and a failed suite."""
    report = run_suite([swarmsim_scenarios / "v_formation_from_pads.yaml"], LabSwarm(), SEEDS)

    (scenario,) = report.scenarios
    assert scenario.outcome == "xpass"
    assert not report.ok


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
