"""The lab's own scenarios (scenarios/): LabSwarm meets them, and they have teeth.

The lab's mutants (lab_swarm/mutants.py) go to swarmsim's mutation check, which flies every
scenario against them: each scenario LabSwarm passes must fail at least one mutant, and
each mutant must fail a scenario. The last mutant is the reference swarm, every decision
undone at once. The lab ran this check itself while swarmsim's refused a swarm it did not
write (finding F1).

Each swarm's known limitations are its own file, in expectations/ (finding F4): the scenario
files carry none. Seeds 1-3, as in CI.
"""

from __future__ import annotations

import pytest
from swarm_coordination.scenarios import ReferenceSwarm, run_suite
from swarm_coordination.scenarios.runner import to_markdown

from lab_swarm.mutants import MUTANTS
from lab_swarm.swarm import LabSwarm

SEEDS = [1, 2, 3]
REFERENCE_FAILS = {
    "v_five_from_pads",
    "v_after_lanes",
    "v_low_battery_drone_1",
    "v_five_very_noisy_gps",
}


@pytest.fixture(scope="module")
def suite(lab_scenarios, lab_expectations):
    """swarmsim's runner on the lab's scenarios: LabSwarm, its expectations, its mutants."""
    return run_suite(
        [lab_scenarios],
        LabSwarm(),
        SEEDS,
        mutation=True,
        mutants=MUTANTS,
        expectations=lab_expectations,
    )


def test_lab_swarm_meets_every_expectation_and_every_scenario_has_teeth(suite):
    assert suite.ok, to_markdown(suite)
    assert not suite.survivors()
    assert not [s.spec.name for s in suite.scenarios if s.toothless]


def test_each_decision_is_caught_by_the_scenario_written_for_it(suite):
    killed = {s.spec.name: set(s.killed_by or []) for s in suite.scenarios}
    assert "no_form_up" in killed["line_beside_the_pads"]
    assert "latest_position" in killed["v_five_very_noisy_gps"]
    assert "closest_tie" in killed["line_five_from_pads"]
    assert "reference_handover" in killed["v_low_battery_drone_2"]
    assert "id_order_roles" in killed["v_five_from_pads"]


def test_the_reference_swarm_fails_what_the_lab_fixed(lab_scenarios, reference_expectations):
    report = run_suite(
        [lab_scenarios], ReferenceSwarm(), SEEDS, expectations=reference_expectations
    )
    failed = {s.spec.name for s in report.scenarios if s.outcome == "xfail"}
    assert failed == REFERENCE_FAILS
    assert report.ok, to_markdown(report)


def test_a_known_limitation_is_a_swarms_and_the_scenario_file_says_nothing(
    lab_scenarios, lab_expectations
):
    """F4: ``line_five_noisy_gps`` is LabSwarm's gap and not the reference's. Under LabSwarm's
    file it is an expected failure; under the scenario file's own expectation, which is the
    reference's, it passes. Before swarmsim took an expectations file per swarm, the lab's
    ``expect: fail`` in the scenario made the reference swarm's run an xpass, and a failed suite."""
    path = [lab_scenarios / "line_five_noisy_gps.yaml"]

    (lab,) = run_suite(path, LabSwarm(), SEEDS, expectations=lab_expectations).scenarios
    (reference,) = run_suite(path, ReferenceSwarm(), SEEDS).scenarios

    assert lab.outcome == "xfail" and lab.spec.expect_reason
    assert reference.outcome == "passed"
