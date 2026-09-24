"""The lab's own scenarios (scenarios/): LabSwarm meets them, and they have teeth.

swarmsim's mutation check refuses a swarm it did not write (finding F1), so the lab has
its own. Every mutant in lab_swarm/mutants.py undoes one of LabSwarm's decisions and
must fail at least one scenario. Every scenario that LabSwarm must pass must fail a
mutant or the reference swarm (all the decisions undone at once), or it proves nothing.
Seeds 1-3, as in CI; a scenario "fails" a swarm when any seed fails.
"""

from __future__ import annotations

import pytest
from swarm_coordination.scenarios import ReferenceSwarm, load_scenario, run_scenario, run_suite

from lab_swarm.mutants import MUTANTS
from lab_swarm.swarm import LabSwarm

SEEDS = (1, 2, 3)
REFERENCE_FAILS = {
    "v_five_from_pads",
    "v_after_lanes",
    "v_low_battery_drone_1",
    "v_five_very_noisy_gps",
}


@pytest.fixture(scope="module")
def specs(lab_scenarios):
    return {p.stem: load_scenario(p) for p in sorted(lab_scenarios.glob("*.yaml"))}


def _passes(spec, sut) -> bool:
    return all(run_scenario(spec, sut, seed)[0].passed for seed in SEEDS)


def test_lab_swarm_meets_every_expectation(specs):
    unmet = [
        name for name, spec in specs.items() if _passes(spec, LabSwarm()) != (spec.expect == "pass")
    ]
    assert unmet == []


@pytest.fixture(scope="module")
def killed_by(specs):
    """For each mutant, the scenarios (that LabSwarm must pass) which it fails."""
    return {
        name: {s for s, spec in specs.items() if spec.expect == "pass" and not _passes(spec, sut)}
        for name, sut in MUTANTS.items()
    }


def test_every_mutant_fails_a_scenario(killed_by):
    assert [name for name, scenarios in killed_by.items() if not scenarios] == []


def test_every_scenario_fails_a_mutant_or_the_reference(specs, killed_by):
    teeth = set().union(*killed_by.values()) | REFERENCE_FAILS
    assert [s for s, spec in specs.items() if spec.expect == "pass" and s not in teeth] == []


def test_each_decision_is_caught_by_the_scenario_written_for_it(killed_by):
    assert "line_beside_the_pads" in killed_by["no_form_up"]
    assert "v_five_very_noisy_gps" in killed_by["latest_position"]
    assert "line_five_from_pads" in killed_by["closest_tie"]
    assert "v_low_battery_drone_2" in killed_by["reference_handover"]
    assert "v_five_from_pads" in killed_by["id_order_roles"]


def test_the_reference_swarm_fails_what_the_lab_fixed(specs):
    failed = {name for name, spec in specs.items() if not _passes(spec, ReferenceSwarm())}
    assert failed == REFERENCE_FAILS


def test_an_expect_fail_is_the_lab_swarms_and_the_reference_turns_it_into_an_xpass(lab_scenarios):
    """F4: ``expect`` belongs to the scenario file, not to the swarm under test. The
    lab's known limitation is one the reference doesn't have, so the lab's suite fails
    the reference on it."""
    report = run_suite([lab_scenarios / "line_five_noisy_gps.yaml"], ReferenceSwarm(), list(SEEDS))
    (scenario,) = report.scenarios
    assert scenario.outcome == "xpass"
    assert not report.ok
