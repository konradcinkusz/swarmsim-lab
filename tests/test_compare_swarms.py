"""tools/compare_swarms.py reads SwarmApi's comparison as the API writes it.

The fixture is a real ``GET /api/scenario-runs/compare`` answer, cut down to four of the
eighteen scenarios, from comparing the reference swarm (base) with LabSwarm (head). The
live run is in EXPERIMENTS.md, E4, and in CI's compare job.
"""

from __future__ import annotations

import json
from pathlib import Path

from tools.compare_swarms import report_for, to_markdown

LAB = Path(__file__).resolve().parents[1]
FIXTURE = Path(__file__).parent / "fixtures" / "comparison.json"
WITH_EXPECTATIONS = Path(__file__).parent / "fixtures" / "comparison_with_expectations.json"


def test_each_scenario_is_a_row_with_its_change_and_closest_approach():
    table = to_markdown(json.loads(FIXTURE.read_text(encoding="utf-8")))
    rows = {line.split("|")[1].strip(" `"): line for line in table.splitlines() if "| `" in line}
    assert set(rows) == {
        "v_five_from_pads",
        "v_formation_from_pads",
        "line_five_noisy_gps",
        "formation_line",
    }
    assert "**fixed**" in rows["v_five_from_pads"] and "0.41 → 3.03" in rows["v_five_from_pads"]


def test_without_expectations_per_swarm_the_api_calls_an_improvement_a_regression():
    """F4 reached the API: ``expect`` was the scenario's, so a known limitation that the head
    swarm does not have read as a regression, and one it has read as a fix. The fixture is an
    answer from before swarmsim took an expectations file per swarm."""
    comparison = json.loads(FIXTURE.read_text(encoding="utf-8"))
    change = {s["name"]: s for s in comparison["scenarios"]}
    fixed_v = change["v_formation_from_pads"]
    assert fixed_v["change"] == "regressed"
    separation = next(m for m in fixed_v["measurements"] if m["assertion"] == "min_separation")
    assert separation["headMean"] > separation["baseMean"] + 2.0  # 0.48 m -> 3.14 m
    assert change["line_five_noisy_gps"]["change"] == "fixed"  # the lab's known limitation


def test_each_swarm_is_held_to_its_own_expectations():
    """With expectations/reference.yaml the reference swarm's four lab gaps are expected
    failures, and with expectations/lab.yaml LabSwarm's are: the same scenarios, a different
    outcome each, and each swarm meets its own (swarmsim's report ``ok``)."""
    paths = [str(LAB / "scenarios" / "v_five_from_pads.yaml")]

    reference = report_for(paths, "reference", [1], str(LAB / "expectations" / "reference.yaml"))
    lab = report_for(paths, "lab_swarm.swarm:LabSwarm", [1], str(LAB / "expectations" / "lab.yaml"))

    assert [s["outcome"] for s in reference["scenarios"]] == ["xfail"]
    assert reference["scenarios"][0]["expect_reason"].startswith("The lowest id leads")
    assert [s["outcome"] for s in lab["scenarios"]] == ["passed"]
    assert reference["ok"] and lab["ok"]


def test_held_to_its_own_expectations_each_swarm_reads_as_changed_not_regressed():
    """The same comparison, after swarmsim took an expectations file per swarm (F4). A real
    answer, cut down to five of the eighteen scenarios, from the reference swarm held to
    expectations/reference.yaml and LabSwarm to expectations/lab.yaml. Where one swarm's
    known gap is the other's pass the API says the expectation changed, and nothing reads
    as a regression."""
    comparison = json.loads(WITH_EXPECTATIONS.read_text(encoding="utf-8"))
    change = {
        s["name"]: (s["baseOutcome"], s["headOutcome"], s["change"])
        for s in comparison["scenarios"]
    }
    assert change == {
        "v_formation_from_pads": ("xfail", "passed", "changed"),  # swarmsim's own gap, fixed
        "v_five_from_pads": ("xfail", "passed", "changed"),
        "line_five_noisy_gps": ("passed", "xfail", "changed"),  # the lab's gap, not the reference's
        "follower_jammed_goes_home": ("passed", "xfail", "changed"),  # F11
        "formation_line": ("passed", "passed", "unchanged"),
    }
