"""tools/compare_swarms.py reads SwarmApi's comparison as the API writes it.

The fixture is a real ``GET /api/scenario-runs/compare`` answer, cut down to four of the
eighteen scenarios, from comparing the reference swarm (base) with LabSwarm (head). The
live run is in EXPERIMENTS.md, E4, and in CI's compare job.
"""

from __future__ import annotations

import json
from pathlib import Path

from tools.compare_swarms import to_markdown

FIXTURE = Path(__file__).parent / "fixtures" / "comparison.json"


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


def test_across_two_swarms_the_api_calls_an_improvement_a_regression():
    """F4 reaches the API: ``expect`` is the scenario's, so a known limitation that the
    head swarm does not have reads as a regression, and one it has reads as a fix."""
    comparison = json.loads(FIXTURE.read_text(encoding="utf-8"))
    change = {s["name"]: s for s in comparison["scenarios"]}
    fixed_v = change["v_formation_from_pads"]
    assert fixed_v["change"] == "regressed"
    separation = next(m for m in fixed_v["measurements"] if m["assertion"] == "min_separation")
    assert separation["headMean"] > separation["baseMean"] + 2.0  # 0.48 m -> 3.14 m
    assert change["line_five_noisy_gps"]["change"] == "fixed"  # the lab's known limitation
