"""tools/sweep.py: one row per scenario and swarm, and one name per scenario."""

from __future__ import annotations

import shutil

import pytest
from swarm_coordination.scenarios import ReferenceSwarm

from lab_swarm.swarm import LabSwarm
from tools.sweep import sweep, to_markdown


def test_a_row_per_scenario_and_swarm_with_the_closest_approach(lab_scenarios):
    rows = sweep([str(lab_scenarios / "v_five_from_pads.yaml")], [ReferenceSwarm(), LabSwarm()], 1)
    by_swarm = {row["sut"]: row for row in rows}
    assert by_swarm["reference"]["failing"] == ["min_separation"]
    assert by_swarm["reference"]["min_separation_worst"] == pytest.approx(0.41, abs=0.01)
    assert by_swarm["lab"]["failed"] == 0
    assert "| `v_five_from_pads` | lab | 0/1 |" in to_markdown(rows)


def test_two_scenarios_with_one_name_are_refused(lab_scenarios, tmp_path):
    copy = tmp_path / "copy.yaml"
    shutil.copy(lab_scenarios / "v_after_lanes.yaml", copy)
    with pytest.raises(SystemExit, match="v_after_lanes"):
        sweep([str(lab_scenarios / "v_after_lanes.yaml"), str(copy)], [LabSwarm()], 1)
