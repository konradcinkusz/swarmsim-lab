"""The assignment behind LabSwarm's formation roles (lab_swarm/planning.py)."""

from __future__ import annotations

import itertools
import random

import pytest
from swarm_coordination.formation import line_formation, v_formation
from swarm_coordination.trajectory import Vector3

from lab_swarm.planning import formation_roles, hungarian


def _brute_force(cost: list[list[float]]) -> float:
    rows = range(len(cost))
    return min(
        sum(cost[r][c] for r, c in zip(rows, columns, strict=True))
        for columns in itertools.permutations(range(len(cost[0])), len(cost))
    )


def _total(cost: list[list[float]], columns: list[int]) -> float:
    return sum(cost[row][column] for row, column in enumerate(columns))


def test_hungarian_finds_the_cheapest_assignment_of_random_matrices():
    rng = random.Random(7)
    for _ in range(300):
        rows = rng.randint(1, 6)
        columns = rng.randint(rows, 7)
        cost = [[rng.uniform(0, 50) for _ in range(columns)] for _ in range(rows)]
        assignment = hungarian(cost)
        assert len(set(assignment)) == rows
        assert _total(cost, assignment) == pytest.approx(_brute_force(cost))


def test_hungarian_needs_a_column_for_every_row():
    with pytest.raises(ValueError):
        hungarian([[1.0], [2.0]])
    assert hungarian([]) == []


def _pads(count: int) -> dict[str, Vector3]:
    """swarmsim's spawn layout: drone_n at (0, 3(n-1), 0)."""
    return {f"drone_{n}": Vector3(0.0, 3.0 * (n - 1), 0.0) for n in range(1, count + 1)}


def test_a_v_from_a_row_of_pads_is_led_from_the_middle_and_nobody_crosses():
    roles = formation_roles(_pads(5), v_formation(4, 3.0))
    assert roles.leader == "drone_3"
    # Every slot is straight behind its drone's pad: the followers only move back.
    assert roles.slots == {
        "drone_1": Vector3(-6.0, -6.0, 0.0),
        "drone_2": Vector3(-3.0, -3.0, 0.0),
        "drone_4": Vector3(-3.0, 3.0, 0.0),
        "drone_5": Vector3(-6.0, 6.0, 0.0),
    }


def test_the_roles_need_one_drone_more_than_slots():
    with pytest.raises(ValueError):
        formation_roles(_pads(3), line_formation(3, 3.0))
