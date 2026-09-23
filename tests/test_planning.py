"""The planning behind LabSwarm's formation roles (lab_swarm/planning.py)."""

from __future__ import annotations

import itertools
import random

import pytest
from swarm_coordination.formation import line_formation, v_formation
from swarm_coordination.trajectory import Vector3

from lab_swarm.planning import (
    EXHAUSTIVE_MAX_FOLLOWERS,
    formation_plans,
    formation_roles,
    hungarian,
    transition_clearance,
    transition_clearances,
)


def _brute_force(cost: list[list[float]]) -> float:
    rows = range(len(cost))
    return min(
        sum(cost[r][c] for r, c in zip(rows, columns, strict=True))
        for columns in itertools.permutations(range(len(cost[0])), len(cost))
    )


def test_hungarian_finds_the_cheapest_assignment_of_random_matrices():
    rng = random.Random(7)
    for _ in range(300):
        rows = rng.randint(1, 6)
        columns = rng.randint(rows, 7)
        cost = [[rng.uniform(0, 50) for _ in range(columns)] for _ in range(rows)]
        assignment = hungarian(cost)
        assert len(set(assignment)) == rows
        total = sum(cost[row][column] for row, column in enumerate(assignment))
        assert total == pytest.approx(_brute_force(cost))


def test_hungarian_needs_a_column_for_every_row():
    with pytest.raises(ValueError):
        hungarian([[1.0], [2.0]])
    assert hungarian([]) == []


def _v(x: float, y: float, z: float = 0.0) -> Vector3:
    return Vector3(x, y, z)


def test_two_drones_swapping_places_meet_halfway():
    starts = {"drone_1": _v(0, 0), "drone_2": _v(4, 0)}
    goals = {"drone_1": _v(4, 0), "drone_2": _v(0, 0)}
    assert transition_clearance(starts, goals) == pytest.approx(0.0)


def test_parallel_moves_keep_their_distance_and_the_closest_pair_comes_first():
    starts = {"drone_1": _v(0, 0), "drone_2": _v(0, 3), "drone_3": _v(0, 9)}
    goals = {d: p + _v(-5, 0) for d, p in starts.items()}
    assert transition_clearances(starts, goals) == pytest.approx((3.0, 6.0, 9.0))


def _pads(count: int) -> dict[str, Vector3]:
    """swarmsim's spawn layout: drone_n at (0, 3(n-1), 0)."""
    return {f"drone_{n}": _v(0.0, 3.0 * (n - 1)) for n in range(1, count + 1)}


def test_every_leader_and_every_order_is_a_plan_up_to_the_exhaustive_limit():
    assert len(formation_plans(_pads(5), v_formation(4, 3.0))) == 5 * 24
    big = EXHAUSTIVE_MAX_FOLLOWERS + 2
    plans = formation_plans(_pads(big + 1), line_formation(big, 3.0))
    assert len(plans) == big + 1  # beyond it: the Hungarian's order, one per leader


def test_a_v_from_a_row_of_pads_is_led_from_the_middle_and_every_follower_moves_back():
    roles = formation_roles(_pads(5), v_formation(4, 3.0))
    assert roles.leader == "drone_3"
    assert roles.slots == {
        "drone_1": _v(-6.0, -6.0),
        "drone_2": _v(-3.0, -3.0),
        "drone_4": _v(-3.0, 3.0),
        "drone_5": _v(-6.0, 6.0),
    }
    assert roles.clearance_m == pytest.approx(3.0)


def test_equally_cheap_lines_go_to_the_plan_that_keeps_followers_apart():
    """Four plans tie for a line from five pads. Every one passes the leader's neighbour
    at 2.12 m, but only two keep the next-closest pair 2.68 m apart. The mutant's
    ``prefer=min`` gets one of the other two, whose next pair is also at 2.12 m."""
    line = line_formation(4, 3.0)
    chosen = formation_roles(_pads(5), line)
    worst = formation_roles(_pads(5), line, prefer=min)
    assert chosen.cost == pytest.approx(worst.cost)
    assert chosen.clearances[:2] == pytest.approx((2.121, 2.683), abs=1e-3)
    assert worst.clearances[:2] == pytest.approx((2.121, 2.121), abs=1e-3)
    # The two near neighbours take the two near slots.
    assert {chosen.slots["drone_2"].x, chosen.slots["drone_4"].x} == {-3.0, -6.0}


def test_a_given_leader_keeps_the_lead():
    roles = formation_roles(_pads(4), v_formation(3, 3.0), leaders=["drone_1"])
    assert roles.leader == "drone_1"


def test_the_roles_need_one_drone_more_than_slots():
    with pytest.raises(ValueError):
        formation_roles(_pads(3), line_formation(3, 3.0))
