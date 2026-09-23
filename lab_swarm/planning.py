"""Who takes which place in a formation: an assignment, not an id order.

swarmsim's reference swarm gives the leader's place to the lowest id and the slots to
the others in id order, wherever they are. Launched from a row of pads, a V then sends a
follower straight through its neighbours (swarmsim's ``v_formation_from_pads``, an
``expect: fail``). Here the places go to the drones as a whole, by the assignment that
minimises the sum of squared distances to travel. That is the cost CAPT uses (Turpin,
Michael and Kumar, IJRR 33(1), 2014). CAPT proves it collision-free when every robot
moves in a straight line, in sync, between well-separated starts and goals.

swarmsim's drones are not in sync: a follower chases a slot that moves with its leader.
So the proof does not carry over, and the lab's scenarios measure what is left
(EXPERIMENTS.md, E2).

Pure: it imports nothing from swarmsim but ``Vector3``. tests/test_planning.py covers it.
"""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass

from swarm_coordination.trajectory import Vector3


def drone_number(drone: str) -> int:
    return int(drone.rsplit("_", 1)[-1])


def hungarian(cost: Sequence[Sequence[float]]) -> list[int]:
    """The column assigned to each row, minimising the total cost (Kuhn-Munkres, O(n²m)).

    ``cost`` has n rows and m >= n columns. The same matrix always gets the same answer.
    """
    n = len(cost)
    if n == 0:
        return []
    m = len(cost[0])
    if m < n or any(len(row) != m for row in cost):
        raise ValueError(f"cost must be n rows by m >= n columns, got {n} rows")
    inf = float("inf")
    u = [0.0] * (n + 1)  # row potentials
    v = [0.0] * (m + 1)  # column potentials
    row_of = [0] * (m + 1)  # row_of[j]: the row (1-based) holding column j; 0: none
    way = [0] * (m + 1)
    for i in range(1, n + 1):
        row_of[0] = i
        j0 = 0
        slack = [inf] * (m + 1)
        used = [False] * (m + 1)
        while True:  # grow an alternating tree from row i until it reaches a free column
            used[j0] = True
            i0, delta, j1 = row_of[j0], inf, 0
            for j in range(1, m + 1):
                if used[j]:
                    continue
                reduced = cost[i0 - 1][j - 1] - u[i0] - v[j]
                if reduced < slack[j]:
                    slack[j], way[j] = reduced, j0
                if slack[j] < delta:
                    delta, j1 = slack[j], j
            for j in range(m + 1):
                if used[j]:
                    u[row_of[j]] += delta
                    v[j] -= delta
                else:
                    slack[j] -= delta
            j0 = j1
            if row_of[j0] == 0:
                break
        while j0:  # flip the augmenting path
            j1 = way[j0]
            row_of[j0] = row_of[j1]
            j0 = j1
    assignment = [0] * n
    for j in range(1, m + 1):
        if row_of[j]:
            assignment[row_of[j] - 1] = j - 1
    return assignment


def squared_distance(a: Vector3, b: Vector3) -> float:
    d = a - b
    return d.x * d.x + d.y * d.y + d.z * d.z


def slot_assignment(
    positions: Mapping[str, Vector3], apex: Vector3, offsets: Sequence[Vector3]
) -> tuple[dict[str, Vector3], float]:
    """Each drone to one of ``offsets`` from ``apex``, by least total squared travel.

    Returns every drone's offset and the total cost in m². Needs at most as many drones
    as offsets.
    """
    drones = sorted(positions, key=drone_number)
    cost = [[squared_distance(positions[d], apex + o) for o in offsets] for d in drones]
    columns = hungarian(cost)
    total = sum(cost[row][column] for row, column in enumerate(columns))
    return {d: offsets[c] for d, c in zip(drones, columns, strict=True)}, total


@dataclass(frozen=True)
class Roles:
    leader: str
    slots: dict[str, Vector3]  # follower -> its offset from the leader (world frame)
    cost_m2: float  # total squared travel to the places


def formation_roles(positions: Mapping[str, Vector3], offsets: Sequence[Vector3]) -> Roles:
    """The leader and every follower's slot for the drones in ``positions``.

    A formation forms around its leader wherever that leader is. The leader flies the
    path, and the followers track their slots from the leader's live position. So each
    drone is tried as the leader where it stands; the other drones are assigned to the
    slots around it; and the cheapest total wins. A tie goes to the lower id.
    """
    if len(positions) != len(offsets) + 1:
        raise ValueError(f"{len(positions)} drones for a leader and {len(offsets)} slots")
    best: Roles | None = None
    for leader in sorted(positions, key=drone_number):
        others = {d: p for d, p in positions.items() if d != leader}
        slots, cost = slot_assignment(others, positions[leader], offsets)
        if best is None or cost < best.cost_m2 - 1e-9:
            best = Roles(leader, slots, cost)
    assert best is not None
    return best
