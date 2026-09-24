"""Who takes which place in a formation: an assignment, not an id order.

swarmsim's reference swarm gives the leader's place to the lowest id and the slots to
the others in id order, wherever they are. Launched from a row of pads, a V then sends a
follower straight through its neighbours (swarmsim's ``v_formation_from_pads``, an
``expect: fail``).

Here each drone is tried as the leader, where it stands. The others take the slots
around it, and the cheapest plan wins. The cost is the total horizontal distance to
travel. The formation forms up there: LabSwarm's leader first climbs straight up
(swarm.py). Mirror-image plans often tie on cost, and a tie goes to the plan that keeps
the drones farthest apart while they form up.

Each choice replaced something that the scenarios measured going wrong (EXPERIMENTS.md,
E2):

* **Distance, not CAPT's squared distance** (Turpin, Michael and Kumar, IJRR 2014).
  Consider a line formed from a row of pads at right angles to it. Every assignment has
  the same squared cost, so GPS noise picked the order. swarmsim's
  ``formation_line_in_wind`` caught it at 1.31 m (E2a).
* **Forming up where the leader stands, not on the way to the first waypoint.** A
  leader taken from the middle of the row drags a forming line sideways, and its
  neighbour passes it at 1.76 m.
* **The tie-break.** Of four equally cheap lines, two pass two followers as close as
  the leader's neighbour: 1.88 m in flight, against 2.44 m for the other two.

Pure: it imports nothing from swarmsim but ``Vector3``. tests/test_planning.py covers it.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
from itertools import permutations

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


def distance(a: Vector3, b: Vector3) -> float:
    """Horizontal distance: every drone climbs to the formation's altitude alike."""
    return math.hypot(a.x - b.x, a.y - b.y)


def squared_distance(a: Vector3, b: Vector3) -> float:
    """CAPT's cost, which E2a used; kept for the mutant that proves it is caught."""
    return (a.x - b.x) ** 2 + (a.y - b.y) ** 2


Cost = Callable[[Vector3, Vector3], float]


def slot_assignment(
    positions: Mapping[str, Vector3],
    apex: Vector3,
    offsets: Sequence[Vector3],
    cost: Cost = distance,
) -> tuple[dict[str, Vector3], float]:
    """Each drone to one of ``offsets`` from ``apex``, by least total ``cost``.

    Returns every drone's offset and the total. Needs at most as many drones as offsets.
    """
    drones = sorted(positions, key=drone_number)
    matrix = [[cost(positions[d], apex + o) for o in offsets] for d in drones]
    columns = hungarian(matrix)
    total = sum(matrix[row][column] for row, column in enumerate(columns))
    return {d: offsets[c] for d, c in zip(drones, columns, strict=True)}, total


def transition_clearances(
    starts: Mapping[str, Vector3], goals: Mapping[str, Vector3]
) -> tuple[float, ...]:
    """How close each pair of drones comes if all fly straight to their goals, arriving
    together, smallest first.

    It is exact, not sampled. Each pair's gap changes linearly, so the gap's length is
    smallest at a point on the segment that has a closed form.
    """
    drones = sorted(starts, key=drone_number)
    gaps = []
    for i, a in enumerate(drones):
        for b in drones[i + 1 :]:
            r0 = starts[a] - starts[b]
            dr = (goals[a] - goals[b]) - r0
            length2 = dr.x * dr.x + dr.y * dr.y + dr.z * dr.z
            f = 0.0
            if length2 > 0.0:
                f = -(r0.x * dr.x + r0.y * dr.y + r0.z * dr.z) / length2
                f = min(1.0, max(0.0, f))
            gaps.append((r0 + dr.scale(f)).norm())
    return tuple(sorted(gaps))


def transition_clearance(starts: Mapping[str, Vector3], goals: Mapping[str, Vector3]) -> float:
    """The closest any two drones come on the way (see ``transition_clearances``)."""
    return min(transition_clearances(starts, goals), default=math.inf)


@dataclass(frozen=True)
class Roles:
    leader: str
    slots: dict[str, Vector3]  # follower -> its offset from the leader (world frame)
    cost: float  # the assignment's total cost
    clearances: tuple[float, ...]  # transition_clearances of getting everyone to their places

    @property
    def clearance_m(self) -> float:
        return self.clearances[0] if self.clearances else math.inf


EXHAUSTIVE_MAX_FOLLOWERS = 6  # 7 leaders x 720 orders; beyond that, the cheapest per leader


def formation_plans(
    positions: Mapping[str, Vector3],
    offsets: Sequence[Vector3],
    cost: Cost = distance,
    leaders: Sequence[str] | None = None,
) -> list[Roles]:
    """Ways to fill the formation, each with its cost and its form-up clearance.

    The formation forms up around its leader where the leader stands. So each plan's
    clearance is the transition's, from where everyone is to the slots around the
    leader. With up to ``EXHAUSTIVE_MAX_FOLLOWERS`` followers, every order is a plan.
    Beyond that, only the cheapest order for each leader is, by the Hungarian algorithm.
    """
    if len(positions) != len(offsets) + 1:
        raise ValueError(f"{len(positions)} drones for a leader and {len(offsets)} slots")
    plans = []
    for leader in leaders or sorted(positions, key=drone_number):
        apex = positions[leader]
        others = sorted((d for d in positions if d != leader), key=drone_number)
        if len(others) <= EXHAUSTIVE_MAX_FOLLOWERS:
            orders = [dict(zip(others, o, strict=True)) for o in permutations(offsets)]
        else:
            orders = [slot_assignment({d: positions[d] for d in others}, apex, offsets, cost)[0]]
        for slots in orders:
            goals = {leader: apex, **{d: apex + o for d, o in slots.items()}}
            total = sum(cost(positions[d], goals[d]) for d in others)
            plans.append(Roles(leader, slots, total, transition_clearances(positions, goals)))
    return plans


def formation_roles(
    positions: Mapping[str, Vector3],
    offsets: Sequence[Vector3],
    cost: Cost = distance,
    leaders: Sequence[str] | None = None,
    prefer: Callable = max,
) -> Roles:
    """The cheapest plan.

    Plans often tie exactly: a row of pads is symmetric, so every plan has a mirror
    image. Mirror images are not equally safe. In a line formed from five pads, the four
    cheapest plans all keep 2.12 m at the leader's neighbour. Two of them also pass two
    followers that close, and the other two keep those followers 2.68 m apart. So a tie
    goes to the plan that keeps the drones farthest apart while they form up: the
    closest pair decides first, then the next closest, and so on (leximin). Any tie left
    after that goes to the lower-numbered leader. (``prefer=min`` turns the tie-break
    round, for the mutant that proves it matters.)
    """
    plans = formation_plans(positions, offsets, cost, leaders)
    cheapest = min(round(p.cost, 6) for p in plans)
    tied = [p for p in plans if round(p.cost, 6) == cheapest]
    return prefer(
        tied, key=lambda p: (tuple(round(c, 6) for c in p.clearances), -drone_number(p.leader))
    )
