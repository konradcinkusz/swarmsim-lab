"""Measurements over a scenario's trace that swarmsim's assertions do not make.

swarmsim's assertions are a closed set (EXPERIMENTS.md, finding F7): a new kind means a
change to swarmsim and to its scenario schema. The trace is public, though, so a check
it lacks can be written in Python and run in pytest. These are the lab's.
"""

from __future__ import annotations

import math

from swarm_coordination.mission_planning import FORMATIONS
from swarm_coordination.scenarios import Trace

from .planning import slot_assignment


def _horizontal(a, b) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def travel_after(trace: Trace, t_s: float) -> dict[str, float]:
    """How far each drone moved horizontally from time ``t_s`` to the end of the run.

    "Stop where you are and land" is a claim about this number. It is relative to where
    the drone was when told, which an absolute final_position cannot express (F9).
    """
    start = next(f for f in trace.frames if f.t_s >= t_s - 1e-9)
    end = trace.frames[-1]
    where = {s.drone_id: s.position for s in start.drones}
    return {s.drone_id: _horizontal(where[s.drone_id], s.position) for s in end.drones}


def ever_airborne(trace: Trace) -> set[str]:
    """The drones that left the ground at any point of the run."""
    return {s.drone_id for f in trace.frames for s in f.drones if s.airborne}


def formation_shape_error(trace: Trace, from_s: float = 0.0) -> tuple[float | None, float | None]:
    """How far the swarm is from the shape of its last formation mission, whoever leads.

    swarmsim's formation_error assumes the lowest id leads and that the others hold the
    slots in id order, which is the reference swarm's policy (F3). This measure lets the
    swarm decide. In each frame it tries every flying drone as the leader, matches the
    others to the slots around it by least total distance, and keeps the best fit's
    worst follower.

    A frame counts when exactly the mission's ``drone_count`` drones are airborne in
    OFFBOARD, meaning the formation is flying: not forming up on the ground, not landing.
    Returns the worst error over the frames from ``from_s``, and when it happened. It
    returns ``(None, None)`` when no frame counts, and a caller should treat that as a
    failure, not a pass (F10).
    """
    formations = [p for _, _, p in trace.missions if p["type"] == "formation"]
    if not formations:
        return None, None
    mission = formations[-1]
    count = mission["drone_count"]
    offsets = FORMATIONS[mission["formation"]](count - 1, mission["spacing_m"])
    worst: tuple[float | None, float | None] = (None, None)
    for frame in trace.frames:
        if frame.t_s < from_s - 1e-9:
            continue
        flying = {
            s.drone_id: s.position for s in frame.drones if s.airborne and s.mode == "OFFBOARD"
        }
        if len(flying) != count:
            continue
        best = None
        for leader, apex in flying.items():
            others = {d: p for d, p in flying.items() if d != leader}
            slots, _ = slot_assignment(others, apex, offsets)
            error = max((others[d].distance_to(apex + o) for d, o in slots.items()), default=0.0)
            best = error if best is None else min(best, error)
        if worst[0] is None or best > worst[0]:
            worst = (best, frame.t_s)
    return worst
