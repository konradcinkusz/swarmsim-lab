"""Measurements over a scenario's trace that swarmsim's assertions do not make.

swarmsim's assertions are a closed set (EXPERIMENTS.md, finding F7): a new kind means a
change to swarmsim and to its scenario schema. The trace is public, though, so a check
it lacks can be written in Python and run in pytest. These are the lab's.
"""

from __future__ import annotations

import math

from swarm_coordination.scenarios import Trace


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
