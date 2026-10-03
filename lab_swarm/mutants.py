"""Deliberately broken LabSwarms. Each one undoes one decision that E2 made.

swarmsim's mutation check flies every scenario against broken swarms and fails a scenario
that none of them fails, and a broken swarm that every scenario passes. It used to refuse
a swarm it did not write (``--mutants`` with ``--sut``, EXPERIMENTS.md finding F1), so the
lab ran the check itself. Now ``MUTANTS`` is what swarmsim's ``--mutants-from`` takes, and
tests/test_lab_scenarios.py and CI use swarmsim's check.

The last mutant is the reference swarm: every decision undone at once. A scenario for a
lab swarm that the reference swarm fails (the scenarios LabSwarm fixed) has teeth against
it, and no single decision need own it.
"""

from __future__ import annotations

from swarm_coordination.scenarios.mutants import Mutant
from swarm_coordination.scenarios.sut import ReferenceSwarm
from swarm_coordination.supervisor import MissionSupervisor

from .swarm import FormationAwareSupervisor, LabSwarm


class NoFormUp(FormationAwareSupervisor):
    """E2a's start: the leader heads for the first waypoint while its followers form up."""

    form_up = False


class LatestPosition(FormationAwareSupervisor):
    """Plans from each drone's latest report, GPS noise and all."""

    average_parked = False


class ClosestTie(FormationAwareSupervisor):
    """Among equally cheap plans, the one whose followers pass closest to each other."""

    prefer = staticmethod(min)


class ReferenceHandover(FormationAwareSupervisor):
    """swarmsim's hand-over: the idle drone on its pad becomes the leader."""

    _hand_over = MissionSupervisor._hand_over


class IdOrderRoles(FormationAwareSupervisor):
    """swarmsim's start: the lowest id leads, the others take the slots in id order."""

    plans_roles = False


def _undone(name: str, supervisor: type[FormationAwareSupervisor]) -> Mutant:
    breaks = " ".join((supervisor.__doc__ or name).split())
    return Mutant(name, breaks, LabSwarm(name=f"lab/{name}", supervisor_cls=supervisor))


MUTANTS: tuple[Mutant, ...] = (
    _undone("no_form_up", NoFormUp),
    _undone("latest_position", LatestPosition),
    _undone("closest_tie", ClosestTie),
    _undone("reference_handover", ReferenceHandover),
    _undone("id_order_roles", IdOrderRoles),
    Mutant("reference", "every decision undone at once: swarmsim's own swarm", ReferenceSwarm()),
)

# Each by name too, for --sut lab_swarm.mutants:<name> (tools.sweep, the scenario runner).
BY_NAME = {m.name: m.sut for m in MUTANTS}
no_form_up = BY_NAME["no_form_up"]
latest_position = BY_NAME["latest_position"]
closest_tie = BY_NAME["closest_tie"]
reference_handover = BY_NAME["reference_handover"]
id_order_roles = BY_NAME["id_order_roles"]
