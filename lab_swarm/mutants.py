"""Deliberately broken LabSwarms. Each one undoes one decision that E2 made.

swarmsim's own mutation check cannot mutate a swarm it did not write: ``--mutants``
refuses ``--sut`` (EXPERIMENTS.md, finding F1). So the lab keeps its mutants here, as
swarmsim keeps its own in ``scenarios/mutants.py``. tests/test_lab_scenarios.py requires
the lab's scenarios to fail every one of them.
"""

from __future__ import annotations

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

    start = MissionSupervisor.start


MUTANTS: dict[str, LabSwarm] = {
    name: LabSwarm(name=f"lab/{name}", supervisor_cls=supervisor)
    for name, supervisor in {
        "no_form_up": NoFormUp,
        "latest_position": LatestPosition,
        "closest_tie": ClosestTie,
        "reference_handover": ReferenceHandover,
        "id_order_roles": IdOrderRoles,
    }.items()
}

# Each by name too, for --sut lab_swarm.mutants:<name> (tools.sweep, the scenario runner).
no_form_up = MUTANTS["no_form_up"]
latest_position = MUTANTS["latest_position"]
closest_tie = MUTANTS["closest_tie"]
reference_handover = MUTANTS["reference_handover"]
id_order_roles = MUTANTS["id_order_roles"]
