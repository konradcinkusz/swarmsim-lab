"""Finds swarmsim's checkout, and makes its ``swarm_coordination`` importable.

swarmsim's scenario runner could not be pip-installed: it read its schema from the
checkout's ``contracts/`` (EXPERIMENTS.md, finding F2, fixed in swarmsim #37). Now
``pip install <checkout>/swarm_coordination`` is enough, and CI does that. The lab still
wants the checkout, for swarmsim's *scenarios*, which are not part of the package, and
for its API. The checkout is the first of these that exists: ``$SWARMSIM_DIR``,
``./swarmsim`` (CI), ``../swarmsim`` (a clone next to this one).

If ``swarm_coordination`` is already importable, because it is installed, it is left
alone: install it from the checkout you are testing against, so that the runner and the
scenarios are one version. Otherwise the checkout's copy goes on ``sys.path``.
"""

from __future__ import annotations

import importlib.util
import os
import sys
from pathlib import Path

LAB = Path(__file__).resolve().parent


def swarmsim_dir() -> Path | None:
    for candidate in (os.environ.get("SWARMSIM_DIR"), LAB / "swarmsim", LAB.parent / "swarmsim"):
        if candidate and (Path(candidate) / "swarm_coordination" / "swarm_coordination").is_dir():
            return Path(candidate).resolve()
    return None


def use_swarmsim() -> Path:
    """The checkout, with ``swarm_coordination`` importable; SystemExit if there is no checkout."""
    found = swarmsim_dir()
    if found is None:
        raise SystemExit(
            "swarmsim checkout not found: clone konradcinkusz/swarmsim next to this "
            "repository, or set SWARMSIM_DIR (README.md, Setup)"
        )
    if importlib.util.find_spec("swarm_coordination") is None:  # not installed
        sys.path.insert(0, str(found / "swarm_coordination"))
    return found
