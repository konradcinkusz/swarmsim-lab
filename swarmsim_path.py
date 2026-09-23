"""Puts a swarmsim checkout's ``swarm_coordination`` package on ``sys.path``.

swarmsim's scenario runner cannot be pip-installed: it reads its schema from the
checkout's ``contracts/`` (EXPERIMENTS.md, finding F2). So the lab uses a checkout, the
way swarmsim's own GitHub Action does. The checkout is the first of these that exists:
``$SWARMSIM_DIR``, ``./swarmsim`` (CI), ``../swarmsim`` (a clone next to this one).
"""

from __future__ import annotations

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
    """The checkout, with its ``swarm_coordination`` importable; SystemExit if none."""
    found = swarmsim_dir()
    if found is None:
        raise SystemExit(
            "swarmsim checkout not found: clone konradcinkusz/swarmsim next to this "
            "repository, or set SWARMSIM_DIR (README.md, Setup)"
        )
    package = str(found / "swarm_coordination")
    if package not in sys.path:
        sys.path.insert(0, package)
    return found
