"""Puts the swarmsim checkout the lab is tested against on the path (swarmsim_path.py)."""

from __future__ import annotations

from pathlib import Path

import pytest

from swarmsim_path import LAB, use_swarmsim

try:
    SWARMSIM = use_swarmsim()
except SystemExit as missing:
    pytest.exit(str(missing), returncode=4)


@pytest.fixture(scope="session")
def swarmsim_scenarios() -> Path:
    """swarmsim's own scenario suite, from the checkout under test."""
    return SWARMSIM / "scenarios"


@pytest.fixture(scope="session")
def lab_scenarios() -> Path:
    return LAB / "scenarios"
