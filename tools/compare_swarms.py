"""Fly one scenario suite against two swarms, store both runs in a SwarmApi, compare them.

swarmsim decides a verdict where the scenarios run. Remembering and comparing runs is
the job of its API (swarmsim's ADR-0011). The comparison is the one the API makes
between two commits of one swarm: per scenario, regressed, fixed, changed or unchanged,
with every assertion's mean measurement in both runs. Here it is pointed at two swarms
instead:

    python -m tools.compare_swarms --api http://127.0.0.1:5080 \\
        --base reference --head lab_swarm.swarm:LabSwarm --seeds 3 scenarios

The runs are stored whole, and so is each report under ``reports/``. A token for an API
in Enforced mode comes from ``SWARMSIM_API_TOKEN``, as for swarmsim's own ``--upload``.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.parse
import urllib.request
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from swarmsim_path import LAB, use_swarmsim  # noqa: E402

use_swarmsim()
from swarm_coordination.scenarios import run_suite  # noqa: E402
from swarm_coordination.scenarios.runner import to_json  # noqa: E402
from swarm_coordination.scenarios.upload import TOKEN_VARIABLE, upload_report  # noqa: E402

from tools.sweep import load_sut  # noqa: E402


def store(api: str, paths: list[str], reference: str, seeds: list[int], token: str | None):
    """Runs the suite against one swarm, stores the report, and returns the stored run."""
    sut = load_sut(reference)
    report = to_json(run_suite(paths, sut, seeds))
    out = LAB / "reports" / f"{sut.name.replace('/', '_')}.json"
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(report, indent=2), encoding="utf-8")
    return upload_report(api, report, label=f"{sut.name} ({reference})", token=token)


def compare(api: str, base_id: str, head_id: str, token: str | None) -> dict:
    query = urllib.parse.urlencode({"base": base_id, "head": head_id})
    request = urllib.request.Request(f"{api.rstrip('/')}/api/scenario-runs/compare?{query}")
    request.add_header("Accept", "application/json")
    if token:
        request.add_header("Authorization", f"Bearer {token}")
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.loads(response.read())


def _mean(value) -> str:
    return "—" if value is None else f"{value:.2f}"


def to_markdown(comparison: dict) -> str:
    base, head = comparison["base"], comparison["head"]
    lines = [
        f"Base: **{base['sut']}** ({base['counts']}), head: **{head['sut']}** ({head['counts']})",
        "",
        "| Scenario | Base | Head | Change | Closest approach, mean (m) |",
        "|---|---|---|---|---|",
    ]
    for scenario in comparison["scenarios"]:
        separation = next(
            (m for m in scenario["measurements"] if m["assertion"] == "min_separation"), None
        )
        closest = "—"
        if separation is not None:
            closest = f"{_mean(separation['baseMean'])} → {_mean(separation['headMean'])}"
        lines.append(
            f"| `{scenario['name']}` | {scenario['baseOutcome']} | {scenario['headOutcome']} "
            f"| **{scenario['change']}** | {closest} |"
        )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("paths", nargs="+", help="scenario files or directories")
    parser.add_argument("--api", required=True, help="a SwarmApi.Api base URL")
    parser.add_argument("--base", default="reference", help="reference or MODULE:ATTR")
    parser.add_argument("--head", required=True, help="reference or MODULE:ATTR")
    parser.add_argument("--seeds", type=int, default=3)
    args = parser.parse_args(argv)

    token = os.environ.get(TOKEN_VARIABLE)
    seeds = list(range(1, args.seeds + 1))
    base = store(args.api, args.paths, args.base, seeds, token)
    head = store(args.api, args.paths, args.head, seeds, token)
    comparison = compare(args.api, base["id"], head["id"], token)
    (LAB / "reports" / "comparison.json").write_text(
        json.dumps(comparison, indent=2), encoding="utf-8"
    )
    print(to_markdown(comparison))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
