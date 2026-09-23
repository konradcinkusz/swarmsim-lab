"""Fly scenarios against several swarms over many seeds, and tabulate what matters.

The scenario runner's report answers "did it pass?". Choosing between designs needs
more than that: how close the drones came in the worst seed and in the median seed, and
how long the missions took. This prints one Markdown row per scenario and swarm:

    python -m tools.sweep --sut reference --sut lab_swarm.swarm:LabSwarm \\
        --seeds 10 explore scenarios

``reference`` is swarmsim's own swarm. Any other ``--sut`` is MODULE:ATTRIBUTE, loaded
the same way as the runner's ``--sut`` does it.
"""

from __future__ import annotations

import argparse
import importlib
import statistics
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from swarmsim_path import use_swarmsim  # noqa: E402

use_swarmsim()
from swarm_coordination.scenarios import load_scenario, run_scenario  # noqa: E402
from swarm_coordination.scenarios.spec import discover  # noqa: E402
from swarm_coordination.scenarios.sut import ReferenceSwarm  # noqa: E402


def load_sut(reference: str):
    if reference == "reference":
        return ReferenceSwarm()
    module, _, attribute = reference.partition(":")
    target = getattr(importlib.import_module(module), attribute)
    return target() if isinstance(target, type) else target


def sweep(paths: list[str], suts: list, seeds: int, first_seed: int = 1) -> list[dict]:
    rows = []
    for path in discover(paths):
        spec = load_scenario(path)
        for sut in suts:
            separations, times, failed, kinds = [], [], 0, set()
            for seed in range(first_seed, first_seed + seeds):
                verdict, _ = run_scenario(spec, sut, seed)
                by_kind = {o.assertion: o for o in verdict.outcomes}
                separation = by_kind.get("min_separation")
                if separation is not None and separation.measured is not None:
                    separations.append(separation.measured)
                completes = by_kind.get("mission_completes")
                if completes is not None and completes.measured is not None:
                    times.append(completes.measured)
                if not verdict.passed:
                    failed += 1
                    kinds |= {o.assertion for o in verdict.outcomes if not o.passed}
            rows.append(
                {
                    "scenario": spec.name,
                    "sut": sut.name,
                    "seeds": seeds,
                    "failed": failed,
                    "failing": sorted(kinds),
                    "min_separation_worst": min(separations, default=None),
                    "min_separation_median": (
                        statistics.median(separations) if separations else None
                    ),
                    "completes_s": (min(times), max(times)) if times else None,
                }
            )
    return rows


def _m(value: float | None) -> str:
    return "—" if value is None else f"{value:.2f}"


def to_markdown(rows: list[dict]) -> str:
    lines = [
        "| Scenario | Swarm | Failed seeds | Closest approach, worst / median (m) "
        "| Mission time (s) | Failing assertions |",
        "|---|---|---|---|---|---|",
    ]
    for r in rows:
        times = r["completes_s"]
        took = "never" if times is None else f"{times[0]:.1f}–{times[1]:.1f}"
        if times is not None and times[0] == times[1]:
            took = f"{times[0]:.1f}"
        lines.append(
            f"| `{r['scenario']}` | {r['sut']} | {r['failed']}/{r['seeds']} "
            f"| {_m(r['min_separation_worst'])} / {_m(r['min_separation_median'])} "
            f"| {took} | {', '.join(r['failing']) or '—'} |"
        )
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("paths", nargs="+", help="scenario files or directories")
    parser.add_argument("--sut", action="append", required=True, help="reference or MODULE:ATTR")
    parser.add_argument("--seeds", type=int, default=10)
    parser.add_argument("--seed", type=int, default=1, help="the first seed")
    args = parser.parse_args(argv)
    print(to_markdown(sweep(args.paths, [load_sut(s) for s in args.sut], args.seeds, args.seed)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
