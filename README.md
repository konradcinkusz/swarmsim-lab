# swarmsim-lab

An example of using [swarmsim](https://github.com/konradcinkusz/swarmsim) from another
repository, to test a swarm that isn't swarmsim's and to change one that is.

- It flies a swarm written from scratch against swarmsim's scenarios.
- It changes swarmsim's reference swarm and measures what the change bought and cost.
- It writes its own scenarios, with its own mutation check.
- It runs them in CI through swarmsim's GitHub Action, and compares two swarms through
  swarmsim's API.

Everything is measured in swarmsim's L0 simulator. [EXPERIMENTS.md](EXPERIMENTS.md) is
the lab notebook, one experiment per commit. It ends with twelve findings about adapting
swarmsim. Eight became pull requests to swarmsim, and E5 moves the lab onto them: what
the lab had worked around, it now uses.

## Four ways to use swarmsim from outside

swarmsim flies a *system under test* (SUT) in a seeded kinematic simulation. The SUT has
two parts:

- **Ground software** takes missions and commands as the rosbridge contract's JSON, and
  reports swarm state.
- **Per-drone software** reads its autopilot and asks for setpoints, a mode and arming,
  under PX4's rules.

The two parts talk to each other only through messages the simulator can delay and
drop. Scenarios are YAML. The verdict is measured from the trace, never declared.
Anything that implements the protocol in `swarm_coordination/scenarios/sut.py` plugs in
with `--sut module:attribute`.

### 1. Test your own swarm

Implement the protocol and point the runner at it:

```python
class MySwarm:
    name = "mine"

    def ground(self, fleet): ...  # submit_mission, submit_command, step, report
    def drone(self, drone, fleet): ...  # step(now_s, observation, inbox) -> (actuation, messages)
```

```bash
pip install ../swarmsim/swarm_coordination pyyaml jsonschema
python -m swarm_coordination.scenarios run ../swarmsim/scenarios --sut my_package:MySwarm \
  --expect my_expectations.yaml --seeds 3
```

The scenario files' `expect` describes swarmsim's reference swarm, not yours. A swarm's
known limitations go in a file of its own, passed with `--expect`
([`expectations/lab.yaml`](expectations/lab.yaml) is one).

The worked example is [`lab_swarm/minimal.py`](lab_swarm/minimal.py), about 150 lines. It
flies lanes and obeys operator commands, and has no formations and no battery policy.
Against swarmsim's ten scenarios it passes the two lane scenarios, and
`operator_land_in_place` since swarmsim measures the distance travelled after the command
(F9). It fails the others exactly where it lacks something, and
[`tests/test_minimal_swarm.py`](tests/test_minimal_swarm.py) pins each failure.

### 2. Change the reference swarm

swarmsim's reference swarm is plain Python behind `ReferenceSwarm`. It is a dataclass
whose fields are the parts, so a variant is a subclass, or a `replace()` with one part
swapped, and its planning is a `planner=` function (F8).
[`lab_swarm/swarm.py`](lab_swarm/swarm.py) plugs in a planner for formations, and keeps a
supervisor subclass to average a parked drone's reports and to re-plan a leader's hand-over,
which swarmsim has no seam for; the drones fly swarmsim's controller unchanged. The planner,
[`lab_swarm/planning.py`](lab_swarm/planning.py), is pure and unit tested. It does four
things:

- chooses who leads and which slot each drone takes from where the drones are;
- forms up before flying;
- breaks ties by clearance;
- averages a parked drone's position.

Here is what that bought, closest approach in the worst of 20 seeds, reference →
LabSwarm:

| Scenario | Reference | LabSwarm |
|---|---|---|
| V from the pads (swarmsim's own `expect: fail`) | 0.48 m | **3.14 m** |
| Five-drone V in a gusty crosswind | 0.31 m | **2.43 m** |
| V superseding lanes mid-air | 0.42 m | **3.00 m** |
| The V's leader running low | 1.56 m, 58.8 s | **5.14 m, 43.7 s** |
| Line from the pads | 2.12 m | 2.44 m |
| Line from the pads in wind | 2.08 m | 1.88 m (limit 1.5 m) |

It also has costs: forming up adds 1–5 s, and a line in GPS noise keeps less margin in
its worst seeds. Both are written down, not hidden. The first design was caught by
swarmsim's own suite, and three ideas after it were measured worse and dropped.
[EXPERIMENTS.md](EXPERIMENTS.md), E2, keeps all of it.

### 3. Write your own scenarios, and check that they have teeth

[`scenarios/`](scenarios) holds the lab's eight scenarios, in swarmsim's schema. A known
weakness is written down with its reason in the swarm's expectations file, not a deleted
scenario.

A scenario that no broken swarm fails proves nothing. swarmsim checks this with its
mutation check, and a swarm brings its own mutants with `--mutants-from` (F1). The lab's
are in [`lab_swarm/mutants.py`](lab_swarm/mutants.py). Each one undoes one decision, and
the last is the reference swarm, every decision undone at once.
[`tests/test_lab_scenarios.py`](tests/test_lab_scenarios.py) and CI run swarmsim's check,
which requires two things:

- every mutant fails a scenario;
- every scenario fails a mutant.

The lab's known limitations are in [`expectations/lab.yaml`](expectations/lab.yaml), and
the reference swarm's, on the same scenarios, in
[`expectations/reference.yaml`](expectations/reference.yaml). The scenario files carry
none.

### 4. Run it in CI, and compare runs

swarmsim's root `action.yml` runs scenarios inside the caller's job and sends nothing
anywhere. Pin it to a commit:

```yaml
- uses: konradcinkusz/swarmsim@5b93cddca74ab99d411201fb0401eba7061edb04
  with:
    scenarios: scenarios
    sut: lab_swarm.swarm:LabSwarm
    expect: expectations/lab.yaml
    mutants-from: lab_swarm.mutants:MUTANTS
    seeds: "3"
```

`include-swarmsim-scenarios: "true"` adds swarmsim's own scenarios, which the action carries
(F5). CI flies them on LabSwarm in a second step, under the same expectations.

To remember and compare runs, swarmsim's API stores reports and answers
`GET /api/scenario-runs/compare`. [`tools/compare_swarms.py`](tools/compare_swarms.py)
flies one suite against two swarms, each held to its own expectations, stores both runs
and prints the API's verdict. CI's `compare` job starts a throwaway API from the swarmsim
checkout and does exactly that on every push. The comparison was built for two commits of
one swarm. Across two swarms with the scenario files' own `expect`, five of the nine
changes it reported were about the scenarios (E4). With each swarm's expectations it
reports one thing per scenario, and none of them a regression (E5).

## Setup

swarmsim's scenario runner installs with pip (F2). The lab also wants a checkout, for
swarmsim's scenarios, which are not in the package, and for its API. It finds the checkout
at `$SWARMSIM_DIR`, `./swarmsim` or `../swarmsim`. Install the runner from the same
checkout:

```bash
git clone https://github.com/konradcinkusz/swarmsim ../swarmsim
git -C ../swarmsim checkout 5b93cddca74ab99d411201fb0401eba7061edb04
pip install ../swarmsim/swarm_coordination -r requirements-dev.txt

ruff check . && pytest                        # 34 tests, about 20 s

# the lab's scenarios on LabSwarm, the way CI's Action runs them
python -m swarm_coordination.scenarios run scenarios --sut lab_swarm.swarm:LabSwarm \
  --expect expectations/lab.yaml --mutants-from lab_swarm.mutants:MUTANTS --seeds 3

# closest approach and mission time, over many seeds, for several swarms
python -m tools.sweep --sut reference --sut lab_swarm.swarm:LabSwarm --seeds 20 explore scenarios

# two swarms through swarmsim's API (needs the .NET 8 SDK)
dotnet run --project ../swarmsim/backend/src/SwarmApi.Api --urls http://127.0.0.1:5080 &
python -m tools.compare_swarms --api http://127.0.0.1:5080 \
  --base reference --base-expect expectations/reference.yaml \
  --head lab_swarm.swarm:LabSwarm --head-expect expectations/lab.yaml \
  scenarios ../swarmsim/scenarios
```

## Layout

| Path | What it is |
|---|---|
| `lab_swarm/minimal.py` | E1: a swarm from scratch against the SUT protocol |
| `lab_swarm/planning.py` | E2: who leads and who takes which slot. Pure, with the Hungarian algorithm and exact form-up clearances |
| `lab_swarm/swarm.py` | E2, E5: `LabSwarm`, the reference swarm with a formation planner and `FormationAwareSupervisor` |
| `lab_swarm/mutants.py` | E3, E5: `LabSwarm` with one decision undone, per mutant, in the form swarmsim's `--mutants-from` takes |
| `expectations/` | What LabSwarm and the reference swarm are each expected to fail (E5) |
| `scenarios/` | The lab's scenarios, run by CI through swarmsim's Action |
| `explore/` | E0's exploratory scenarios that E3 did not promote to `scenarios/` |
| `tools/sweep.py` | Many seeds, several swarms: worst and median closest approach, mission time |
| `tools/compare_swarms.py` | E4, E5: two swarms, each held to its expectations, stored and compared by swarmsim's API |
| `tests/` | Unit tests, the lab's mutation check, and swarmsim's suite on both swarms |
| `swarmsim_path.py` | Finds the swarmsim checkout; imports its runner unless it is installed |
| `EXPERIMENTS.md` | The notebook: every number, every dead end, twelve findings |

## Status

This is a lab project. It has no users and no deployment. Its purpose is to find out
what adapting swarmsim takes and to write that down. Nothing in it has flown in SITL,
because swarmsim has no such path for another swarm yet (F6).
