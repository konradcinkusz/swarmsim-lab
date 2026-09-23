# Lab notebook

Every number here was measured in swarmsim's L0 simulator at swarmsim commit
[`56e150b`](https://github.com/konradcinkusz/swarmsim/commit/56e150be669477db23a274a835503d41e65c8ee8),
the one `.github/workflows/ci.yml` pins. **Closest approach** is the smallest distance
between two airborne drones, from swarmsim's `min_separation` assertion. "Worst" is the
worst seed and "median" the median seed. A ✗ marks a failed assertion.

Each experiment is one commit, so its numbers can be rerun from that commit (see
`git log`):

```bash
python -m tools.sweep --sut reference --sut lab_swarm.swarm:LabSwarm --seeds 20 explore scenarios
```

The E2 design sweep is the exception, and its section says why.

## E0: the reference swarm under stress

E0 wrote nine scenarios that swarmsim's own suite does not have. E3 promoted four of
them to `scenarios/`; `explore/` keeps the other five. They cover:

- missions that supersede one another mid-air;
- larger formations;
- a battery fault inside a formation;
- GPS noise and wind.

Here is swarmsim's reference swarm on them, over 20 seeds:

| Scenario | Closest approach, worst / median | Mission time | |
|---|---|---|---|
| `v_after_lanes`: a V supersedes lanes mid-air | 0.42 / 0.42 m | 35.9 s | ✗ |
| `v_five_from_pads`: five drones form a V from the pads | 0.41 / 0.41 m | 36.8 s | ✗ |
| `v_five_in_wind`: the same in a gusty crosswind, GPS noise 0.3 m | 0.31 / 0.41 m | 36.9–64.0 s | ✗ |
| `v_leader_low_battery`: the V's leader (drone_1) goes low at 20 s | 1.56 / 1.56 m | 58.8 s | ✗ |
| `v_middle_low_battery`: drone_2 goes low instead | 5.14 / 5.14 m | 41.8 s | |
| `line_three_from_pads`: a line of three, GPS noise 0.3 m | 2.11 / 2.12 m | 35.8–37.6 s | |
| `line_after_lanes`: a line supersedes lanes mid-air | 2.13 / 2.13 m | 35.9 s | |
| `hold_then_new_mission`: hold, then a new mission | 2.13 / 2.13 m | 40.1 s | |
| `tight_lanes_gps_noise`: lanes 2 m apart, GPS noise 0.5 m | 1.47 / 1.70 m | 30.2–32.9 s | |

E3 moved `v_after_lanes` and `v_five_from_pads` to `scenarios/` unchanged. It also moved
`v_leader_low_battery` and `v_middle_low_battery`, renamed `v_low_battery_drone_1` and
`v_low_battery_drone_2` (E3 says why). E2's tables use the names above.

The first four fail for one of two reasons:

- **The reference swarm plans formations by id.** drone_1 leads, and the others take
  the slots in order, wherever they are. That is swarmsim's own `v_formation_from_pads`
  (`expect: fail`), and it happens again in every V formed from a row, whether the row
  is on the pads or in mid-air.
- **The hand-over sends the formation home.** The fourth is a different mechanism. When
  the leader's battery drops, the idle drone on its pad becomes the leader, and every
  follower turns back towards that pad.

## E1: a swarm written from scratch

`lab_swarm/minimal.py` is about 150 lines and implements swarmsim's system-under-test
protocol directly. It reuses nothing of swarmsim's swarm, only the types the simulator
exchanges with a swarm. The protocol asks for surprisingly little:

- The ground software:
  - takes the rosbridge contract's JSON payloads;
  - talks to the drones only through messages the simulator can delay and drop;
  - reports `{"mission": {"id", "complete"}}`.
- Each drone's software reads a noisy position in its *own* local frame, plus armed,
  mode and battery. It asks for setpoints, a mode and arming. PX4's rules apply: there
  is no OFFBOARD and no arming without a stream of setpoints.

On swarmsim's ten scenarios (seeds 1–2) it passes `waypoint_lanes` and
`scale_ten_lanes`. The rest fail on assertions that `tests/test_minimal_swarm.py` pins
one by one:

| Scenario | Fails | Why |
|---|---|---|
| `formation_line`, `formation_line_in_wind`, `follower_comms_blip` | `mission_completes` | it ignores formation missions |
| `v_formation_from_pads` | `mission_completes` (an xfail) | likewise |
| `low_battery_handover`, `plan_around_low_battery` | battery and position checks | it has no battery policy |
| `follower_jammed_goes_home` | `final_position` only | nobody takes off, and four of five assertions pass anyway (**F9**) |
| `operator_land_in_place` | `final_position` | it *does* land in place, but not where the reference swarm is (**F9**) |

The last row is the interesting one. MinimalSwarm cruises at PX4's 5 m/s, so at 15 s,
when "land" comes, it is at x = 42 m. It lands 0.5 m from there. The scenario checks
that drone_1 ends within 6 m of x = 16 m. That is where the reference swarm, cruising
at 2 m/s, has got to. `lab_swarm.metrics.travel_after` measures the intent, "stop where
the command found you". Both swarms meet it, by less than 1 m.

## E2: formation roles from where the drones are

LabSwarm is swarmsim's reference swarm with one part replaced: the ground's
`MissionSupervisor`. The drones run swarmsim's `DroneController` unchanged.

### E2a: CAPT's assignment, anchored at the leader

The first design tried each drone as the leader where it stands. It assigned the others
to the slots around it by the least sum of *squared* distances, using the Hungarian
algorithm. That is the cost CAPT uses (Turpin, Michael and Kumar, IJRR 2014), and CAPT
proves it collision-free for synchronised straight-line moves. A leader low on battery
is succeeded by one of its followers.

Against the reference, over 10 seeds:

- v_formation_from_pads: 0.48 → 2.81 m
- v_five_from_pads: 0.41 → 2.58 m
- v_after_lanes: 0.42 → 2.97 m
- v_leader_low_battery: 1.56 → 3.90 m, and 58.8 → 42.1 s

swarmsim's own `formation_line_in_wind` caught a regression: on seed 1, 1.31 m against a
1.5 m limit, where the reference keeps 2.08 m. The cause is exact ties. A line formed
from a row of pads at right angles to it gives *every* assignment the same squared cost.
For five drones that is 360 m², for all 24 assignments (enumerated). So the GPS noise
chose the order.

### The design sweep

Four variants on eleven scenarios, 10 seeds:

- **Anchor.** A: the formation is anchored at each candidate leader where it stands.
  B: it is anchored at the first waypoint, and the leader's own travel counts.
- **Cost.** Plain distance, or squared distance.

The figures are the closest approach in the worst seed. A ✗ is a `min_separation`
failure:

| Scenario | A-plain | A-squared | B-plain | B-squared | reference |
|---|---|---|---|---|---|
| `formation_line` | 1.76 | 1.82 | 2.12 | 2.12 | 2.12 |
| `formation_line_in_wind` | 1.69 | 1.31 ✗ | 2.08 | 1.31 ✗ | 2.08 |
| `v_formation_from_pads` | 2.81 | 2.81 | 1.77 ✗ | 2.81 | 0.48 ✗ |
| `line_three_from_pads` | 1.75 ✗ | 1.99 ✗ | 2.11 | 1.75 ✗ | 2.11 |
| `v_five_from_pads` | 2.58 | 2.58 | 1.34 ✗ | 2.58 | 0.41 ✗ |
| `v_five_in_wind` | 2.62 | 2.62 | 1.30 ✗ | 2.62 | 0.31 ✗ |
| `v_after_lanes` | 2.97 | 2.97 | 2.97 | 2.97 | 0.42 ✗ |
| `line_after_lanes` | 2.41 | 2.41 | 2.12 | 2.13 | 2.13 |
| `hold_then_new_mission` | 2.59 | 2.59 | 2.12 | 2.12 | 2.13 |
| `v_leader_low_battery` | 5.14 | 3.90 | 1.56 ✗ | 3.90 | 1.56 ✗ |
| `v_middle_low_battery` | 4.33 | 4.33 | 5.14 | 4.33 | 5.14 |
| **worst** | **1.69** | **1.31** | **1.30** | **1.31** | **0.31** |

No variant won everywhere:

- Anchoring at the leader is right for a V, because every follower moves straight back
  from its pad.
- It is wrong for a line.
- Anchoring at the first waypoint is right for a line and wrong for a V.

This sweep ran on an intermediate `planning.py` that took the anchor as a parameter. It
is not in the history; the conclusions are.

**The traces showed why.** swarmsim's follower chases its slot relative to the leader's
*live* position, and the leader flies to the first waypoint at 2 m/s while the formation
forms up. In E2a's line, the leader from the middle of the row flies over the pad of the
very drone that is to take the slot behind it, and passes it at 1.76 m. Straight-line
models of the transition miss this. Adding a straight-line clearance check to B-plain did not
fix it: v_five_from_pads still came within 1.48 m, and v_five_in_wind within 1.14 m.

### E2b: form up first

Once the leader stops moving sideways while its followers form up, the formation forms
around a fixed point, and the choice becomes simple. The design has four parts:

1. **Form up above the leader.** The leader's path starts straight above where it
   stands. Then it flies the route.
2. **Plain horizontal distance**, over every plan, meaning every leader and every order.
   The search is exhaustive up to seven drones. Beyond that, the Hungarian algorithm
   gives one order per leader.
3. **Break exact ties by clearance.** Of four equally cheap lines from five pads, two
   pass two followers 2.12 m apart and two keep them 2.68 m apart, which is 1.88 m
   against 2.44 m in flight. A tie goes to the plan whose closest pairs are farthest
   apart, closest pair first (leximin).
4. **Average a disarmed drone's position reports.** It is on the ground and not moving,
   so the plan starts from the average. One noisy report per drone decides between
   plans that differ by less than the noise.

Two other ideas were measured worse and dropped:

- **Choosing the plan with the largest predicted clearance.** formation_line fell to
  2.03 m, because the straight-line model misjudges long, speed-limited moves.
- **A cost tolerance band with the clearance tie-break.** GPS noise moved plans into and
  out of the band. On seed 12 it chose a worse leader.

Reference → LabSwarm, 20 seeds, closest approach in the worst seed:

| Scenario | Closest approach | Mission time |
|---|---|---|
| `v_formation_from_pads` (swarmsim's) | 0.48 → **3.14 m** | 31.8 → 33.7 s |
| `v_five_from_pads` | 0.41 → **3.03 m** | 36.8 → 40.2 s |
| `v_five_in_wind` | 0.31 → **2.43 m** | |
| `v_after_lanes` | 0.42 → **3.00 m** | 35.9 → 37.3 s |
| `v_leader_low_battery` | 1.56 → **5.14 m** | 58.8 → **43.7 s** |
| `v_middle_low_battery` | 5.14 → 4.15 m | 41.8 → 46.4 s |
| `formation_line` (swarmsim's) | 2.12 → 2.44 m | 36.8 → 40.2 s |
| `line_three_from_pads` | 2.11 → 2.30 m | |
| `hold_then_new_mission` | 2.13 → 2.19 m | |
| `line_after_lanes` | 2.13 → 2.02 m | |
| `formation_line_in_wind` (swarmsim's) | 2.08 → 1.88 m (limit 1.5 m; median 2.12 → 2.42 m) | |

What it costs:

- **Missions take 1–5 s longer**, for the form-up climb.
- **In GPS noise, a line led from the middle of the row keeps less margin in its worst
  seeds** than the reference's line led from the end. The reference's followers all
  move in parallel. The lab records this as an `expect: fail` (E3).

## E3: the lab's scenarios and its own mutation check

`scenarios/` holds eight scenarios, each there to catch something. swarmsim's mutation
check refuses a swarm it did not write (**F1**), so `tests/test_lab_scenarios.py` is the
lab's own. It enforces two rules:

- Each mutant in `lab_swarm/mutants.py` undoes one of E2b's decisions, and must fail
  some scenario.
- Each scenario must fail a mutant, or the reference swarm (every decision undone at
  once).

The figures are the closest approach in the worst of seeds 1–3, as in CI:

| Scenario | LabSwarm | Reference | Lab mutants it fails |
|---|---|---|---|
| `v_five_from_pads` | 3.03 m | 0.41 m ✗ | `id_order_roles` |
| `v_after_lanes` | 3.00 m | 0.42 m ✗ | `id_order_roles` |
| `v_low_battery_drone_1` | 5.14 m | 1.56 m ✗ | (none) |
| `v_low_battery_drone_2` | 4.15 m | 5.14 m | `reference_handover` (1.49 m) |
| `line_five_from_pads` | 2.44 m | 2.12 m | `no_form_up` (1.76 m), `closest_tie` (1.88 m) |
| `line_beside_the_pads` | 2.44 m | 2.18 m | `no_form_up` (1.48 m), `closest_tie` (1.88 m) |
| `v_five_very_noisy_gps` | 2.91 m | 0.35 m ✗ | `latest_position` (1.13 m), `id_order_roles` |
| `line_five_noisy_gps` (`expect: fail`) | 1.81 m ✗ | 2.09 m | (none) |

`v_low_battery_drone_1` fails no single mutant: the reference fails it only with both of
its choices at once. Its name says a drone, not a role, because the scenario format can
only name drones (**F11**). drone_1 leads the reference's V and follows in LabSwarm's.

One decision did not survive E3. Re-planning the slots when a *follower* hands over
measured no better than the reference's way of doing it, where the replacement takes
the vacated slot. That held on every scenario tried, including five-drone Vs (2.48 m
against 2.29 m, both clear). So LabSwarm uses the reference's, and only a leader's
hand-over is re-planned.

**swarmsim's own suite against LabSwarm** (`tests/test_swarmsim_suite_on_lab.py`). Every
failure is the scenario's, not the swarm's:

- `formation_line` and `formation_line_in_wind` fail `formation_error` (**F3**).
- `follower_jammed_goes_home` fails because it jams LabSwarm's leader (**F11**).
- `v_formation_from_pads` passes, and the xpass fails the suite (**F4**).

A role-free measure, `lab_swarm.metrics.formation_shape_error`, puts LabSwarm's line at
2.20 m from its shape. `formation_error` says 17.2 m. On the reference swarm the two
measures agree, at 2.20 m.

## E4: two swarms compared by SwarmApi

`tools/compare_swarms.py` does three things:

1. It flies one suite against two swarms.
2. It stores both reports in a SwarmApi (`POST /api/scenario-runs`, like swarmsim's
   `--upload`).
3. It prints the API's comparison (`GET /api/scenario-runs/compare`).

The run: an API started from swarmsim 56e150b (`dotnet run`, Open, runs in memory),
reference → LabSwarm, the lab's 8 scenarios plus swarmsim's 10, seeds 1–3. It took 11 s:

| Scenario | Base | Head | API says | Closest approach, mean |
|---|---|---|---|---|
| `v_after_lanes` | failed | passed | fixed | 0.42 → 3.00 m |
| `v_five_from_pads` | failed | passed | fixed | 0.41 → 3.03 m |
| `v_five_very_noisy_gps` | failed | passed | fixed | 0.45 → 2.93 m |
| `v_low_battery_drone_1` | failed | passed | fixed | 1.56 → 5.14 m |
| `line_five_noisy_gps` | xpass | xfail | **fixed** | 2.11 → 2.00 m |
| `v_formation_from_pads` | xfail | xpass | **regressed** | 0.48 → 3.14 m |
| `formation_line` | passed | failed | **regressed** | 2.12 → 2.44 m |
| `formation_line_in_wind` | passed | failed | **regressed** | 2.12 → 2.26 m |
| `follower_jammed_goes_home` | passed | failed | **regressed** | 2.12 → 2.44 m |
| nine others | | | unchanged | |

Five of the nine changes the API reports are about the scenarios, not the swarms (**F12**).
The comparison was designed for two commits of one swarm. There, the scenario's
`expect` is right for both runs; across two swarms it is right for neither. CI's
`compare` job repeats this run on every push and writes the table to the job summary.

## Findings about swarmsim

Each finding comes with evidence from the experiments above and a change to swarmsim
that would address it. None of them is a bug in the reference swarm's flying. They are
about using swarmsim from the outside, which is what this lab tried.

| # | Finding | Evidence | Proposed change in swarmsim |
|---|---|---|---|
| F1 | `--mutants` refuses `--sut`: an external swarm gets no mutation check. | The lab wrote its own: `lab_swarm/mutants.py`, `tests/test_lab_scenarios.py`. | `--mutants MODULE:ATTR` loading a list of `Mutant`s. `run_suite` already runs them. |
| F2 | The scenario runner cannot be pip-installed. `spec.SCHEMA_PATH` is `parents[3]/contracts/...`. | `pip install ./swarm_coordination`, then `load_scenario` raises `FileNotFoundError: .../lib/python3.11/contracts/scenario/scenario.v1.schema.json`. | Ship the schema as package data (`importlib.resources`), with a test that it equals `contracts/`. |
| F3 | `formation_error` assumes the reference's roles: `drone_1` leads and the rest hold the slots in id order. | LabSwarm's line: 17.2 m by `formation_error`, 2.20 m by a role-free fit. The two agree (2.20 m) on the reference. | Make it role-free, as in `lab_swarm.metrics.formation_shape_error`, or read the roles from the reported swarm state. |
| F4 | `expect` belongs to the scenario file, not to the (scenario, swarm) pair. | LabSwarm fixes `v_formation_from_pads` and fails swarmsim's suite with an xpass. The lab's `expect: fail` does the same to the reference. | Expectations per swarm, for example `--expect FILE` mapping scenario to outcome, defaulting to pass. |
| F5 | The Action runs only the caller's scenarios. Running swarmsim's own suite means checking swarmsim out yourself, and then F4 fails it. | This lab's `tests` job checks swarmsim out to reach its scenarios. | An `include-swarmsim-scenarios` input, useful once F4 is fixed. |
| F6 | There is no L1 (SITL) path for another swarm. The SITL smoke flies swarmsim's own nodes. | Nothing in this lab has flown in SITL. | Let `mission_dispatcher_node` load its supervisor class by name, and feed it positions. LabSwarm's changes are all in the supervisor. |
| F7 | Assertions are a closed set, so a new kind means a change to swarmsim and its schema. | The lab's role-free formation check and its "stopped where the command found it" check run in pytest, where neither the Action nor the API sees them. | Add `formation_shape` and `travel_after` kinds, or a plugin point for assertions. |
| F8 | `MissionSupervisor` has no planning hook. | `FormationAwareSupervisor` re-plans by rewriting the private `_mission.plan` and overriding the private `_hand_over`. | `MissionSupervisor(planner=...)`, a callable `(mission, drones, positions) -> MissionPlan`, plus `observe_position`. |
| F9 | Assertions on absolute end states encode the reference's timing, and pass for a swarm that never flew. | `operator_land_in_place` pins x = 16 m, the reference's 2 m/s; MinimalSwarm lands 0.5 m from where "land" found it and fails. In `follower_jammed_goes_home`, a grounded swarm passes 4 of 5 assertions. | A `travel_after: {event, max_m}` kind; every scenario asserts something only a flying swarm can meet. |
| F10 | An assertion with nothing to measure passes. | `follower_comms_blip` checks `formation_error` from 40 s; the formation lands at 37.1 s. It measures `None` on every seed, for the reference too. From 31 s it would measure 2.42 m and catch the `no_frame_conversion` mutant (6.47 m), which it lets through today. | A check with no qualifying frame fails. The window becomes `from_s: 31, to_s: 37`. |
| F11 | Faults and position checks name drones, meaning the roles the reference gives them. | `follower_jammed_goes_home` jams drone_2 and drone_3 to jam "the followers"; LabSwarm's line is led by drone_2. | Role-addressed targets (`{role: leader}`), resolved from roles the swarm state reports. |
| F12 | SwarmApi's comparison across two swarms inherits F3, F4 and F11. | Five of its nine changes, reference → LabSwarm, are artefacts. A 0.48 → 3.14 m improvement is called "regressed". | With F4 fixed, compare each run against its own expectations, and flag a scenario whose expectation differs between runs. |

## The lab's own known limitations

- **Lines in GPS noise.** `line_five_noisy_gps` is an `expect: fail`. LabSwarm falls
  under 2 m on 12 of 20 seeds (worst 1.81 m); the reference does on 1 of 20 (1.99 m). In
  `formation_line_in_wind` its worst seed is 1.88 m against the reference's 2.08 m,
  though its median is better (2.42 m against 2.12 m).
- **A form-up that waits.** The fix for the above is a form-up that waits until every
  follower is in its slot. That needs `DroneController` to hold at a waypoint, which is a
  drone-side change, outside a ground-only lab.
- **Only L0.** Everything is measured in L0 (F6). L0 keeps PX4's mode and arming rules
  and its speed limits, but not its acceleration limits or its estimator.
- **Noisy reports mid-air.** Only a disarmed drone's reports are averaged. A re-plan in
  mid-air uses each drone's latest noisy report.
- **Search limit.** The search is exhaustive up to seven drones in a formation. Beyond
  that, ties between orders are not broken.
