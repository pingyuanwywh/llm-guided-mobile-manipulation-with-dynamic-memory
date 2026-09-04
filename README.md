# LLM-Guided Mobile Manipulation with Dynamic Memory

A mobile manipulator that clears drink cans from a room. An overhead drone reveals can
coordinates a few at a time; a **local** LLM decides what to do next from a task memory that
is rewritten after every attempt; a mecanum rover navigates to a standoff pose, grasps the can
with a depth camera and a 5-DoF arm, and delivers it to a collection station. Then it does it
again — replanning against whatever the drone has revealed by then.

**Best real-robot run: 5 / 5 cans, zero human intervention, 6 min 12 s.** 5/5 has been
reproduced on several later runs, including one planned live in the order the drone revealed
the cans.

> 中文版完整说明（更详细，含全部踩坑记录）见 **[README.zh.md](README.zh.md)**。

![Qualitative sequence of a five-can run](figures/qualitative_0815_drone5/qualitative_sequence_0815_drone5.png)

*One complete five-can run (2026-08-15). Each panel: ceiling camera (left), onboard RGB (top
right), depth colormap (bottom right). This layout is `free` mode — five cans (four red, one
green), two collection stations, and the LLM picks which station to use on each trip.
(a) start, gripper facing the empty bin; (b)–(d) the first three cans entering the grasp sweet
spot; (e) the green can; (f) the last can, with the coordinate-reporting drone visible at right.*

---

## What is actually interesting here

Most of the engineering in this repo is not "we called an LLM". It is the boundary drawn
around the LLM, and the fact that every constant in that boundary came from a measurement on
the real robot.

### 1. The LLM is structurally incapable of choosing a collision

```
  DETERMINISTIC LAYER              LLM LAYER                EXECUTION LAYER
  (plan_next.py, upper half)       (lower half)             (on the rover, frozen)

  synthesise standoff poses   ->   pick one candidate   ->   run_one_can.sh
  geometric clearance gate         from those that           nav → approach → grasp → deliver
  drop anything that fails         PASSED the gate
```

The LLM never emits a coordinate. It emits an **action name**, constrained to a JSON-schema
`enum` that was built *after* the clearance gate ran. A leg that would sweep too close to
another can is not a bad option the model might avoid — **it is not in the option list at all.**
Worst case, the model picks a suboptimal route; it cannot pick a crash.

Two measured reasons this had to be structural rather than prompt-based:

- **`enum` guarantees vocabulary, not semantics** (observed 2026-07-01) — schema-constrained
  decoding keeps the output parseable, not correct.
- **`temperature=0` does not guarantee determinism** (observed 2026-07-08) — so no safety
  property may rest on the model reproducing a previous answer.

Why the cans need a geometric gate at all: they sit **below the lidar's scan plane**, so they
never enter the costmap and the obstacle-avoidance stack does not know they exist. `move_base`
will happily plan a path straight through one, and the rover will knock it flying. The gate is
what supplies the obstacle knowledge the navigation stack structurally cannot have.

### 2. Dynamic memory: the plan is rebuilt from state, not from a script

`mission_state.yaml` is the single source of truth, re-read before every decision and rewritten
after every attempt:

```yaml
cans:
  can4:
    x: 2.362
    y: -1.436
    color: red
    source: drone        # who reported it
    status: collected    # pending | collected | failed | stuck
    attempts: 1          # drives the retry / give-up logic
    seen_at: '15:00:12'  # when the drone revealed it
    station: collect     # which station it went to
robot: {x: -1.169, y: -1.136, holding: null, at: collect}
active: null             # the leg in flight, if any
```

Because the state is data and not a program counter, three things fall out for free:

- **Cans can appear mid-mission.** The drone feed reveals coordinates on a timer; a can that
  shows up while the rover is already driving somewhere becomes a new candidate at the next
  decision point.
- **Failure is a first-class state.** `attempts` and `status: failed` feed back into the next
  prompt as *"this target has failed N times"*, so the model can choose to move on. The rule
  learned on the robot is **retry once unchanged, stop after two consecutive failures** — one
  early run was lost by a human stopping a loop that would have succeeded on its own retry.
- **Interruption is decidable, and bounded by physics.** A leg is split into
  `--phase nav` (drive to the standoff pose **empty-handed**) and `--phase rest` (grasp and
  deliver). Only `nav` is interruptible — there is no safe way to abandon a leg with a can
  half-gripped. While a `nav` phase is in flight, a newly revealed can triggers one narrow
  question, *should we abandon the current target?*, rather than a silent re-plan.

### 3. Prompt decomposition, found by measurement

The model is `qwen3:8b` served locally by ollama. It is small, and it fails in a specific,
reproducible way: **ask it two questions in one JSON and it conflates them.**

- Asking *"which can?"* and *"which station?"* together (2026-08-01): the model was handed both
  stations' trip costs (4.60 vs 5.50), chose the expensive one, and **fabricated** a
  justification — *"collect_b is closer"* — citing a number that was not in the candidate set.
- Asking *"should I abort?"* inside the general planning prompt: the general rule *"prefer
  smaller trip"* and the re-routing rule *"being closer is not a reason to switch"* directly
  contradict each other, and the model reliably picks the easier rule — producing exactly the
  banned behaviour.

The fix is not a better prompt, it is **three separate single-question calls**:
`should_abort()` → `ask_llm()` (which can) → `pick_station()` (which station). The abort prompt
does not contain trip costs at all, so the contradiction cannot be constructed.

A related trap, same class: when a leg is in flight, the actions `continue` and `can3` mean the
same thing. The model then picks the more familiar-looking token and the decision degenerates
into *"this is the current task, so continue."* The active can is therefore **removed** from the
selectable list whenever `continue` is offered.

### 4. Every safety constant is an experimental result

| Constant | Value | Where it came from |
|---|---|---|
| `HARD` | 0.193 m | robot radius 0.16 + can radius 0.033 — geometric certain-collision line |
| `WARN` | 0.50 m | raised from 0.30 after a leg with **0.48 m straight-line clearance knocked a can over** |
| `CAN_AHEAD` | 0.45 m | can position = standoff + 0.45 m along approach yaw; calibrated by two 5/5 runs |
| `SELF_MIN` | 0.30 m | a standoff on the far side of its own can made the approach drive **over** it (measured 0.039 m) |
| TEB path deviation | ≤ 0.166 m | measured deviation of the executed trajectory from the global plan |

The straight-line clearance model that these thresholds originally used was itself audited
against the planner: over 450 paired samples it **overestimates clearance on 56% of legs**,
P90 +0.18 m, worst case +0.35 m. Twelve legs were "safe" by straight line while the real path
entered the warning band. Hence `plan_clearance.py`, which asks `move_base/make_plan` for the
**actual** global path and measures against that. The straight-line model is still used for
offline simulation — and is explicitly **not** treated as evidence of real-robot safety.

---

## Pipeline

```
drone / drone_sim.py             [host]   reveals can x/y over time
        ↓ drone_feed.jsonl
   plan_next.py                  [host]   read memory → build candidates → clearance gate → LLM
        ↓ action name only, never a coordinate
   mission_run.py                [host]   main loop: drive the rover over ssh, write memory back
        ↓ ssh
   nav_goto.py → rotate_to.py    [rover]  rotate-then-go navigation to the standoff pose
        ↓
   approach.py                   [rover]  pure translation to bring the can into the sweet spot
        ↓
   final_track_and_grab_y0.py    [rover]  depth detection + IK + gripper, then deliver
```

---

## Platform

| | |
|---|---|
| Rover | Hiwonder JetRover, mecanum base, Jetson Nano, ROS Noetic |
| Sensing | Orbbec depth camera (on the arm), RPLidar |
| Arm | 5-DoF with parallel gripper |
| Mapping / nav | hector SLAM + `move_base` with **TEB** local planner and mecanum lateral motion |
| LLM | `qwen3:8b` via local ollama, `temperature=0`, JSON-schema constrained. **No cloud, no API key.** |
| Host | Ubuntu 20.04 — planning, rviz god-view, offline re-rendering, video production |

Switching the local planner from DWA to TEB with holonomic motion was itself measured:
**−89% yaw wobble, 5× faster legs, zero sign changes on large turns, obstacle detours 43 s → 10 s**,
and TEB recovered in 9.3 s from a pose where DWA deadlocked against an obstacle.

---

## Getting started

### Try the decision layer with no robot at all

The planning and LLM layer depends on **`PyYAML` and nothing else** — no ROS, no hardware.
It is worth five minutes even if you never touch a JetRover:

```bash
pip install PyYAML
ollama pull qwen3:8b                       # local model; there is no cloud in this project
cd code/planning

# deterministic geometric gate only
python3 plan_next.py --state ../../run_data/mission_state_0816a.yaml --no-llm
# gate + LLM
python3 plan_next.py --state ../../run_data/mission_state_0816a.yaml
```

That state file is a real recorded layout. Expected output: of five cans, **only `can3` passes
the gate** — the other four are reported blocked by `can3` with their clearances. Those four are
then absent from the `enum` the LLM is given. This is the safety property in the introduction,
executable in one command.

`mission_sim.py` goes further and replays real recorded coordinates through several contrived
situations, still with the robot powered off.

### On real hardware

⚠️ **This repository is an archive, not an installed package.** The directory layout exists so
the code can be *read*; every script calls its neighbours as `~/foo.py`. A fresh clone runs
nothing until the files are laid back out:

```bash
cp jetrover_env.example ~/.jetrover_env && chmod 600 ~/.jetrover_env   # your rover's IP
bash tools/deploy.sh            # dry run: prints exactly what it would write, touches nothing
bash tools/deploy.sh --apply    # lay files out into host ~/ and rover ~/
bash ~/jetrover_up.sh           # seven-step preflight
```

`deploy.sh` backs up anything it overwrites, skips site calibration by default (waypoints and
IMU bias must be re-measured, never copied), and **refuses to overwrite a rover file that is
newer than the repo's copy** — the rover is the source of truth for rover-side code.

`jetrover_up.sh` turns "connect → grasp stack → cmd_vel bridge → read-only health check → lidar →
navigation stack → clear stale waypoints" into seven steps, **each with a pass/fail criterion,
halting on the first failure.** It never commands the rover to move. It exists because the
alternative — discovering a broken link after driving a full five-can run — costs a battery and
an hour.

Rover-side environment must export `MACHINE_TYPE=JetRover_Mecanum` explicitly.
Dependencies are tiered in [`requirements.txt`](requirements.txt);
porting to other hardware is documented in [`docs/PORTING.md`](docs/PORTING.md);
how the demo videos are recorded, synchronised and re-rendered is in
[`docs/VIDEO_PRODUCTION.md`](docs/VIDEO_PRODUCTION.md).

---

## Repository layout

| Path | Contents |
|---|---|
| `code/planning/` | Task planning, LLM decision layer, offline simulation. Runs on the host |
| `code/grasp/` | Grasping + navigation. Mostly executes **on the rover**; this is a synced copy |
| `code/car/` | Rover-only files with no host copy: startup scripts, nav/EKF config, IMU debias, waypoints |
| `code/demo_tools/` | rviz god-view, offline re-rendering from rosbag, screen capture, video repair — method in [`docs/VIDEO_PRODUCTION.md`](docs/VIDEO_PRODUCTION.md) |
| `code/demo_scripts/` | Multi-view composition, detection overlays, paper figures |
| `code/ops/` | Startup + health-check script |
| `run_data/` | Mission states, maps, planned paths, per-run logs |
| `figures/` | Qualitative sequences and diagnostic comparisons |
| `deliverables/` | Portfolio page, teaser deck |

`README.zh.md` carries a **file-by-file index ranked by reuse value** — which files transfer to
a different robot, which are project-specific, and which are one-shot diagnostics.

### Things here that transfer to other robots

- `grasp/plan_clearance.py` — ask the planner for the real path before trusting a clearance number
- `grasp/rotate_to.py` — closed-loop in-place rotation on TF feedback; cures DWA turn-around deadlock
- `car/imu_debias.py` — online gyro bias estimation. Bias drifts with temperature (**15% in 15 min**),
  so it must be measured every session and can never be hard-coded
- `car/cmd_odom.py` — treat commanded velocity as an odometry source when there are no wheel encoders
- `demo_tools/demo_markers.py` + `pub_map.py` — overlay task state onto rviz **after** the fact from
  a rosbag, so a demo video does not require a re-shoot
- `demo_tools/fix_truncated_mp4.py` — rebuild an H.264 MP4 that lost its `moov` atom to a power cut
  (recovered 10 333 frames from a truncated recording)
- `planning/plan_next.py` — the layered safety boundary and the prompt-decomposition notes above

---

## Honest limitations

- **Localisation is the weak point, not grasping.** Across every failed run in the log, grasping
  was near-perfect and the failures were SLAM map corruption or hardware. hector SLAM
  scan-matching diverges in feature-poor spaces (an open outdoor area) or when large geometry
  moves (a lift door opening). The intended fix — build the map once, then run `map_server` +
  AMCL so localisation can never write back into the map — is designed and the packages are on
  the rover, but **it has not been run yet.**
- **The LLM has not been shown to beat greedy.** On a real layout with real clearances, its
  output matched an online greedy baseline token-for-token (4/4 reproductions, 16.74 m vs 16.08 m
  optimal), and its own `reason` field admitted it was minimising trip length. The cause is
  identified: the greedy rule is written into the system prompt. This is a known open item, not
  a result being claimed.
- **Single-station layouts have no optimisation headroom at all** — total distance is
  order-independent by construction. Two stations are required before planning quality is even
  measurable.
- **Placement is open-loop.** Grasp success is verified by gripper closure width; release is not
  verified at all. Drop-point error has a median of 10.1 cm, and one failure missed the bin by
  5 mm more than the next-worst success.

---

## License

Own code is MIT — see [`LICENSE`](LICENSE).

`code/grasp/final_track_and_grab_y0.py` is **derived from** a Hiwonder JetRover factory example.
The vendor original is **not redistributed here**. See [`NOTICE.md`](NOTICE.md) for the full
third-party attribution, including the ROS packages used and the privacy scrubbing applied to
the run logs.
