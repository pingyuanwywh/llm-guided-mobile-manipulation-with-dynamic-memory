# Porting Guide

**The porting boundary is one shell script.** Everything above `run_one_can.sh` is
robot-agnostic Python that needs nothing but `PyYAML`. Everything below it is JetRover-specific
and assumes Hiwonder's ROS packages. Knowing exactly where that line falls is most of what this
document is for.

```
  ┌─────────────────────────────────────────────┐
  │  code/planning/          host, PyYAML only  │   ← portable, no ROS, no robot
  │  memory · clearance gate · LLM · simulation │
  └──────────────────┬──────────────────────────┘
                     │   ssh + argv + exit code      ← THE BOUNDARY
  ┌──────────────────┴──────────────────────────┐
  │  run_one_can.sh <can> <color> <station>     │   ← reimplement this for a new robot
  ├─────────────────────────────────────────────┤
  │  code/grasp/ + code/car/    JetRover-only   │   ← rewrite for a new robot
  │  hiwonder_sdk · kinematics · Orbbec · TEB   │
  └─────────────────────────────────────────────┘
```

---

## Before anything: this repo is an archive, not an installed package

The directory layout (`code/grasp/`, `code/planning/`, …) exists **so the code can be read**.
It is not the layout the code runs from. Every script calls its neighbours as `~/foo.py`, and
the rover-side scripts source ROS workspaces at absolute paths under `/home/uavg/`.

**A fresh `git clone` runs nothing.** Lay the files out first:

```bash
bash tools/deploy.sh              # dry run — prints exactly what it would write
bash tools/deploy.sh --apply      # host ~/ and rover ~/
```

`deploy.sh` refuses to overwrite a rover file that is **newer** than the repo's copy, and prints
the command to pull the rover's version back instead. That guard exists because it already
happened: the repo's `standoff.sh` was four weeks behind the rover's, which had been generalised
to N cans. The rover is the source of truth for rover-side code; the repo lags by default.

---

## Case 1 — New host machine, same rover

**Cost: an afternoon, nearly all of it installing things.**

The good news, and it is not obvious from the code layout: **the planning and LLM layer needs
only `PyYAML`.** No ROS on the host. `mission_run.py` drives the rover entirely over ssh.

```bash
pip install PyYAML
curl -fsSL https://ollama.com/install.sh | sh && ollama pull qwen3:8b
ssh-copy-id uavg@<rover-ip>
cp jetrover_env.example ~/.jetrover_env    # then edit in your IPs
bash tools/deploy.sh --host --apply
```

Verify without touching the rover — this runs the real deterministic gate and the real LLM:

```bash
cd code/planning
python3 plan_next.py --state ../../run_data/mission_state_0816a.yaml --no-llm   # gate only
python3 plan_next.py --state ../../run_data/mission_state_0816a.yaml           # + LLM
```

Expected: only `can3` passes the gate; the other four are reported as blocked by `can3`.
If you get that, the whole planning layer is working.

Only the demo/visualisation pipeline needs more: `numpy Pillow matplotlib` via pip, plus ROS
Noetic, `ffmpeg` and `xvfb` via apt. See `requirements.txt` for the three tiers.

**Note:** `plan_next.py` appends a decision ledger to `~/mission_log.jsonl` on every run.

---

## Case 2 — A different JetRover of the same model

**Cost: one to two sessions. The code mostly works; the calibration does not transfer.**

The three vendor ROS workspaces (`ros_car`, `ros1_ws`, `JetRover-Jetson_nano_ros1/ros_ws`) ship
with the robot image and are deliberately **not** in this repo.

Everything below is a measurement of *one specific robot in one specific room*. Copying these
values to a second robot will not work, and several will fail in ways that look like software
bugs:

| What | Where | Cost to redo | Why it cannot be copied |
|---|---|---|---|
| Waypoints | `code/car/llm_nav_places.yaml` | ~15 min | Coordinates in a map frame that no longer exists |
| The map | rebuild with hector | ~2 min | Same |
| LAB colour thresholds | `lab_config_can.yaml` | varies | Lighting-dependent. Outdoors at night the cans had **no chroma at all** — no threshold value fixes that, only a light does |
| Gyro bias | `code/car/imu_bias.yaml` | per session | Drifts with temperature (**15% in 15 min**). Designed to be re-measured every session; a stale value is worse than none |
| Lidar → rotation-centre offset | `nav_hector_bfc.launch:21` (`0.10 0 0`) | measure once | `base_footprint` is defined **on the lidar**, 0.10 m ahead of the true rotation centre. Getting this wrong looks exactly like "the robot drifts 0.2 m every turn" |
| Grasp sweet spot | `CAN_AHEAD=0.45`, `approach.py --target 0.35` | recalibrate | Tied to this arm's reach and this gripper |
| Gripper closure judgement | accept `150 < counts < 360` | recalibrate | Reference readings on this gripper: can gripped ≈ 250, closed on nothing ≈ 418, fully open ≈ 63. Tied to this gripper and this can diameter (Ø66 mm) |

`deploy.sh` **skips all of these by default** — that is the correct default, not an oversight.
`--calib` forces them, and will tell you it is probably a mistake.

---

## Case 3 — A different robot entirely

**`code/grasp/` and `code/car/` are a rewrite.** They import `hiwonder_sdk`,
`hiwonder_kinematics`, `hiwonder_interfaces`, `ros_robot_controller`. None of that exists
elsewhere.

**`code/planning/` needs no changes at all** — provided the new robot answers the same contract.

### The contract

```
run_one_can.sh <can_name> <color> <station_name> [--phase nav|rest]
```

| Exit code | Meaning | What the planner does with it |
|---|---|---|
| `0` | Can collected and delivered | `status: collected`; move on |
| `2` | Could not navigate there | `status: failed`; the can stays a candidate, `attempts` increments |
| `3` | Gripper closed on nothing | `status: failed`; retried with the approach direction rotated 60° |
| `4` | **Holding a can and cannot return** | `status: stuck` — halts the mission and asks for a human. The one state the system will not try to recover from |
| `5` | Release/placement service failed | `status: failed` |
| `10` | `--phase nav` finished, awaiting `--phase rest` | Interruption point |

The `nav` / `rest` split is not an optimisation. `nav` drives to the standoff pose
**empty-handed** and is the only phase that can be safely abandoned when the drone reveals a
better target. Once a can is in the gripper there is no safe abort, so `rest` is atomic.

### Two more things the new robot must supply

1. **A waypoint store.** Something equivalent to `nav_goto.py --place <name>`, resolving a name
   to a pose. The planner only ever passes names.
2. **A real-path clearance query.** `plan_clearance.py` asks `move_base/make_plan` for the
   *actual* global path and measures the closest approach to every other can.

   Do not substitute a straight line. Audited over 450 paired samples, the straight-line model
   **overestimates clearance on 56% of legs**, P90 +0.18 m, worst case +0.35 m — twelve legs
   were "safe" by straight line while the real path entered the warning band. If your planner
   cannot return a candidate path, raise the warning threshold from 0.50 m to 0.82 m to
   compensate, and expect the system to become noticeably more conservative.

### Why the clearance gate is not optional

The cans sit **below the lidar's scan plane**. They are not in the costmap, and `move_base`
will happily route straight through one. The gate is the *only* thing that knows they exist.
Any robot whose obstacle sensing does see the targets can simplify this — but then it must
still enforce the boundary, because the safety property is *"a colliding option is never in the
LLM's `enum`"*, not *"the LLM avoids collisions"*.

### What you get for free once the contract is met

Task memory with failure counting and mid-mission target discovery; the geometric gate; the
three-call prompt decomposition; two-station assignment; and `mission_sim.py`, which replays
real recorded coordinates offline so you can test decision logic **without powering the robot
on at all**.

---

## Known-bad shortcuts

Recorded because each one cost a run:

- **Do not trust `--dry-run` to leave state alone.** `mission_run.py --dry-run` writes mission
  state. Reset everything to `pending` before a real run.
- **Give a fixed order if you must, but never pin the station with it.**
  `--order can4,can5` is safe: the gate still runs and the mission halts if the next can fails
  it. `--order can4:collect_b` is **not**. Clearance is measured over *both* the outbound leg
  and the return leg to the station, so a clearance number is only valid for the station it was
  computed against. The code gates the can against the *system's* station choice and then
  substitutes yours — leaving the return leg ungated. Measured: the gate approved
  `can2 → collect_a` at 5.36 m; forcing `can2:collect_b` produced a real clearance of
  **0.207 m** against a 0.193 m certain-collision line, and the dry run still reported 5/5.
  Let the system assign stations; that is the half of the problem it is there to optimise.
- **Do not take an offline dry run as evidence that a route is safe.** With no connection to
  `move_base`, `REAL_CLEARANCE` is `None` and the gate **silently** falls back to the
  straight-line model, printing a single `!!` line. Offline results tell you the logic is sound,
  never that a route is clear.
- **Do not treat `temperature=0` as reproducible**, and do not treat a JSON-schema `enum` as a
  semantic guarantee. Both were tested; both fail. This is why the gate runs *before* the enum
  is built, not after.
- **Do not re-rotate in place to recover a failed navigation goal.** Repeated in-place rotation
  is the trigger for hector map divergence. `rotate_to.py` exists to make rotation closed-loop
  and bounded.
