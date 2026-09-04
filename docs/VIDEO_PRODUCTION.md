# Multi-View Demo Video Production

How a five- or six-view JetRover demo video gets made, and — more usefully — the specific ways
it has gone wrong. Every number here was measured on real footage; where a timestamp was
inferred visually rather than read from a clock, it says so.

This merges two previously separate sources: the production notes kept alongside the runs, and
a tool-specific agent skill that lived only on one workstation. It is in the repository so it
survives both.

---

## The views

| View | Who records it | Source |
|---|---|---|
| Ceiling (lab camera) | operator | Windows PC in the lab |
| Handheld / phone | operator | phone |
| Onboard RGB | `code/grasp/record_cam.py`, auto | rover |
| Onboard depth (pseudocolour) | same process, second stream | rover |
| rviz god view | `code/demo_tools/rec_rviz.sh` | host |
| Drone | operator | drone, flown separately |

Onboard RGB and depth carry the detection overlay; the god view carries the map, the planned
path, the cans, and the geometric-gate state.

---

## 1 · Recording

### The checklist

Two full runs were lost to process, not technology — one with **no footage at all** of the best
result of the day, one where the recorder was never stopped and the H.264 files never got their
`moov` atom. Both were "I'll remember" failures. What remains manual is minimised:

1. **Operator starts ceiling + phone first** — both must be rolling *before the rover first
   moves*, because that motion is the synchronisation anchor.
2. Host: `bash ~/start_rviz_virtual.sh` → `bash ~/rec_rviz.sh start <tag>`
3. Rover: `bash ~/run_fixed5.sh --record <tag> …` — onboard RGB and depth start and stop
   **inside the script**, under `trap … EXIT INT TERM`, so a crash, a Ctrl-C or a SIGTERM all
   still close the files properly.
4. `bash ~/rec_rviz.sh stop` — **SIGINT, never SIGKILL.** `rec_rviz.sh:66` does this
   deliberately: ffmpeg only writes the `moov` atom on a clean interrupt. A killed recorder
   produces an unplayable file.
5. **Close rviz before pulling files.** With rviz attached, `/map` alone eats about half the
   available bandwidth (2 frames/s × ~1.05 MB).
6. **Before cutting power, confirm nothing is still recording:** `bash ~/rec.sh status`.

### Two rover-side settings that silently ruin footage

- **LDP makes depth entirely black.** With laser near-distance protection on,
  `/depth_cam/depth/image_raw` is all zeros. The grasp node turns it off, but the recorder
  usually starts first — so `record_cam.py` calls `/depth_cam/set_ldp False` itself
  (idempotent) and prints the first-frame valid-depth ratio.
- **Depth pseudocolour range is TURBO 0.30–0.90 m, not 0.25–2.5 m.** With the arm in its
  observation pose the entire frame lies between 0.35 m and 0.64 m (p5 = 350, p50 = 438,
  p95 = 600, max = 639 mm). The wider range spends 90 % of the colour ramp on empty bins and
  renders as flat blue-violet. Invalid points are drawn black, which makes a metal can a **black
  hole** in the depth view — an accidental but honest visualisation of why grasping has to rely
  on colour.

### Detection boxes are data, not pixels

`final_track_and_grab_y0.py` publishes `/track_and_grab/vis_info` (a `std_msgs/String` of JSON:
`ok / color / box[x,y,w,h] / c[cx,cy] / dist_mm / tracking`). `record_cam.py` subscribes and
draws onto **its own** RGB and depth streams — same camera, same resolution, so pixel
coordinates carry over. Published only when someone is subscribed, rate-limited to 15 Hz, so
recording costs nothing when off.

An earlier version published the grasp node's composed `imshow` image as a *third* video
stream. It was a re-render of two views already being recorded; only the annotation was new.
Deleting it halved the bitrate.

⚠️ Coordinate trap: `tracker.proc()` computes `boundingRect` on a **half-resolution** image
(×2 for the full frame), but the `center_x/center_y` it returns is **already doubled**.

Boxes appear only during measure/grasp phases, never during navigation. That is honest — it
shows when the robot is actually looking at a can.

---

## 2 · Synchronisation

**This is the part that decides whether the edit is truthful.**

### Use the first sustained drive as the anchor, not a slate

The obvious approach — a clapper-board action the rover performs — was tried and **measured to
fail**. A 5 cm mecanum sidestep produces a ceiling-view frame-difference peak of 2.05 against a
baseline of 0.53: buried in noise. Handheld footage is worse, because the whole frame is moving.

What works is the rover's **first sustained drive** (the moment `nav_goto` issues the first leg
goal). It is displacement, not a twitch, and every view shows a clean step:

| View | frame-diff before → after |
|---|---|
| Onboard RGB | 0.49 → 4.11 |
| Ceiling | 0.44 → 1.04 |
| Phone (handheld) | 0.47 → 0.85 |
| rviz | 0.05 → 0.08 |

Tool: `code/grasp/profile.py <frame-dir> <sample-rate> <start-sec>` prints a per-second
frame-difference curve as a bar chart. **Read the curve; do not set a threshold.** A "3× median"
auto-threshold failed on every one of these signals, including the onboard camera whose peak was
15.7 against a 4.69 baseline. Crop far views to the area around the rover first — averaging over
the whole frame dilutes the signal away.

### Wall clocks are trustworthy; use them as the first guess

All three machines (rover, Android, Windows) are NTP-synced. The rover's `wall_start_epoch`
converted on the host matched the rover's self-reported ISO time exactly, and the five streams'
filename timestamps agreed to within 0.37–0.77 s (the rounding to whole seconds). So filename
times are a legitimate starting estimate — just not the final answer.

**Cross-validate rather than measure once.** In one run the derived phone start time (21:44:57.77)
landed 0.61 s after the slate sidestep at 21:44:57.16 — which matched the operator's independent
account that they "missed the move out and only started filming on the way back". That kind of
agreement between two unrelated sources is worth more than any single measurement.

⚠️ **The variable-frame-rate camera is the lab Windows PC, not the phone.** It reports 30 fps and
delivers **28.79** (833 frames short over 690 s; ≈20 s of drift by the end if not converted).
The phone was a constant 30 fps. Convert with `-vsync cfr -r 15` before compositing.

### Drone timing must come from the drone

**Do not use the rover's mission-log write times as drone detection times.** They are a different
process on a different machine describing a different event. Drone detection timing comes from
the drone video or from an explicit detection artifact, and nothing else.

The drone MP4 has **no embedded `creation_time`**; the filename time alone is not sufficient. The
usable anchor is a visual one: identify the drone's takeoff in the ceiling view and align it to
drone t = 0.

---

## 3 · The reveal timeline

The narrative claim of the whole demo is *"the drone finds a can, then the rover goes and gets
it."* The reveal timeline is what makes that claim checkable, so it is stored as a YAML artifact
with evidence, `bbox_xywh`, and an explicit confidence field — **uncertainty is recorded, never
hidden.**

Rules:

- The first can is `initial_task_input` — known before the drone flies, visible from rviz t = 0.
- Every later can reveals at its **first clear visual detection in the drone video**.
- Each entry carries the evidence that justifies it.

⚠️ **Do not hard-code which red can is which.** In a low-altitude narrow-field drone scan the
four red cans are indistinguishable, the frame shows only floor, tape lines and foam blocks, and
the drone provides no pose data. The green can is uniquely identifiable; the red ones are not.
Where red cans were assigned identities, it was done by **position in frame** by a human, and
marked `confidence: medium`. The honest fallback, if that is not defensible for a given run, is
to label the green can `can2 DETECTED` and the red ones simply `CAN DETECTED`.

⚠️ **Check that the drone footage is from the run you think it is.** One drone clip was 25
minutes off from the run it was being edited into — it was from an earlier failed attempt, after
which the layout had been changed. Compare timestamps against the rover's start, and compare the
layout visible in frame.

---

## 4 · Offline re-rendering of the god view

Cans are below the lidar plane, so they are not in the costmap and the raw rviz view shows an
**empty floor** — the rover appears to swerve for no reason, and the geometric gate's decisions
exist only in a text log. This was the biggest defect of the early recordings.

The fix does not require re-shooting: **record a rosbag, then replay it offline and overlay
markers.** Cost of the bag is ~127 KB/s.

- `code/demo_tools/demo_markers.py` — draws cans (true colour; grey once collected), both
  stations, the WARN = 0.50 status ring (red = blocked by the gate with `BLOCKED x.xxm by canN`,
  green = reachable, yellow = current target), and the rover from `/tf`. **Every value comes from
  the real logs**, not from a re-simulation.
- `code/demo_tools/demo_markers_dynamic.py` — the same, plus time-gated reveals driven by the
  timeline YAML; without one it falls back to the mission log.
- `code/demo_tools/pub_map.py` — publishes a saved pgm+yaml as a latched `/map`, because the host
  has no `map_server` and the bag deliberately excludes `/map`. ⚠️ The `image:` field in the yaml
  is an absolute path *on the rover*; the script instead looks for the pgm next to the yaml.
- `code/demo_tools/replay_<run>.sh` / `render_<run>_rviz_dynamic.sh` — bring up roscore,
  `use_sim_time`, the map and the marker node, then render.

Run with `/usr/bin/python3`, not the Anaconda interpreter — the latter has no `rospy`.

### ⛔ If you test-played the bag, restart the replay environment before rendering

Test-playing a segment (`rosbag play -s 100 -u 30`) moves the clock to 100 s. Starting the real
render from the top then moves it **backwards**, and under `use_sim_time` both `demo_markers.py`
and `pub_map.py` raise `ROSTimeMovedBackwardsException` inside `rate.sleep()` and die. The
recording keeps going and produces a file with laser points and the rover but **no map and no
markers**. Seven minutes of render, wasted.

The render scripts already guard this — `render_gateB_rviz_dynamic.sh:52,59` calls
`replay_*.sh stop` then `start` — so use them rather than driving `rosbag play` by hand.

Related: **`rosbag play` finishing does not stop the screen recorder.** One session recorded 46
extra minutes of a frozen frame. Either watch it or set a timeout from the bag duration.

### Three things that will look wrong

1. **Negative `Scale` mirrors text.** The original view config pairs `Scale:-145` with
   `Angle:3.22`; two negatives cancel and the geometry is correct, which is why nobody noticed —
   until `TEXT_VIEW_FACING` markers were added and every label rendered backwards. Use
   `Scale:+145` with `Angle:0`: same geometry, readable text.
2. **A real can is Ø66 mm ≈ 10 px** at 145 px/m, invisible and covered by its own label. Draw a
   0.30 m colour disc underneath as a marker (annotated as *not* true scale) and move the label
   0.30 m above.
3. **"blocked by canN" contradicts "canN is already grey."** The gate's verdict is computed at
   planning time and would otherwise persist until the next planning cycle. Clear a block the
   moment the blocker enters the gripper. This is what makes the best single second of the video
   possible: can5 is lifted, and can1's ring turns from red to green in the same instant.

⚠️ **Only runs with a bag can be re-rendered.** Two early scenes have no bag; their god view can
only be a synthetic animation with interpolated trajectories and no laser points. Record a bag
every run.

---

## 5 · rviz composition

**`|Scale|` is pixels per metre. Directly. No coefficient.** Verified twice: `Scale=175`
measured 175.6 px/m (two foam blocks 1.435 m apart spanning 252 px); `Scale=110` measured 109.

```
X, Y      = centre of the site
|Scale|   = min( viewport_h(718) / (y-span + 1.2) ,
                 viewport_w(1246) / (x-span + 1.2) )
```

Cross-checked by reconstruction: a site spanning y = 3.63…−1.2 gives centre (−1.88, 1.11) and
718/4.83 = 148, against `X:-1.88 Y:1.11 Scale:-145` actually stored in the config. **Every new
map needs X/Y/Scale recomputed** — reusing an old `Scale=145` on a site spanning y = 4.51 pushes
a can at y = −1.69 outside the bottom edge (≈ −1.37).

- **Edit by line number**, not by pattern: `grep -nE "^ +(Scale|X|Y|Angle):"` then
  `sed -i '<line>s/.*/.../'`. A content regex for `X:`/`Y:` also hits `GlobalPlan.Offset`, which
  silently translates the whole planned path.
- Restart rviz for config changes to take effect. `cp` a `.bak_<date>` first.
- To make the 3D view fill the window, set `Panels: []`. `Hide Left Dock: true` alone does
  nothing — it is overridden by the base64 `QMainWindow State` blob in `Window Geometry`, which
  must be deleted. A window height of 810 yields exactly 1280×720 of render area.

⛔ **Confirm what you are looking at before calibrating against it.** The axes visible in the
view are `WorldAxes` (the map origin), not the rover — TF display is disabled in that config and
the rover is drawn only as a cyan Footprint circle. Mistaking one for the other produced a
px/m coefficient that was wrong by 1.5×.

### The virtual screen

`start_rviz_virtual.sh` runs rviz on Xvfb `:99`, so it cannot be occluded — x11grab captures
whatever is on that screen region, and the first attempt caught a terminal window sitting on
top of rviz.

- The virtual screen must be **larger than the rviz window**: 1440×960 for a 1280×810 window at
  `+100+100`, or the capture region falls off the screen.
- `DISABLE_ROS1_EOL_WARNINGS=1` suppresses the ROS 1 end-of-life dialog, which otherwise sits in
  the middle of the view and cannot be dismissed (no xdotool/wmctrl on this host). Both launch
  scripts set it.
- 🚨 **`start_rviz_virtual.sh` reporting "didn't start" is a false negative** — it checks too
  early. Trusting it once cost an entire god-view recording; rviz had been alive the whole time
  at 31 fps with the correct composition. **Verify independently:** `pgrep -af "[r]viz -d"`, or
  grab a frame:
  ```bash
  DISPLAY=:99 ffmpeg -v error -f x11grab -video_size 1440x960 -i :99 -frames:v 1 -y peek.png
  ```
- If rviz genuinely will not start, **check for a master first.** Without roscore it blocks
  waiting, creating only a 10×10 window with no main window and an empty log — which looks like
  an Xvfb or GL problem and is not.

---

## 6 · Composition

Layout is **two on top, three below**, and the split carries meaning: the top row is the world as
people see it (ceiling, handheld), the bottom row is the world as the robot sees it (onboard RGB,
depth, god view — at native resolution, never upscaled). An "Olympic rings" 3-over-2 cannot be
made equal-sized without letterboxing on a rectangular canvas.

Six-view adds the drone. Three revisions worth carrying forward:

1. **Trim the opening.** In one run the rover first moved at t = 12.8 s while the drone did not
   appear until t = 34.4 s — so the opening showed planning before any detection had happened,
   the one stretch of the video that contradicted its own narrative. Cutting 11.5 s made every
   remaining can "seen first, fetched second" (by 28 / 50 / 130 / 201 s respectively).
2. **Pad short views by cloning end frames**, not with black:
   `tpad=start_mode=clone:…:stop_mode=clone:…`. A 118.3 s drone clip in a 426.5 s edit had left
   that panel black for 274 s — two thirds of the runtime.
3. **Do not spend a panel on a duplicate.** A raw rviz path view alongside the dynamic rviz view
   is the same information twice; onboard depth is a genuinely different modality.

⚠️ Known cost of (3): the depth panel is black for 47.7 % of the runtime, in five stretches of
38–42 s that coincide exactly with carrying a can to a station (can in gripper at < 0.3 m, plus
metal reflection). Acceptable at full length; in a 75 s highlight cut each stretch compresses to
~7 s and reads as a fault.

⚠️ A note claiming "depth is basically black for this run" was **overstated** and later
disproved — per-segment brightness measured 149/0/128/87/0/132/140/0, identical to a run whose
depth track had already shipped in a five-view edit.

---

## 7 · Recovering a truncated recording

A power cut mid-run leaves an H.264 MP4 with no `moov` atom — `ffprobe` reports
`moov atom not found` and nothing will open it. The NAL units inside `mdat` are intact; only the
parameter sets and the index are missing.

`code/demo_tools/fix_truncated_mp4.py` takes SPS/PPS from a **known-good file recorded by the
same `record_cam.py` on the same rover**, converts `mdat` from AVCC to Annex-B inserting SPS/PPS
before each IDR, and remuxes at a fixed frame rate:

```bash
python3 fix_truncated_mp4.py broken.mp4 reference_good.mp4 out.mp4 15
```

Measured recovery: **10 333 frames / 688.9 s** of RGB and 9 907 frames / 660.5 s of depth. Only
the final NAL — the one being written when power was lost — fails to decode; drop it.

⚠️ The reference file must be one that **closed cleanly**. Using another power-cut file as the
reference gives two broken files. ⚠️ Do not hand-parse the `avc1`/`avcC` offsets: the
`VisualSampleEntry` 78 bytes *include* an 8-byte header, and an off-by-eight there is easy. Let
`ffmpeg -bsf:v h264_mp4toannexb -f h264` emit the stream and take NAL 7/8 from that.

---

## 8 · Validation before calling it done

- Syntax-check anything edited: `python3 -m py_compile`, `bash -n`.
- For marker timing: start the replay and sample `/demo/markers` around **each** reveal time,
  confirming the expected cans are present and the not-yet-revealed ones are absent.
- For detection boxes: extract frames at each detection timestamp and **look at them**.
- Stop ROS processes afterwards: `bash ~/replay_<run>.sh stop`.
- **There is no `ffprobe` on the rover** — pull a file to the host to verify it plays.
- If a timestamp was inferred visually, say so in the YAML or a comment. Do not launder an
  estimate into a fact.

### A trap when choosing the anchor

Do not use the first leg's approach as the alignment anchor. In one run the rover was already
parked on that can's standoff (`nav dist = 0.01`, it never moved) and the approach shifted it by
8 cm — undetectable from a far camera, the same failure as the 5 cm slate. **Where waypoint
recording leaves the rover determines whether the first leg gives you a usable anchor at all.**
