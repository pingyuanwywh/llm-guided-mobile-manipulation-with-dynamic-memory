#!/bin/bash
# Render the 2026-08-27 daytime dynamic RVIZ/god-view to MP4.
set -euo pipefail

RUN="$HOME/jetrover_demo/raw/20260827_1631_drone5_2station-5can-SUCCESS-5of5"
EDITED="$HOME/jetrover_demo/edited/20260827_1631_drone5_2station-5can-SUCCESS-5of5/intermediate"
BAG="$RUN/0827b_drone5.bag"
RVIZ_CFG="$HOME/.rviz/jetrover_demo_markers.rviz"
MODE="${1:-full}"
DISPLAY_ID="${DISPLAY_ID:-:99}"
SCREEN="${SCREEN:-1280x810x24}"
SIZE="${SIZE:-1280x810}"
FPS="${FPS:-15}"

case "$MODE" in
preview) OUT="${OUT:-/tmp/drone5_0827b_dynamic_rviz_preview.mp4}"; DUR="${DUR:-12}" ;;
full) OUT="${OUT:-$EDITED/20260827_1631_drone5_dynamic_rviz_full.mp4}"; DUR="${DUR:-687}" ;;
*) echo "usage: $0 [preview|full]" >&2; exit 2 ;;
esac

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1 DISPLAY="$DISPLAY_ID" QT_X11_NO_MITSHM=1 LIBGL_ALWAYS_SOFTWARE=1 DISABLE_ROS1_EOL_WARNINGS=1
mkdir -p "$(dirname "$OUT")"

cleanup() {
  set +e
  pkill -f "[r]osbag play --clock" >/dev/null 2>&1
  pkill -f "[r]viz.*jetrover_demo_markers.rviz" >/dev/null 2>&1
  $HOME/replay_0827b_drone5.sh stop >/dev/null 2>&1
  if [ -n "${XVFB_PID:-}" ]; then kill "$XVFB_PID" >/dev/null 2>&1; fi
}
trap cleanup EXIT

$HOME/replay_0827b_drone5.sh stop >/dev/null 2>&1 || true
pkill -f "[X]vfb $DISPLAY_ID" >/dev/null 2>&1 || true
Xvfb "$DISPLAY_ID" -screen 0 "$SCREEN" -ac >/tmp/drone5_0827b_xvfb.log 2>&1 & XVFB_PID=$!
sleep 2
$HOME/replay_0827b_drone5.sh start >/tmp/drone5_0827b_render_replay.log 2>&1
rviz -d "$RVIZ_CFG" >/tmp/drone5_0827b_rviz.log 2>&1 &
sleep 10
ffmpeg -y -nostdin -hide_banner -loglevel warning -stats -f x11grab -draw_mouse 0 -framerate "$FPS" \
  -video_size "$SIZE" -i "$DISPLAY_ID.0" -t "$DUR" -vf "crop=1152:648:118:162,scale=960:540" \
  -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p -movflags +faststart "$OUT" & FFMPEG_PID=$!
sleep 2
rosbag play --clock --quiet "$BAG" >/tmp/drone5_0827b_rosbag_play.log 2>&1 & ROSBAG_PID=$!
wait "$FFMPEG_PID"
kill "$ROSBAG_PID" >/dev/null 2>&1 || true
wait "$ROSBAG_PID" >/dev/null 2>&1 || true
echo "$OUT"
