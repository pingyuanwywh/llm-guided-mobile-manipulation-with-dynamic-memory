#!/bin/bash
# Render the 2026-08-15 gateB dynamic RVIZ/godview to mp4.
set -euo pipefail

RUN="$HOME/jetrover_demo/raw/20260815_1858_gateB_2station-5can-SUCCESS-5of5"
EDITED="$HOME/jetrover_demo/edited/20260815_1858_gateB_2station-5can-SUCCESS-5of5/intermediate"
BAG="$RUN/0815_gateB.bag"
RVIZ_CFG="$HOME/.rviz/jetrover_demo_markers.rviz"
MODE="${1:-full}"

DISPLAY_ID="${DISPLAY_ID:-:99}"
SCREEN="${SCREEN:-1280x810x24}"
SIZE="${SIZE:-1280x810}"
FPS="${FPS:-15}"

case "$MODE" in
preview)
  OUT="${OUT:-/tmp/gateB_dynamic_rviz_preview.mp4}"
  DUR="${DUR:-35}"
  ;;
full)
  OUT="${OUT:-$EDITED/20260815_1858_gateB_dynamic_rviz_full.mp4}"
  DUR="${DUR:-432}"
  ;;
*)
  echo "usage: $0 [preview|full]" >&2
  exit 2
  ;;
esac

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1
export DISPLAY="$DISPLAY_ID"
export QT_X11_NO_MITSHM=1
export LIBGL_ALWAYS_SOFTWARE=1
export DISABLE_ROS1_EOL_WARNINGS=1

mkdir -p "$(dirname "$OUT")"

cleanup() {
  set +e
  pkill -f "[r]osbag play --clock" >/dev/null 2>&1
  pkill -f "[r]viz.*jetrover_demo_markers.rviz" >/dev/null 2>&1
  $HOME/replay_gateB.sh stop >/dev/null 2>&1
  if [ -n "${XVFB_PID:-}" ]; then
    kill "$XVFB_PID" >/dev/null 2>&1
  fi
}
trap cleanup EXIT

$HOME/replay_gateB.sh stop >/dev/null 2>&1 || true
pkill -f "[X]vfb $DISPLAY_ID" >/dev/null 2>&1 || true

Xvfb "$DISPLAY_ID" -screen 0 "$SCREEN" -ac >/tmp/gateB_xvfb.log 2>&1 &
XVFB_PID=$!
sleep 2

$HOME/replay_gateB.sh start >/tmp/gateB_render_replay.log 2>&1

rviz -d "$RVIZ_CFG" >/tmp/gateB_rviz.log 2>&1 &
RVIZ_PID=$!
sleep 10

ffmpeg -y -nostdin -hide_banner -loglevel warning -stats \
  -f x11grab -draw_mouse 0 -framerate "$FPS" -video_size "$SIZE" -i "$DISPLAY_ID.0" \
  -t "$DUR" -vf "crop=1152:648:118:162,scale=960:540" \
  -c:v libx264 -preset veryfast -crf 20 -pix_fmt yuv420p -movflags +faststart "$OUT" &
FFMPEG_PID=$!

sleep 2
rosbag play --clock --quiet "$BAG" >/tmp/gateB_rosbag_play.log 2>&1 &
ROSBAG_PID=$!

wait "$FFMPEG_PID"
kill "$ROSBAG_PID" >/dev/null 2>&1 || true
wait "$ROSBAG_PID" >/dev/null 2>&1 || true

echo "$OUT"
