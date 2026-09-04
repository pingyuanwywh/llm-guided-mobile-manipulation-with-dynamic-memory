#!/usr/bin/env bash
set -eo pipefail

ROOT="/home/uavg/JetRover-Jetson_nano_ros1"
ROS_WS="${ROOT}/ros_ws"
LOG="/tmp/final_track_and_grab.log"

usage() {
  cat <<'EOF'
Usage:
  final_grabctl.sh start [color] [low_pitch] [high_pitch]
  final_grabctl.sh grab
  final_grabctl.sh place
  final_grabctl.sh status
  final_grabctl.sh log
  final_grabctl.sh stop

Examples:
  final_grabctl.sh start green
  final_grabctl.sh start red
  final_grabctl.sh start green 10 30
  final_grabctl.sh grab
  final_grabctl.sh place
EOF
}

source_ros() {
  source /opt/ros/noetic/setup.bash
  source "${ROS_WS}/devel/setup.bash"
}

cmd="${1:-}"
case "${cmd}" in
  start)
    color="${2:-green}"
    low_pitch="${3:-10}"
    high_pitch="${4:-30}"
    "${ROOT}/start_final_grab.sh" "${color}" "${low_pitch}" "${high_pitch}"
    ;;
  grab)
    source_ros
    rosservice call /track_and_grab/grab "{}"
    ;;
  place)
    source_ros
    rosservice call /track_and_grab/place "{}"
    ;;
  status)
    source_ros
    echo "--- tmux ---"
    tmux ls 2>/dev/null || true
    echo "--- nodes ---"
    rosnode list | grep -E "board|depth_cam|kinematics|hiwonder_servo_manager|track_and_grab" || true
    echo "--- services ---"
    rosservice list | grep /track_and_grab || true
    ;;
  log)
    tail -f "${LOG}"
    ;;
  stop)
    source_ros
    rosservice call /track_and_grab/stop "{}" || true
    tmux kill-session -t final_track_grab 2>/dev/null || true
    ;;
  -h|--help|help|"")
    usage
    ;;
  *)
    echo "Unknown command: ${cmd}" >&2
    usage
    exit 2
    ;;
esac
