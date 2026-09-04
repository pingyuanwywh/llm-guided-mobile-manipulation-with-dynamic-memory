#!/bin/bash
# Offline replay for the 2026-08-27 nighttime drone5 run with dynamic markers.
set -u

RUN="$HOME/jetrover_demo/raw/20260827_2030_drone5_2station-5can-SUCCESS-5of5"
BAG="${BAG:-$RUN/0827d_drone5.bag}"
STATE="${STATE:-$RUN/mission_state_0827d.yaml}"
LOG="${LOG:-$RUN/mission_0827d.log}"
MAP="${MAP:-$RUN/map_0827d.yaml}"
TIMELINE="${TIMELINE:-$RUN/drone_reveal_timeline_0827d_drone5.yaml}"

source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

case "${1:-start}" in
stop)
  pkill -f "[r]osbag play"
  pkill -f "[d]emo_markers_dynamic.py"
  pkill -f "[p]ub_map.py"
  sleep 1
  pkill -f "[r]oscore"
  pkill -f "[r]osmaster"
  echo "0827 nighttime drone5 replay stopped"
  exit 0
  ;;
play)
  rosbag play --clock "$BAG"
  exit $?
  ;;
esac

rosnode list >/dev/null 2>&1 || {
  setsid nohup roscore >/tmp/drone5_0827d_replay_roscore.log 2>&1 </dev/null &
  sleep 4
}
rosparam set /use_sim_time true
setsid nohup /usr/bin/python3 $HOME/pub_map.py "$MAP" >/tmp/drone5_0827d_replay_map.log 2>&1 </dev/null &
sleep 2
MARKER_ARGS=(--state "$STATE" --log "$LOG" --reveal-timeline "$TIMELINE")
setsid nohup /usr/bin/python3 $HOME/demo_markers_dynamic.py "${MARKER_ARGS[@]}" \
  >/tmp/drone5_0827d_replay_markers.log 2>&1 </dev/null &
sleep 2
rostopic list | grep -E "demo/markers|^/map$" || true
tail -12 /tmp/drone5_0827d_replay_markers.log
