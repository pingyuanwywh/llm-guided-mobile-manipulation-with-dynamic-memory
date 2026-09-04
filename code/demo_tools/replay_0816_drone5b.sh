#!/bin/bash
# Offline replay for 2026-08-16 drone5b with dynamic demo markers.
set -u

RUN="$HOME/jetrover_demo/raw/20260816_1207_drone5b_2station-5can-SUCCESS-5of5"
BAG="${BAG:-$RUN/0816_drone5b.bag}"
STATE="${STATE:-$RUN/mission_state_0816c.yaml}"
LOG="${LOG:-$RUN/m_g.log}"
MAP="${MAP:-$RUN/map_0816_drone5b.yaml}"
TIMELINE="${TIMELINE:-$RUN/drone_reveal_timeline_0816_drone5b.yaml}"

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
  echo "0816 drone5b replay stopped"
  exit 0
  ;;
play)
  rosbag play --clock "$BAG"
  exit $?
  ;;
esac

rosnode list >/dev/null 2>&1 || {
  setsid nohup roscore >/tmp/drone5b_0816_replay_roscore.log 2>&1 </dev/null &
  sleep 4
}
rosparam set /use_sim_time true
setsid nohup /usr/bin/python3 $HOME/pub_map.py "$MAP" >/tmp/drone5b_0816_replay_map.log 2>&1 </dev/null &
sleep 2
MARKER_ARGS=(--state "$STATE" --log "$LOG")
if [ -f "$TIMELINE" ]; then
  MARKER_ARGS+=(--reveal-timeline "$TIMELINE")
fi
setsid nohup /usr/bin/python3 $HOME/demo_markers_dynamic.py "${MARKER_ARGS[@]}" \
  >/tmp/drone5b_0816_replay_markers.log 2>&1 </dev/null &
sleep 2
echo "--- topics ---"
rostopic list | grep -E "demo/markers|^/map$" || true
echo "--- dynamic marker log ---"
tail -12 /tmp/drone5b_0816_replay_markers.log
