#!/bin/bash
# 离线重渲染上帝视角 (2026-08-08): 用 rosbag 里的真实轨迹 + demo_markers 叠罐子/站/几何门状态。
# 车不用开机。不覆盖任何原始素材。
set -u
BAG="${BAG:-$HOME/jetrover_demo/raw/20260808_2115_scene3_2station-5can-closedloop/scene3.bag}"
STATE="${STATE:-$HOME/mission_state_0808c.yaml}"
LOG="${LOG:-/tmp/replay_scene3.log}"   # mission_run 的运行日志; 没有就用空文件
MAP="${MAP:-$HOME/map_0808c.yaml}"
source /opt/ros/noetic/setup.bash
export ROS_MASTER_URI=http://localhost:11311
export ROS_IP=127.0.0.1

case "${1:-start}" in
stop)
  pkill -f "[r]osbag play" ; pkill -f "[d]emo_markers.py" ; pkill -f "[p]ub_map.py" 
  sleep 1; pkill -f "[r]oscore" ; pkill -f "[r]osmaster"
  echo "回放环境已停"; exit 0;;
esac

pgrep -f "[r]osmaster" >/dev/null || { setsid nohup roscore >/tmp/replay_roscore.log 2>&1 </dev/null & sleep 4; }
rosparam set /use_sim_time true
setsid nohup /usr/bin/python3 $HOME/pub_map.py "$MAP" >/tmp/replay_map.log 2>&1 </dev/null &
sleep 2
setsid nohup /usr/bin/python3 $HOME/demo_markers.py --state "$STATE" --log "$LOG" \
  >/tmp/replay_markers.log 2>&1 </dev/null &
sleep 2
echo "--- 话题 ---"; rostopic list | grep -E "demo/markers|^/map$"
echo "--- marker 节点日志 ---"; tail -4 /tmp/replay_markers.log
