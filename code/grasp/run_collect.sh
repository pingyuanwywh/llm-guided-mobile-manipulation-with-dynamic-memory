#!/usr/bin/env bash
# run_collect.sh — 闭环收集编排 (2026-07-25)
# 倒车30cm -> 对每个罐: set_color -> nav_goto --face -> approach --then-grab
#            -> (抓上了) nav_goto collect --face -> place(投矮盒)
# 每步独立 subshell 分别 source (nav=ros1_ws / grasp=ros_car+JetRover+MACHINE_TYPE)。
# 一口气连跑, 步与步之间零暂停。日志 /tmp/run_collect.log。
set -u

NAV="source /opt/ros/noetic/setup.bash; source /home/uavg/ros1_ws/devel/setup.bash"
GRASP="export MACHINE_TYPE=JetRover_Mecanum; source /opt/ros/noetic/setup.bash; source /home/uavg/ros_car/devel/setup.bash; source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash"

echo "=== setup: place_pose for low box ==="
bash -c "$GRASP; rosparam set /track_and_grab/place_pose '[0.32, 0.0, 0.13]'"

echo "=== reverse 30cm to start position ==="
bash -c "$GRASP; python3 -u /home/uavg/approach.py --nudge -0.30 0"

run_can () {
  local NAME=$1 COLOR=$2
  echo "===================== $NAME ($COLOR) ====================="
  bash -c "$GRASP; rosservice call /track_and_grab/set_color \"data: $COLOR\"" >/dev/null 2>&1
  echo "--- nav to $NAME ---"
  bash -c "$NAV; python3 -u /home/uavg/nav_goto.py --place $NAME --face"
  if [ $? -ne 0 ]; then echo "!! $NAME nav FAILED, skip can"; return; fi
  echo "--- approach + grab $NAME ---"
  bash -c "$GRASP; python3 -u /home/uavg/approach.py --then-grab --key x --target 0.35 --search-max 0.22 --creep-max 0.15"
  local G
  G=$(bash -c "$GRASP; python3 /home/uavg/read_servos.py" 2>/dev/null | grep -oE '10=[0-9]+' | head -1 | cut -d= -f2)
  echo "--- gripper=$G (夹住罐~250 / 空~418 / 张开~63) ---"
  if [ -n "$G" ] && [ "$G" -gt 150 ] && [ "$G" -lt 360 ]; then
    echo "--- grabbed, nav to collect ---"
    bash -c "$NAV; python3 -u /home/uavg/nav_goto.py --place collect --face"
    if [ $? -ne 0 ]; then echo "!! collect nav FAILED (still holding $NAME)"; return; fi
    echo "--- place into box ---"
    bash -c "$GRASP; rosservice call /track_and_grab/place \"{}\""
  else
    echo "!! $NAME NOT grabbed (g=$G), skip place, go next"
  fi
}

run_can can1 green
run_can can2 red
run_can can3 red
run_can can4 red
echo "===================== ALL DONE ====================="
