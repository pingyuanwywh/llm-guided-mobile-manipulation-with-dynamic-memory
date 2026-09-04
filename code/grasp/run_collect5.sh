#!/usr/bin/env bash
# run_collect5.sh — 五罐闭环收集 (2026-07-29, 从 run_collect.sh 改)
#
# 相对 07-25 那版 run_collect.sh 的三处改动, 每一处都有理由:
#  ① 罐子 4 个 -> 5 个, 且**顺序改成 can1 can2 can4 can3 can5**。
#     原因: 按 standoff 位姿反推罐子位置后算过, collect<->can3 这条腿会从 can4 旁 **0.12m** 掠过
#     (车体半径 0.16), 而罐子太矮不在雷达平面 => costmap 里根本没有它, 车会直接撞飞。
#     先收 can4, 那条路就空了。其余各腿最近 0.42m, 安全。
#  ② 删掉开头那句 `approach.py --nudge -0.30 0`(盲倒 30cm)。
#     那是 07-25 为当时的起始位置写的, 现在 TEB 从任何位置都能规划, 少一次盲动。
#  ③ nav_goto 加 `--thresh 181` = **关掉 pre-rotate**。
#     rotate_to 那层外壳是当初为"DWA 不会掉头"写的补丁; 07-28 实测 TEB 面对正后方目标
#     SUCCEEDED 15.3s、angular.z 变号 0 次, 原生就会转。留着只是每条腿白多一次原地转,
#     而原地转是涂花地图的头号元凶(同日实测: 反复原地转会让位移误差 0.099->0.455->1.645m 雪崩)。
#     ⚠️ 万一某条腿走不动, 去掉这个 flag 就回到 07-25 的行为。
#
# 保持不变(都是验证过的): --face 到点转正对罐 / 夹爪回读判成败(150<g<360) / 抓不上跳过下一个 /
#   place_pose 矮盒 [0.32,0,0.13] / 每步独立 subshell 分别 source。
set -u

NAV="source /opt/ros/noetic/setup.bash; source /home/uavg/ros1_ws/devel/setup.bash"
GRASP="export MACHINE_TYPE=JetRover_Mecanum; source /opt/ros/noetic/setup.bash; source /home/uavg/ros_car/devel/setup.bash; source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash"

echo "=== setup: place_pose for low box ==="
bash -c "$GRASP; rosparam set /track_and_grab/place_pose '[0.32, 0.0, 0.13]'"

run_can () {
  local NAME=$1 COLOR=$2
  echo "===================== $NAME ($COLOR)  $(date +%H:%M:%S) ====================="
  bash -c "$GRASP; rosservice call /track_and_grab/set_color \"data: $COLOR\"" >/dev/null 2>&1
  echo "--- nav to $NAME ---"
  bash -c "$NAV; python3 -u /home/uavg/nav_goto.py --place $NAME --face --thresh 181"
  if [ $? -ne 0 ]; then echo "!! $NAME nav FAILED, skip can"; return; fi
  echo "--- approach + grab $NAME ---"
  bash -c "$GRASP; python3 -u /home/uavg/approach.py --then-grab --key x --target 0.35 --search-max 0.22 --creep-max 0.15"
  local G
  G=$(bash -c "$GRASP; python3 /home/uavg/read_servos.py" 2>/dev/null | grep -oE '10=[0-9]+' | head -1 | cut -d= -f2)
  echo "--- gripper=$G (夹住罐~250 / 空~418 / 张开~63) ---"
  if [ -n "$G" ] && [ "$G" -gt 150 ] && [ "$G" -lt 360 ]; then
    echo "--- grabbed, nav to collect ---"
    bash -c "$NAV; python3 -u /home/uavg/nav_goto.py --place collect --face --thresh 181"
    if [ $? -ne 0 ]; then echo "!! collect nav FAILED (still holding $NAME)"; return; fi
    echo "--- place into box ---"
    bash -c "$GRASP; rosservice call /track_and_grab/place \"{}\""
  else
    echo "!! $NAME NOT grabbed (g=$G), skip place, go next"
  fi
}

run_can can1 red
run_can can3 green
run_can can5 red
run_can can2 red
run_can can4 red
echo "===================== ALL DONE  $(date +%H:%M:%S) ====================="
