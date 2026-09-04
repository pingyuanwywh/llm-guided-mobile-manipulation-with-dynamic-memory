#!/usr/bin/env bash
# finish_deliver.sh -- 手上夹着罐子时, 把它送到指定站放下 (2026-08-01)
# 用途: 板子中途挂掉导致 run_one_can 半途而废, 罐子留在爪里 —— 不能重跑整条腿
#       (那会对着空地再抓一次), 只能补送。用的就是 run_one_can.sh 里那两条原样命令。
# 用法: bash ~/finish_deliver.sh collect_a
set -u
STATION="${1:?用法: finish_deliver.sh <站名>}"
NAV="source /opt/ros/noetic/setup.bash; source /home/uavg/ros1_ws/devel/setup.bash"
GRASP="export MACHINE_TYPE=JetRover_Mecanum; source /opt/ros/noetic/setup.bash; source /home/uavg/ros_car/devel/setup.bash; source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash"

echo "+ rosparam set place_pose [0.32, 0.0, 0.13]   # 矮盒落点"
bash -c "$GRASP; rosparam set /track_and_grab/place_pose '[0.32, 0.0, 0.13]'"

echo "+ nav_goto --place $STATION --face --thresh 181"
bash -c "$NAV; python3 -u /home/uavg/nav_goto.py --place $STATION --face --thresh 181"
if [ $? -ne 0 ]; then echo "!! 导航到 $STATION 失败, 罐子还在手里"; exit 4; fi

echo "+ place"
bash -c "$GRASP; timeout 90 rosservice call /track_and_grab/place '{}'"
RC=$?

echo "+ 回读夹爪(放成了应该张开 ~63)"
timeout 25 bash -c "$GRASP; python3 /home/uavg/read_servos.py" 2>/dev/null | head -2
exit $RC
