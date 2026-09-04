#!/bin/bash
# 抓取最小栈: roscore + 深度相机 + board_node(舵机). kinematics 由 grab_depth 进程内算.
# 关键: 相机 launch 必须"只 source JetRover 工作区"(含 hiwonder_peripherals + OrbbecSDK),
#       否则 source ros_car 会把它挤出包路径 -> 找不到 depth_cam.launch (2026-07-13 踩过).
mkdir -p ~/Log
source /opt/ros/noetic/setup.bash
setsid nohup roscore > ~/Log/roscore.log 2>&1 < /dev/null &
sleep 5
# 深度相机 (Orbbec DaBai DCW) —— 只 source JetRover ros_ws
( source /opt/ros/noetic/setup.bash
  source ~/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash
  export DEPTH_CAMERA_TYPE=Dabai
  setsid nohup roslaunch hiwonder_peripherals depth_cam.launch > ~/Log/depthcam.log 2>&1 < /dev/null & )
# 舵机 board_node —— source ros_car
( source /opt/ros/noetic/setup.bash
  source ~/ros_car/devel/setup.bash
  export MACHINE_TYPE=JetRover_Mecanum
  setsid nohup roslaunch ros_robot_controller board_node.launch > ~/Log/board.log 2>&1 < /dev/null & )
sleep 10
echo "stack up: roscore + depth_cam(JetRover ws) + board_node(ros_car ws). logs ~/Log/"
echo "  then: DRY_RUN=false bash ~/run_depth.sh   (深度凸起抓取)"
