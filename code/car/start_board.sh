#!/bin/bash
# 启动主控板节点 (board_node): 中转 /board/set_motor -> ttyACM0 电机/舵机。
source /opt/ros/noetic/setup.bash
source ~/ros_car/devel/setup.bash
exec roslaunch ros_robot_controller board_node.launch
