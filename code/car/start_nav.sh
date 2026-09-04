#!/bin/bash
# 启动无里程计导航栈(hector + 静态TF + move_base)。
# 只 source /opt/ros 和 ros1_ws —— 不要 source ros_car, 否则会把 navigation 包挤出包路径。
source /opt/ros/noetic/setup.bash
source ~/ros1_ws/devel/setup.bash
exec roslaunch navigation nav_hector.launch use_teb:=false
