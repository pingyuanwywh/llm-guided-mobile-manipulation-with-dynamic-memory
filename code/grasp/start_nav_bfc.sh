#!/bin/bash
# base_footprint 挪到真实旋转中心(雷达后方0.10m)的导航栈。回退 = 用回 start_nav.sh
source /opt/ros/noetic/setup.bash
source ~/ros1_ws/devel/setup.bash
exec roslaunch ~/nav_hector_bfc.launch use_teb:=false
