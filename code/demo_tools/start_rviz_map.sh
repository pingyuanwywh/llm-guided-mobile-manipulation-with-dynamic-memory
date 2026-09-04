#!/bin/bash
# 建图用 rviz(本机桌面)。与 start_rviz_demo.sh 逐字相同, 只换配置文件:
#   建图看 jetrover_nav.rviz (Map + LaserScan + TF, TopDownOrtho)
#   录制看 jetrover_demo.rviz (上帝视角, 无面板, 走虚拟屏 start_rviz_virtual.sh)
export DISPLAY=:1
export DISABLE_ROS1_EOL_WARNINGS=1   # 不然 rviz 会弹 "ROS 1 End-of-Life" 对话框盖在视图上
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
[ -f "$HOME/.jetrover_env" ] && . "$HOME/.jetrover_env"   # 现场参数(车IP/本机IP)，见仓库 jetrover_env.example
CAR_IP="${CAR_IP:?未设置 CAR_IP：请配置 ~/.jetrover_env（参考仓库 jetrover_env.example）}"
MY_IP="${MY_IP:?未设置 MY_IP：请配置 ~/.jetrover_env（参考仓库 jetrover_env.example）}"
export ROS_MASTER_URI=http://${CAR_IP}:11311
export ROS_IP=${MY_IP}
source /opt/ros/noetic/setup.bash
exec rviz -d $HOME/.rviz/jetrover_nav.rviz
