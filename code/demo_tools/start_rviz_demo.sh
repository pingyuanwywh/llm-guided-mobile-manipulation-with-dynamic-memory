#!/bin/bash
# 本机弹 rviz 看车的地图/雷达/位姿。多行 export 直接塞进 ssh/工具会被压成一行串味,
# 所以固定写成脚本文件再 bash 它(记忆里的老坑)。
export DISPLAY=:1
export DISABLE_ROS1_EOL_WARNINGS=1   # 不然 rviz 会弹 "ROS 1 End-of-Life" 对话框盖在视图上
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
[ -f "$HOME/.jetrover_env" ] && . "$HOME/.jetrover_env"   # 现场参数(车IP/本机IP)，见仓库 jetrover_env.example
CAR_IP="${CAR_IP:?未设置 CAR_IP：请配置 ~/.jetrover_env（参考仓库 jetrover_env.example）}"
MY_IP="${MY_IP:?未设置 MY_IP：请配置 ~/.jetrover_env（参考仓库 jetrover_env.example）}"
export ROS_MASTER_URI=http://${CAR_IP}:11311
export ROS_IP=${MY_IP}
source /opt/ros/noetic/setup.bash
exec rviz -d $HOME/.rviz/jetrover_demo.rviz
