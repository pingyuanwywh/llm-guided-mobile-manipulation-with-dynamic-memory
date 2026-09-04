#!/bin/bash
# 深度凸起抓取 runner (立即返回). 用法: DRY_RUN=false Z_ADJ=-0.02 X_ADJ=0.0 bash run_depth.sh
pkill -f grab_depth.py 2>/dev/null; pkill -f grab_can.py 2>/dev/null; sleep 1
mkdir -p /home/uavg/Log; : > /home/uavg/Log/grab_depth.log
source /opt/ros/noetic/setup.bash
source ~/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash
source ~/ros_car/devel/setup.bash
export MACHINE_TYPE=JetRover_Mecanum
setsid nohup python3 -u /home/uavg/grab_depth.py \
  _dry_run:=${DRY_RUN:-true} _x_adj:=${X_ADJ:-0.0} _y_adj:=${Y_ADJ:-0.0} \
  _z_adj:=${Z_ADJ:-0.0} _above:=${ABOVE:-0.03} _nframes:=${NF:-5} \
  _radius:=${RADIUS:-0.027} _grasp_h:=${GRASP_H:-0.10} \
  _aim:=${AIM:-true} _pix_gain:=${PIX_GAIN:-0.6} \
  > /home/uavg/Log/grab_depth.log 2>&1 < /dev/null &
disown
echo "LAUNCHED grab_depth dry=${DRY_RUN:-true} radius=${RADIUS:-0.027} grasp_h=${GRASP_H:-0.10} x_adj=${X_ADJ:-0.0} z_adj=${Z_ADJ:-0.0} aim=${AIM:-true}"
