#!/bin/bash
# 启动 RPLidar A1。用 by-id 稳定路径, 不怕 ttyUSB 号变来变去。
source /opt/ros/noetic/setup.bash
source ~/rplidar_ws/devel/setup.bash
exec roslaunch rplidar_ros rplidar_a1.launch serial_port:=/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0
