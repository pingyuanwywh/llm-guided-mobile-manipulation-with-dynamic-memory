#!/usr/bin/env bash
set -eo pipefail

TARGET_COLOR="${1:-green}"
LOW_GRASP_PITCH="${2:-10}"
HIGH_GRASP_PITCH="${3:-30}"
FINAL_SCRIPT="/home/uavg/JetRover-Jetson_nano_ros1/ros_ws/src/hiwonder_example/scripts/rgbd_function/final_track_and_grab.py"
FINAL_LOG="/tmp/final_track_and_grab.log"

start_tmux() {
  local name="$1"
  local cmd="$2"
  if tmux has-session -t "${name}" 2>/dev/null; then
    echo "${name} already running"
  else
    tmux new-session -d -s "${name}" "${cmd}"
    echo "${name} started"
  fi
}

start_tmux vision_roscore "source /opt/ros/noetic/setup.bash; roscore"
sleep 3

start_tmux board "source /opt/ros/noetic/setup.bash; source /home/uavg/ros_car/devel/setup.bash; roslaunch ros_robot_controller board_node.launch"
sleep 2

start_tmux official_depth "source /opt/ros/noetic/setup.bash; source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash; roslaunch hiwonder_peripherals depth_cam.launch"
sleep 3

start_tmux official_kinematics "export MACHINE_TYPE=JetRover_Mecanum; source /opt/ros/noetic/setup.bash; source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash; roslaunch hiwonder_kinematics kinematics_node.launch"
sleep 2

start_tmux official_servo_manager "export MACHINE_TYPE=JetRover_Mecanum; source /opt/ros/noetic/setup.bash; source /home/uavg/ros_car/devel/setup.bash; source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash; roslaunch hiwonder_servo_controllers start.launch"
sleep 4

tmux kill-session -t official_track_grab 2>/dev/null || true
tmux kill-session -t final_track_grab 2>/dev/null || true
rm -f "${FINAL_LOG}"
tmux new-session -d -s final_track_grab "export DISPLAY=:0; export XAUTHORITY=/home/uavg/.Xauthority; export MACHINE_TYPE=JetRover_Mecanum; source /opt/ros/noetic/setup.bash; source /home/uavg/ros_car/devel/setup.bash; source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash; python3 -u ${FINAL_SCRIPT} __name:=track_and_grab _start:=false _color:=${TARGET_COLOR} _low_grasp_pitch:=${LOW_GRASP_PITCH} _high_grasp_pitch:=${HIGH_GRASP_PITCH} 2>&1 | tee ${FINAL_LOG}"
echo "final_track_grab started"
sleep 4

source /opt/ros/noetic/setup.bash
source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash
rosservice list | grep /track_and_grab
echo "log: ${FINAL_LOG}"
