#!/usr/bin/env python3
# encoding: utf-8
# 一次性场景采集: 摆观察姿势 -> 掐激光 -> 存 depth/rgb/K/FK末端 供离线设计深度凸起抓取.
import sys, time
DIST_HW = "/home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/lib/python3/dist-packages"
DIST_CAR = "/home/uavg/ros_car/devel/lib/python3/dist-packages"
for _p in (DIST_HW, DIST_CAR):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import numpy as np
import rospy
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool
import hiwonder_kinematics.transform as tf
from hiwonder_kinematics.forward_kinematics import ForwardKinematics
from hiwonder_kinematics.inverse_kinematics import set_link, set_joint_range
from ros_robot_controller.srv import GetBusServoState, GetBusServoStateRequest
from ros_robot_controller.msg import GetBusServoCmd, SetBusServoState, BusServoState

OBSERVE_POSE = [(1, 500), (2, 720), (3, 100), (4, 120), (5, 500), (10, 200)]
OUT = "/home/uavg/scene"


def servo_state(sid, pulse):
    st = BusServoState()
    st.present_id = [1, int(sid)]; st.target_id = [0, 0]
    st.position = [1, int(round(pulse))]; st.offset = [0, 0]
    st.voltage = [0]; st.temperature = [0]; st.position_limit = [0, 0, 0]
    st.voltage_limit = [0, 0, 0]; st.max_temperature_limit = [0, 0]
    st.enable_torque = [0, 0]; st.save_offset = [0, 0]; st.stop = [0, 0]
    return st


rospy.init_node("scene_capture", anonymous=True)
board = rospy.Publisher('/board/bus_servo/set_state', SetBusServoState, queue_size=1, latch=True)
rospy.wait_for_service('/board/bus_servo/get_state', timeout=10)
get_state = rospy.ServiceProxy('/board/bus_servo/get_state', GetBusServoState)
t = time.time()
while board.get_num_connections() == 0 and time.time() - t < 3:
    rospy.sleep(0.1)

msg = SetBusServoState(); msg.duration = 1.5
for sid, pos in OBSERVE_POSE:
    msg.state.append(servo_state(sid, pos))
board.publish(msg); rospy.sleep(2.0)

try:
    rospy.wait_for_service('/depth_cam/set_ldp', timeout=10)
    rospy.ServiceProxy('/depth_cam/set_ldp', SetBool)(False)
except Exception as e:
    print("ldp err:", e)
rospy.sleep(1.5)

depth_msg = rospy.wait_for_message('/depth_cam/depth/image_raw', Image, timeout=5)
rgb_msg = rospy.wait_for_message('/depth_cam/rgb/image_raw', Image, timeout=5)
info = rospy.wait_for_message('/depth_cam/depth/camera_info', CameraInfo, timeout=5)
depth = np.ndarray((depth_msg.height, depth_msg.width), np.uint16, depth_msg.data).copy()
rgb = np.ndarray((rgb_msg.height, rgb_msg.width, 3), np.uint8, rgb_msg.data).copy()

set_link(tf.base_link, tf.link1, tf.link2, tf.link3, tf.tool_link)
set_joint_range(tf.joint1, tf.joint2, tf.joint3, tf.joint4, tf.joint5, 'deg')
fk = ForwardKinematics('')
fk.set_link(tf.base_link, tf.link1, tf.link2, tf.link3, tf.tool_link)
fk.set_joint_range(tf.joint1, tf.joint2, tf.joint3, tf.joint4, tf.joint5, 'deg')
req = GetBusServoStateRequest()
req.cmd = [GetBusServoCmd(id=s, get_position=1) for s in range(1, 6)]
resp = get_state(req)
pulses = [int(it.position[0]) for it in resp.state]
ang = tf.pulse2angle(pulses)
pos, quat = fk.get_fk(list(ang))

np.save(OUT + "_depth.npy", depth)
np.save(OUT + "_rgb.npy", rgb)
np.save(OUT + "_K.npy", np.array(list(info.K)))
np.save(OUT + "_fkpos.npy", np.array(pos))
np.save(OUT + "_fkquat.npy", np.array([quat.x, quat.y, quat.z, quat.w]))

valid = depth[depth > 0]
print("DEPTH shape=%s dtype=%s valid=%d min=%d max=%d" % (
    depth.shape, depth.dtype, valid.size,
    int(valid.min()) if valid.size else 0, int(depth.max())))
print("RGB shape=%s" % (rgb.shape,))
print("K=", [round(x, 2) for x in info.K])
print("pulses=", pulses)
print("fk_pos=", [round(float(x), 4) for x in pos])
print("fk_quat=", [round(float(quat.x), 4), round(float(quat.y), 4), round(float(quat.z), 4), round(float(quat.w), 4)])
print("SAVED", OUT + "_{depth,rgb,K,fkpos,fkquat}.npy")
