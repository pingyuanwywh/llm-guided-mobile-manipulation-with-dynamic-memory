#!/usr/bin/env python3
# encoding: utf-8
# 探放置/中转路点的 IK 可达性. 打印每个候选点的 pitch/IK/servo1(看转向方向).
import sys
DIST_HW = "/home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/lib/python3/dist-packages"
DIST_CAR = "/home/uavg/ros_car/devel/lib/python3/dist-packages"
for _p in (DIST_HW, DIST_CAR):
    if _p not in sys.path:
        sys.path.insert(0, _p)
import hiwonder_kinematics.transform as tf
from hiwonder_kinematics.inverse_kinematics import get_ik, set_link, set_joint_range

set_link(tf.base_link, tf.link1, tf.link2, tf.link3, tf.tool_link)
set_joint_range(tf.joint1, tf.joint2, tf.joint3, tf.joint4, tf.joint5, 'deg')


def ik(x, y, z, pitch):
    sols = get_ik([x, y, z], float(pitch), [-180, 180], 1)
    if not sols:
        return None
    return [int(round(p)) for p in tf.angle2pulse([sols[0][0][0]])[0]]


# 候选: 高位前伸中转(center) + 两侧放置点, 多个 pitch
cands = [
    ("lift-high-center",   0.18, 0.00, 0.28),
    ("lift-high-center2",  0.20, 0.00, 0.30),
    ("lift-high-center3",  0.15, 0.00, 0.33),
    ("transit-R-high",     0.18, -0.18, 0.26),
    ("transit-R-high2",    0.16, -0.22, 0.24),
    ("transit-L-high",     0.18,  0.18, 0.26),
    ("transit-L-high2",    0.16,  0.22, 0.24),
    ("place-R-mid",        0.22, -0.22, 0.14),
    ("place-R-low",        0.24, -0.20, 0.10),
    ("place-L-mid",        0.22,  0.22, 0.14),
    ("place-L-low",        0.24,  0.20, 0.10),
]
for pitch in (80, 50, 30):
    print("==== pitch=%d ====" % pitch)
    for name, x, y, z in cands:
        p = ik(x, y, z, pitch)
        s1 = p[0] if p else None
        print("  %-18s [%.2f,%.2f,%.2f]  IK=%s  servo1=%s"
              % (name, x, y, z, "OK" if p else "无解", s1))
