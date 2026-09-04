#!/usr/bin/env python3
# encoding: utf-8
'''连采 N 帧, 看每帧被 ROI 保留了几个轮廓、合并后的外接框中心跳不跳。
用来判断 measure 的"连续0.8秒稳定"为什么达不到。'''
import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from hiwonder_sdk import common

CFG = ('/home/uavg/JetRover-Jetson_nano_ros1/ros_ws/src/hiwonder_example/'
       'scripts/rgbd_function/lab_config_can.yaml')
ROI_X_LEFT, ROI_X_RIGHT = 0.35, 0.72
ROI_Y_TOP = float(sys.argv[2]) if len(sys.argv) > 2 else 0.10
ROI_Y_BOTTOM = 0.78
N = 40

rospy.init_node('jitter_check', anonymous=True)
color = common.get_yaml_data(CFG)['lab']['Stereo'][sys.argv[1] if len(sys.argv) > 1 else 'green']
print('ROI_Y_TOP=%.2f' % ROI_Y_TOP)
prev = None
jumps = 0
for i in range(N):
    msg = rospy.wait_for_message('/depth_cam/rgb/image_raw', Image, timeout=5)
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
    rgb = img if msg.encoding == 'rgb8' else cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    h, w = rgb.shape[:2]
    half = cv2.resize(rgb, (int(w / 2), int(h / 2)))
    lab = cv2.cvtColor(cv2.GaussianBlur(half, (3, 3), 3), cv2.COLOR_RGB2LAB)
    mask = cv2.inRange(lab, tuple(color['min']), tuple(color['max']))
    d = cv2.dilate(cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))),
                   cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
    cs = cv2.findContours(d, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
    rx0, rx1 = int(w * ROI_X_LEFT / 2), int(w * ROI_X_RIGHT / 2)
    ry0, ry1 = int(h * ROI_Y_TOP / 2), int(h * ROI_Y_BOTTOM / 2)
    kept, areas = [], []
    for c in cs:
        a = abs(cv2.contourArea(c))
        if a < 10:
            continue
        (cx, cy), _ = cv2.minEnclosingCircle(c)
        if rx0 <= cx <= rx1 and ry0 <= cy <= ry1:
            kept.append(c)
            areas.append(int(a))
    if not kept:
        print('%2d  KEPT=0' % i)
        prev = None
        continue
    x, y, bw, bh = cv2.boundingRect(np.vstack(kept))
    cx, cy = (x + bw / 2) * 2, (y + bh / 2) * 2
    tag = ''
    if prev is not None:
        dx, dy = abs(cx - prev[0]), abs(cy - prev[1])
        if dx >= 45 or dy >= 45:
            tag = '  <<< JUMP dx=%.0f dy=%.0f 会重置稳定计时' % (dx, dy)
            jumps += 1
    print('%2d  kept=%d areas=%s  center=(%.0f,%.0f)%s' % (i, len(kept), areas, cx, cy, tag))
    prev = (cx, cy)
print('---- %d/%d 帧发生 >=45px 跳变 ----' % (jumps, N))
