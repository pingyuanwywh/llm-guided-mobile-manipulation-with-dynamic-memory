#!/usr/bin/env python3
# encoding: utf-8
'''抓一帧 RGB 存盘, 画出 ROI 框和 LAB 阈值命中的绿色区域, 用来确认检测锁的到底是不是罐子。'''
import sys
import rospy
import numpy as np
import cv2
from sensor_msgs.msg import Image
from hiwonder_sdk import common

OUT = '/tmp/scene_check.png'
CFG = ('/home/uavg/JetRover-Jetson_nano_ros1/ros_ws/src/hiwonder_example/'
       'scripts/rgbd_function/lab_config_can.yaml')

rospy.init_node('capture_one', anonymous=True)
msg = rospy.wait_for_message('/depth_cam/rgb/image_raw', Image, timeout=10)
img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
print('encoding=%s size=%dx%d' % (msg.encoding, msg.width, msg.height))
rgb = img if msg.encoding == 'rgb8' else cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

h, w = rgb.shape[:2]
color = common.get_yaml_data(CFG)['lab']['Stereo'][sys.argv[1] if len(sys.argv) > 1 else 'green']

# 完全照抄 tracker.proc 的处理
half = cv2.resize(rgb, (int(w / 2), int(h / 2)))
blur = cv2.GaussianBlur(half, (3, 3), 3)
lab = cv2.cvtColor(blur, cv2.COLOR_RGB2LAB)
mask = cv2.inRange(lab, tuple(color['min']), tuple(color['max']))
eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2)))
dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5)))
contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]

# 必须和 final_track_and_grab_y0.py 顶部的 ROI_* 常数保持一致
ROI_X_LEFT, ROI_X_RIGHT = 0.35, 0.72
ROI_Y_TOP, ROI_Y_BOTTOM = 0.10, 0.78
roi_x_min, roi_x_max = int(w * ROI_X_LEFT / 2), int(w * ROI_X_RIGHT / 2)
roi_y_min, roi_y_max = int(h * ROI_Y_TOP / 2), int(h * ROI_Y_BOTTOM / 2)

out = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
# ROI 框(全分辨率坐标 = 半分辨率 *2)
cv2.rectangle(out, (roi_x_min * 2, roi_y_min * 2), (roi_x_max * 2, roi_y_max * 2), (0, 255, 255), 2)
# 绿色掩膜叠一层洋红
m_full = cv2.resize(dilated, (w, h), interpolation=cv2.INTER_NEAREST)
out[m_full > 0] = (0.5 * out[m_full > 0] + 0.5 * np.array([255, 0, 255])).astype(np.uint8)

print('contours=%d' % len(contours))
kept = []
for c in contours:
    area = abs(cv2.contourArea(c))
    (cx, cy), r = cv2.minEnclosingCircle(c)
    inside = roi_x_min <= cx <= roi_x_max and roi_y_min <= cy <= roi_y_max
    if area >= 10:
        print('  area=%7.1f center=(%6.1f,%6.1f) r=%5.1f  %s'
              % (area, cx * 2, cy * 2, r * 2, 'KEPT' if inside else 'dropped_outside_ROI'))
    if area >= 10 and inside:
        kept.append(c)
        cv2.circle(out, (int(cx * 2), int(cy * 2)), 6, (0, 0, 255), -1)

if kept:
    x, y, bw, bh = cv2.boundingRect(np.vstack(kept))
    cv2.rectangle(out, (x * 2, y * 2), ((x + bw) * 2, (y + bh) * 2), (0, 255, 0), 2)
    cx, cy = (x + bw / 2) * 2, (y + bh / 2) * 2
    cv2.drawMarker(out, (int(cx), int(cy)), (255, 255, 255), cv2.MARKER_CROSS, 20, 2)
    print('FINAL center=(%.1f, %.1f)  bbox=(%d,%d,%d,%d) 全分辨率' % (cx, cy, x * 2, y * 2, bw * 2, bh * 2))
else:
    print('FINAL: 没有幸存轮廓')

cv2.imwrite(OUT, out)
print('saved ' + OUT)
