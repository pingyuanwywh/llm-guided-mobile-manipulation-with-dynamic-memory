#!/usr/bin/env python3
# encoding: utf-8
'''【只读, 车不动】把当前 /scan 按当前 TF 投到 map 系, 量每个点到最近占据格的距离。
   健康: 中位 ~0.000m, 94~99% 落在 10cm 内。漂了: 中位 0.335m, 只有 5% 在 10cm 内。
   不发布任何指令, 不碰 /cmd_vel。'''
import math, numpy as np, rospy, tf2_ros
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from scipy import ndimage

rospy.init_node('align_check', anonymous=True)
buf = tf2_ros.Buffer(); tf2_ros.TransformListener(buf)
grid = rospy.wait_for_message('/map', OccupancyGrid, timeout=20)
scan = rospy.wait_for_message('/scan', LaserScan, timeout=10)
rospy.sleep(1.0)
tr = buf.lookup_transform('map', scan.header.frame_id, rospy.Time(0), rospy.Duration(5.0))

W, H, res = grid.info.width, grid.info.height, grid.info.resolution
ox, oy = grid.info.origin.position.x, grid.info.origin.position.y
occ = (np.array(grid.data, dtype=np.int16).reshape(H, W) > 65)
if occ.sum() == 0:
    print('地图里没有占据格'); raise SystemExit
dist = ndimage.distance_transform_edt(~occ) * res

q = tr.transform.rotation; t = tr.transform.translation
yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
ds = []
for i, r in enumerate(scan.ranges):
    if not (scan.range_min < r < scan.range_max) or math.isinf(r) or math.isnan(r):
        continue
    a = scan.angle_min + i*scan.angle_increment
    lx, ly = r*math.cos(a), r*math.sin(a)
    mx = t.x + lx*math.cos(yaw) - ly*math.sin(yaw)
    my = t.y + lx*math.sin(yaw) + ly*math.cos(yaw)
    cx, cy = int((mx-ox)/res), int((my-oy)/res)
    if 0 <= cx < W and 0 <= cy < H:
        ds.append(dist[cy, cx])
ds = np.array(ds)
print('投影点数 = %d' % len(ds))
print('中位 = %.3f m   均值 = %.3f m' % (np.median(ds), ds.mean()))
print('落在 10cm 内 = %.0f%%   20cm 内 = %.0f%%' % (100*(ds<0.10).mean(), 100*(ds<0.20).mean()))
print('判据: 健康=中位 0.000 / 94~99%% 在 10cm 内;  漂了=中位 0.335 / 仅 5%%')
print('>>> %s' % ('健康' if np.median(ds) <= 0.05 else '已漂, 中位超过 0.05m 的停机线'))
