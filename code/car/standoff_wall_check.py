#!/usr/bin/env python3
# encoding: utf-8
"""【只读, 车不动】量一串 map 系坐标到最近占据格的距离。不发布任何指令。
   判据(记忆 08-15): standoff 离墙 <0.35m 必须换摆位, 指望 nav 重试自救=整场作废。
   用法: python3 standoff_wall_check.py 名字:x:y [名字:x:y ...]
"""
import sys, numpy as np, rospy
from nav_msgs.msg import OccupancyGrid
from scipy import ndimage

rospy.init_node("standoff_wall_check", anonymous=True)
g = rospy.wait_for_message("/map", OccupancyGrid, timeout=20)
W, H, res = g.info.width, g.info.height, g.info.resolution
ox, oy = g.info.origin.position.x, g.info.origin.position.y
occ = (np.array(g.data, dtype=np.int16).reshape(H, W) > 65)
dist = ndimage.distance_transform_edt(~occ) * res
print("地图 %dx%d res=%.3f 占据格=%d 原点(%.2f,%.2f)" % (W, H, res, occ.sum(), ox, oy))
for a in sys.argv[1:]:
    n, xs, ys = a.split(":")
    x, y = float(xs), float(ys)
    j, i = int((x - ox) / res), int((y - oy) / res)
    if not (0 <= i < H and 0 <= j < W):
        print("  %-11s (%7.3f,%7.3f)  地图外" % (n, x, y)); continue
    d = dist[i, j]
    unk = " 未知区" if g.data[i * W + j] < 0 else ""
    f = "换摆位 <0.35" if d < 0.35 else ("偏紧 0.35~0.45" if d < 0.45 else "OK")
    print("  %-11s (%7.3f,%7.3f)  离最近占据格 %.3f m  %s%s" % (n, x, y, d, f, unk))
