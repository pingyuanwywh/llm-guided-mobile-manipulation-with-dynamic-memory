#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""scan_probe.py -- 发运动指令前先看一眼车周围有什么 (2026-07-28)

07-26 的教训: 因为没看 /scan 就瞎选方向, 白费了两组测量(车侧面贴着障碍, DWA 原地摆死)。
这个脚本 2 秒给出 8 个扇区的最近距离 + 全场最近点, 判"往哪个方向有空地"用。
0 度 = 雷达 x 正方向(标称车头), 逆时针为正。
"""
import argparse
import math
import sys

import rospy
from sensor_msgs.msg import LaserScan


def fine(s, front_deg, half=90.0, step=15.0):
    """车头前方 +-half 度, 每 step 度一格的最近距离。判"障碍挡住多少度、缝在哪边"用。
    角度按**车体**给(0=正前方, 正=左), 已经把雷达那 180 度装反的事折进去了。"""
    bins = {}
    n = int(2 * half / step) + 1
    for i, r in enumerate(s.ranges):
        if not (s.range_min < r < s.range_max) or math.isnan(r):
            continue
        lang = math.degrees(s.angle_min + i * s.angle_increment)
        body = (lang - front_deg + 180.0) % 360.0 - 180.0   # 转成车体角
        if abs(body) > half + step / 2:
            continue
        k = int(round(body / step))
        bins[k] = min(bins.get(k, float('inf')), r)
    print('车体角(0=正前, +=左)   最近距离')
    for k in range(int(half / step), -int(half / step) - 1, -1):
        v = bins.get(k)
        bar = '#' * int(min(v, 3.0) / 0.1) if v else ''
        print('  %+4d deg  %8s  %s' % (k * step, ('%.2f m' % v) if v else '(空)', bar))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--front-deg', dest='front_deg', type=float, default=None,
                   help='车头在雷达系里是多少度。这台车 base_to_laser 的 yaw=3.14159 => 传 180。')
    p.add_argument('--fine', action='store_true', help='车头前方 +-90 度按 15 度一格细看')
    a = p.parse_args(rospy.myargv()[1:])
    rospy.init_node('scan_probe', anonymous=True)
    try:
        s = rospy.wait_for_message('/scan', LaserScan, timeout=5.0)
    except Exception:
        print('!! 拿不到 /scan')
        return 1
    sect = {}
    nearest = (float('inf'), 0.0)
    for i, r in enumerate(s.ranges):
        if not (s.range_min < r < s.range_max) or math.isnan(r):
            continue
        ang = math.degrees((s.angle_min + i * s.angle_increment + math.pi) % (2 * math.pi) - math.pi)
        k = int(round(ang / 45.0)) % 8
        sect[k] = min(sect.get(k, float('inf')), r)
        if r < nearest[0]:
            nearest = (r, ang)
    names = {0: '  0 front', 1: ' 45 fl', 2: ' 90 left', 3: '135 rl',
             4: '180 rear', 5: '-135 rr', 6: '-90 right', 7: '-45 fr'}
    print('sector      min_range')
    for k in range(8):
        v = sect.get(k)
        print('%-11s %s' % (names[k], ('%.2f m' % v) if v else '(none)'))
    print('\nnearest overall: %.2f m at %+.0f deg' % nearest)
    print('scan points: %d, range %.2f~%.2f' % (len(s.ranges), s.range_min, s.range_max))
    if a.fine:
        print()
        fine(s, a.front_deg if a.front_deg is not None else 180.0)
    return 0


if __name__ == '__main__':
    sys.exit(main())
