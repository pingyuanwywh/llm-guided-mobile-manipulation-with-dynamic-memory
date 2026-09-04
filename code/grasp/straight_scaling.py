#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""straight_scaling.py -- 直行偏航到底是「按米累积」还是「每次起停固定踢一下」(2026-07-28)

## 为什么要有这个脚本
07-26 测过一次: 命令纯直行(angular.z 恒 0) 0.48m, 陀螺仪测到净转角 +-1.85 度,
前进歪一边、后退歪另一边、大小几乎相同(完美镜像), 当时判成"机械不对称, 约 3.9 度/米"。
**但那是只在一个距离上测的**, 而"完美镜像"这个特征, 下面两种病因都满足:
  (A) 持续偏航(麦轮出力不均) => 净转角 **正比于距离**, 跑 4m 的长腿要歪 15 度(横偏半米), 必须标定电机;
  (B) 起停冲击(加减速时的一次性偏航踢) => 净转角 **与距离无关**, 跑 4m 也只歪 1.85 度(横偏 6cm), 可以无视。
用户实测"给底层命令让车跑直线, 车是能跑直线的" => 更像 (B)。本脚本用三个距离把两者分开。

## 做法
同一次标定下, 依次跑 0.5 / 1.0 / 1.5m, 每个距离前进一趟再后退回来, 全程 angular.z = 0(开环, move_base 不参与)。
拿陀螺仪积分的净转角对距离做一次线性拟合 yaw = a + b*d:
  a 大 b 小 => 起停冲击(固定踢), 别标定电机, 直接上"直线腿不走 DWA"的方案就行
  b 大      => 真的按米累积, b 就是每米歪几度, 标定电机才有意义

⚠️ 车会真的往前开。默认最远那趟 1.5m, 车头前方至少要留 2m 空地。脚本会先看 /scan 自检。
"""
import argparse
import math
import subprocess
import sys
import time

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, LaserScan


def yaw_from_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Runner:
    def __init__(self, flip=True):
        self.flip = flip
        self.gyro = 0.0
        self.bias = 0.0
        self.samples = []
        self.last_t = None
        self.scan = None
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        rospy.Subscriber('/board/imu_raw', Imu, self.on_imu, queue_size=200)
        rospy.Subscriber('/scan', LaserScan, self.on_scan, queue_size=1)
        self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def on_imu(self, msg):
        t = msg.header.stamp.to_sec()
        gz = -msg.angular_velocity.z if self.flip else msg.angular_velocity.z
        if self.last_t is not None:
            self.gyro += (gz - self.bias) * (t - self.last_t)
        self.last_t = t
        self.samples.append(gz)

    def on_scan(self, msg):
        self.scan = msg

    def calib(self, secs):
        self.samples = []
        rospy.sleep(secs)
        if len(self.samples) > 10:
            self.bias = sum(self.samples) / len(self.samples)
        self.last_t = None
        return self.bias

    def pose(self):
        try:
            tr = self.buf.lookup_transform('map', 'base_footprint',
                                           rospy.Time(0), rospy.Duration(0.5))
            t = tr.transform.translation
            return t.x, t.y, yaw_from_q(tr.transform.rotation)
        except Exception:
            return None

    def sector(self, center_deg, half=20.0):
        """某个方向扇区内的最近距离。0 度 = 雷达坐标系的 x 正方向(标称车头)。"""
        s = self.scan
        if s is None:
            return None
        best = float('inf')
        for i, r in enumerate(s.ranges):
            if not (s.range_min < r < s.range_max) or math.isnan(r):
                continue
            ang = math.degrees(wrap(s.angle_min + i * s.angle_increment))
            if abs(wrap(math.radians(ang - center_deg))) <= math.radians(half):
                best = min(best, r)
        return None if best == float('inf') else best

    def run(self, vx, dur):
        """开环直行 dur 秒, angular.z 全程 0。返回(真实净转角 rad, hector 净转角 rad, map 位移 m)。"""
        p0, g0 = self.pose(), self.gyro
        m = Twist()
        m.linear.x = vx
        r = rospy.Rate(20)
        t0 = time.time()
        while time.time() - t0 < dur and not rospy.is_shutdown():
            self.cmd.publish(m)
            r.sleep()
        self.cmd.publish(Twist())
        rospy.sleep(1.5)
        p1 = self.pose()
        return (self.gyro - g0, wrap(p1[2] - p0[2]),
                math.hypot(p1[0] - p0[0], p1[1] - p0[1]))


def fit(xs, ys):
    """最小二乘 y = a + b*x。点少的时候也稳。"""
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxx = sum(x * x for x in xs)
    sxy = sum(x * y for x, y in zip(xs, ys))
    den = n * sxx - sx * sx
    if abs(den) < 1e-12:
        return sy / n, 0.0
    b = (n * sxy - sx * sy) / den
    return (sy - b * sx) / n, b


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--speed', type=float, default=0.08, help='直行速度 m/s')
    p.add_argument('--dists', default='0.5,1.0,1.5', help='测哪几个距离(米), 逗号分隔')
    p.add_argument('--cal', type=float, default=5.0, help='每趟前静止标定零偏几秒')
    p.add_argument('--margin', type=float, default=0.5, help='最远那趟之外还要留多少空地')
    p.add_argument('--front-deg', dest='front_deg', type=float, default=180.0,
                   help='车头在**雷达坐标系**里是多少度。这台车 base_to_laser 的 yaw 是 3.14159 '
                        '(雷达反装), 所以车头 = 雷达系的 180 度, 不是 0 度。搞反了安全自检就是反的。')
    a = p.parse_args(rospy.myargv()[1:])

    if not subprocess.run(['pgrep', '-f', 'cmd_vel_to_motor'],
                          stdout=subprocess.PIPE).stdout.strip():
        print('!! cmd_vel_to_motor.py 没在跑 => 车不会动。')
        return 1

    dists = [float(x) for x in a.dists.split(',')]
    rospy.init_node('straight_scaling', anonymous=True)
    rn = Runner()
    rospy.sleep(1.5)
    if rn.pose() is None:
        print('!! 拿不到 map->base_footprint, hector 起了吗?')
        return 1

    need = max(dists) + a.margin
    front = rn.sector(a.front_deg)
    rear = rn.sector(a.front_deg + 180.0)
    print('车头方向最近障碍 %s m, 车尾方向 %s m (需要车头 >= %.2f m)'
          % ('%.2f' % front if front else '?', '%.2f' % rear if rear else '?', need))
    if front is not None and front < need:
        print('!! 车头前方空地不够。要么挪车, 要么用 --dists 0.4,0.8,1.2 之类跑短的。')
        return 1

    print('\n%-6s %-4s | %-12s %-12s %-10s %-9s %s'
          % ('距离', '方向', '陀螺净转角', 'hector转角', 'map位移', '实际/指令', '度/米'))
    print('-' * 78)
    fwd, bwd = [], []
    front_before = front
    for d in dists:
        dur = d / a.speed
        for sign, name in ((+1, '前进'), (-1, '后退')):
            rn.calib(a.cal)
            g, h, disp = rn.run(sign * a.speed, dur)
            gd, hd = math.degrees(g), math.degrees(h)
            print('%-6.2f %-4s | %+12.2f %+12.2f %10.3f %9.0f%% %+8.2f'
                  % (d, name, gd, hd, disp, 100 * disp / d if d else 0, gd / d))
            (fwd if sign > 0 else bwd).append((d, gd))
            rospy.sleep(1.0)

    front_after = rn.sector(a.front_deg)
    print('\n(自检) 车头扇区距离: 开跑前 %s -> 跑完 %s —— 一来一回应该接近相等; '
          '差很多说明车没回到原位或扇区方向定义不对'
          % ('%.2f' % front_before if front_before else '?',
             '%.2f' % front_after if front_after else '?'))

    print('\n================ 拟合 yaw = a + b*d ================')
    for name, data in (('前进', fwd), ('后退', bwd)):
        if len(data) >= 2:
            aa, bb = fit([x for x, _ in data], [y for _, y in data])
            print('%s: 固定项 a = %+.2f deg (起停冲击) | 斜率 b = %+.2f deg/m (持续偏航)'
                  % (name, aa, bb))
            print('      => 跑 4m 的长腿按这个拟合会歪 %+.2f deg, 横偏约 %.2f m'
                  % (aa + bb * 4, abs(math.sin(math.radians(abs(aa + bb * 4)) / 2) * 4)))
    print('\n判读: |b| < 1 deg/m 且 |a| 明显 => 起停冲击为主, **别去标定电机**, 长腿不会累积;')
    print('      |b| > 2 deg/m       => 真的按米歪, 标定电机/补一个前馈才有意义。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
