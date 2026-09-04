#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""yaw_check.py -- 量"车头朝向"这一维到底有多脏 (2026-07-26)

同时采两路朝向信息, 车**不用动**:
  1. IMU 陀螺仪 z (/board/imu_raw 的 angular_velocity.z, 100Hz)
     -> 零偏(bias) / 噪声(std) / 去零偏后积分出来的随机游走漂移
  2. hector 的朝向 (TF map->base_footprint 的 yaw)
     -> 静止时它自己抖多少度。**DWA 就是在追这个数**。

判读:
  hector 静止抖动 >~0.5 deg  => "一扭一扭"里有一大半是 DWA 在追估计噪声, 接 IMU 值得做。
  hector 静止抖动 <~0.2 deg  => 扭不是估计噪声来的, 去查 DWA 参数或麦轮机械偏航, 别急着上 EKF。
  IMU bias 去掉后 60s 漂移 <~2 deg => 陀螺仪足够好, 拿它做 hector 的转动先验能成。

用法: python3 ~/yaw_check.py [秒数, 默认 20]
"""
import sys
import math
import time

import rospy
import tf2_ros
from sensor_msgs.msg import Imu


def yaw_from_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class YawCheck:
    def __init__(self):
        self.gz = []          # (t, angular_velocity.z)
        self.acc = []         # |a|, 用来确认车真的没动
        rospy.Subscriber('/board/imu_raw', Imu, self.cb, queue_size=200)
        self.buf = tf2_ros.Buffer()
        self.ls = tf2_ros.TransformListener(self.buf)

    def cb(self, msg):
        self.gz.append((msg.header.stamp.to_sec(), msg.angular_velocity.z))
        a = msg.linear_acceleration
        self.acc.append(math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z))

    def tf_yaw(self, parent, child):
        try:
            t = self.buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0.05))
            return yaw_from_q(t.transform.rotation)
        except Exception:
            return None


def stats(v):
    n = len(v)
    m = sum(v) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / n) if n > 1 else 0.0
    return m, sd, min(v), max(v)


def main():
    # 用法: yaw_check.py [秒数] [parent] [child]
    #   默认看 map->base_footprint = "hector 给 DWA 的那个朝向"
    #   起了 EKF 之后还可以看 odom->base_footprint = "陀螺仪推算出来的那段平滑变换"
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    parent = sys.argv[2] if len(sys.argv) > 2 else 'map'
    child = sys.argv[3] if len(sys.argv) > 3 else 'base_footprint'
    rospy.init_node('yaw_check', anonymous=True)
    yc = YawCheck()
    rospy.sleep(1.5)

    hy = []
    t0 = time.time()
    print('采样 %.0fs (%s -> %s), 车别动...' % (dur, parent, child))
    rate = rospy.Rate(20)
    while time.time() - t0 < dur and not rospy.is_shutdown():
        y = yc.tf_yaw(parent, child)
        if y is not None:
            hy.append((time.time(), y))
        rate.sleep()

    print('\n================ IMU 陀螺仪 z (%d 帧) ================' % len(yc.gz))
    if len(yc.gz) < 10:
        print('  没收到 IMU 数据 —— board_node 起了吗?')
    else:
        g = [v for _, v in yc.gz]
        m, sd, lo, hi = stats(g)
        print('  零偏 bias   = %+.5f rad/s  (%+.3f deg/s)' % (m, math.degrees(m)))
        print('  噪声 std    =  %.5f rad/s  (%.3f deg/s)' % (sd, math.degrees(sd)))
        print('  峰峰       = %+.4f .. %+.4f rad/s' % (lo, hi))
        span = yc.gz[-1][0] - yc.gz[0][0]
        # 不去零偏: 直接积分, 这是"什么都不补"时的漂移
        raw = 0.0
        deb = 0.0
        prev = yc.gz[0][0]
        for t, v in yc.gz[1:]:
            dt = t - prev
            prev = t
            raw += v * dt
            deb += (v - m) * dt
        print('  %.0fs 内积分漂移: 不去零偏 %+.2f deg / 去零偏后 %+.3f deg'
              % (span, math.degrees(raw), math.degrees(deb)))
        print('  外推 60s:        不去零偏 %+.1f deg / 去零偏后 %+.2f deg'
              % (math.degrees(raw) / span * 60, math.degrees(deb) / span * 60))
        am, asd, _, _ = stats(yc.acc)
        print('  |加速度| = %.3f m/s2 (std %.4f)  <- 应 ~9.8 且 std 很小 = 车确实没动' % (am, asd))

    print('\n================ TF 朝向 (%s -> %s, %d 帧) ================' % (parent, child, len(hy)))
    if len(hy) < 10:
        print('  没拿到 TF —— 对应节点起了吗?')
    else:
        ys = [math.degrees(y) for _, y in hy]
        base = ys[0]
        rel = [(y - base + 180) % 360 - 180 for y in ys]
        m, sd, lo, hi = stats(rel)
        print('  静止抖动 std = %.3f deg' % sd)
        print('  峰峰         = %.3f deg  (%.3f .. %.3f)' % (hi - lo, lo, hi))
        print('  整段净漂移   = %+.3f deg' % rel[-1])
        # 逐帧跳变: 相邻两帧差多少, 反映"噪声"而非"慢漂"
        d = [abs(rel[i] - rel[i - 1]) for i in range(1, len(rel))]
        dm, dsd, _, dhi = stats(d)
        print('  相邻帧跳变   = 均值 %.4f deg, 最大 %.3f deg' % (dm, dhi))
        print('\n  判读: >0.5 deg 抖 => DWA 在追噪声, IMU/EKF 值得上;'
              ' <0.2 deg => 扭不是估计噪声来的, 先查 DWA 参数/麦轮机械偏航')


if __name__ == '__main__':
    main()
