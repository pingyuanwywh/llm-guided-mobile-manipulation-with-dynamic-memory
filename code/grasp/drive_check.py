#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drive_check.py -- 直线行驶时把"扭"的三个候选病因分开 (2026-07-26)

静止测不出扭的病因(2026-07-26 实测 A/B 两组静止抖动都只有 0.1 deg, 都够不上门限)。
扭是在**运动中**发生的, 这个脚本就是跑一段直线, 同时记三路数据把病因拆开:

  (a) 物理层 —— 麦轮出力不均/打滑, 命令纯直行车也会歪。
      看**陀螺仪积分出来的净转角**: 陀螺仪直接测角速度, 不依赖任何定位算法,
      它说车转了多少度就是转了多少度。命令 angular.z 全程为 0 却转了 >3 deg => 物理歪。
  (b) 估计层 —— hector 运动中的朝向误差。
      看 **hector 转角 - 陀螺仪转角**。静止时 hector 最轻松(两帧几乎相同),
      运动时才见真章(07-23 实测纯旋转下位置估计滑了 0.42m)。
  (c) 规划层 —— DWA 自己在拧车头。
      本脚本发的是**开环恒定速度指令, angular.z 恒为 0**, move_base 完全不参与,
      所以这一路被排除在外。要测 (c) 得用 nav_goto 跑一条腿再看 /cmd_vel。

⚠️ **车会真的动**。跑之前确认车头前方有足够空地(默认 0.06m/s x 8s = 约 0.5m)。
   脚本会先检查 cmd_vel_to_motor.py 在跑(不在跑车不会动, 那就成了空测)。
   全程 angular.z = 0, 不转车, 所以不会触发 hector 的旋转漂移。

用法: python3 ~/drive_check.py [--speed 0.06] [--dur 8] [--vy 0]
"""
import argparse
import math
import subprocess
import sys
import time

import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu


def yaw_from_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class DriveCheck:
    def __init__(self, flip=True):
        self.flip = flip             # z 轴朝下装 => 原始 z 取反才符合 ROS 约定
        self.gyro_yaw = 0.0          # 陀螺仪积分出来的相对转角(rad, 已按 flip 修正)
        self.gyro_yaw_raw = 0.0      # 未修正的, 只给"符号判定"那一节用
        self.last_acc_z = float('nan')
        self.gz_last_t = None
        self.gz_bias = 0.0
        self.gz_bias_raw = 0.0
        self.gz_samples = []
        self.gz_samples_raw = []
        self.buf = tf2_ros.Buffer()
        tf2_ros.TransformListener(self.buf)
        rospy.Subscriber('/board/imu_raw', Imu, self.on_imu, queue_size=200)
        self.cmd = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    def on_imu(self, msg):
        t = msg.header.stamp.to_sec()
        # 本脚本订阅的是**原始** /board/imu_raw(不是 imu_debias 洗过的 /imu/data),
        # 这样它是一路独立校验、不受管道 bug 影响。但这颗 IMU z 轴朝下装,
        # 原始 z 的符号与 ROS 约定相反, 所以这里默认取反后再用。
        raw_gz = msg.angular_velocity.z
        gz = -raw_gz if self.flip else raw_gz
        self.last_acc_z = msg.linear_acceleration.z
        if self.gz_last_t is not None:
            dt = t - self.gz_last_t
            self.gyro_yaw += (gz - self.gz_bias) * dt
            self.gyro_yaw_raw += (raw_gz - self.gz_bias_raw) * dt
        self.gz_last_t = t
        self.gz_samples.append(gz)
        self.gz_samples_raw.append(raw_gz)

    def calib(self, secs):
        """静止采一段求零偏 —— 不去零偏的话 8 秒会凭空多出 0.7 度, 足以污染结论。"""
        self.gz_samples = []
        self.gz_samples_raw = []
        rospy.sleep(secs)
        if len(self.gz_samples) > 10:
            self.gz_bias = sum(self.gz_samples) / len(self.gz_samples)
            self.gz_bias_raw = sum(self.gz_samples_raw) / len(self.gz_samples_raw)
        self.gyro_yaw = 0.0
        self.gyro_yaw_raw = 0.0
        self.gz_last_t = None
        return self.gz_bias

    def tf_pose(self, parent, child):
        try:
            t = self.buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0.3))
            tr = t.transform.translation
            return tr.x, tr.y, yaw_from_q(t.transform.rotation)
        except Exception:
            return None


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--speed', type=float, default=0.06, help='前进速度 m/s')
    p.add_argument('--vy', type=float, default=0.0, help='横移速度 m/s')
    p.add_argument('--wz', type=float, default=0.0,
                   help='原地旋转角速度 rad/s。**用来定陀螺仪的符号约定**: ROS 约定 '
                        'angular.z 为正 = 逆时针 = yaw 增大。发一个已知符号的转动指令, '
                        '看陀螺仪和 hector 的符号是否一致 —— 不一致就说明 IMU 是 z 轴朝下装的, '
                        '喂给 EKF 前必须取反(静止时 accel z 读 -9.8 而不是 +9.8 也是同一个证据)。')
    p.add_argument('--dur', type=float, default=8.0, help='走几秒')
    p.add_argument('--cal', type=float, default=6.0, help='起步前静止标定几秒')
    p.add_argument('--raw-gyro', dest='raw_gyro', action='store_true',
                   help='别对陀螺仪 z 取反。默认取反 —— 这颗 IMU z 轴朝下装(2026-07-26 坐实)')
    a = p.parse_args(rospy.myargv()[1:])

    out = subprocess.run(['pgrep', '-f', 'cmd_vel_to_motor'],
                         stdout=subprocess.PIPE).stdout.decode().strip()
    if not out:
        print('!! cmd_vel_to_motor.py 没在跑 —— 电机收不到指令, 车不会动, 测了也是空的。')
        print('!! 先起它: python3 ~/cmd_vel_to_motor.py')
        return 1

    rospy.init_node('drive_check', anonymous=True)
    dc = DriveCheck(flip=not a.raw_gyro)
    rospy.sleep(1.5)

    print('静止标定陀螺仪零偏 %.0fs ...' % a.cal)
    bias = dc.calib(a.cal)
    print('  bias_z = %+.5f rad/s (%+.3f deg/s)\n' % (bias, math.degrees(bias)))

    p0_map = dc.tf_pose('map', 'base_footprint')
    p0_odom = dc.tf_pose('odom', 'base_footprint')
    if p0_map is None:
        print('!! 拿不到 map->base_footprint, 导航栈起了吗?')
        return 1
    g0 = dc.gyro_yaw

    acc_z = None
    if dc.gz_samples:
        acc_z = dc.last_acc_z
    print('开始: vx=%.2f vy=%.2f wz=%.2f, %.1fs'
          % (a.speed, a.vy, a.wz, a.dur))
    m = Twist()
    m.linear.x, m.linear.y = a.speed, a.vy
    m.angular.z = a.wz
    t0 = time.time()
    r = rospy.Rate(20)
    while time.time() - t0 < a.dur and not rospy.is_shutdown():
        dc.cmd.publish(m)
        r.sleep()
    dc.cmd.publish(Twist())
    rospy.sleep(1.5)

    p1_map = dc.tf_pose('map', 'base_footprint')
    p1_odom = dc.tf_pose('odom', 'base_footprint')
    g1 = dc.gyro_yaw

    d_gyro = math.degrees(g1 - g0)
    d_map = math.degrees((p1_map[2] - p0_map[2] + math.pi) % (2 * math.pi) - math.pi)
    dx = p1_map[0] - p0_map[0]
    dy = p1_map[1] - p0_map[1]

    print('\n================ 结果 ================')
    print('(a) 物理层 —— 陀螺仪测到的净转角: %+.2f deg' % d_gyro)
    print('    命令 angular.z 全程为 0, 所以这就是车"自己歪掉"的量。')
    print('    >3 deg => 麦轮出力不均/打滑是真病因之一, 要标定电机;'
          ' <1 deg => 车走得很直, 物理层没问题。')
    print('\n(b) 估计层 —— hector 报的净转角: %+.2f deg  (与陀螺仪差 %+.2f deg)'
          % (d_map, d_map - d_gyro))
    print('    这个差值 = hector 在运动中的朝向误差。DWA 就是拿这个错的朝向去算控制量的。')
    print('    差 >2 deg => hector 运动中确实不准, EKF/先验值得上;'
          ' <0.5 deg => hector 运动中也挺准, 扭不是它的锅。')
    if p0_odom and p1_odom:
        d_odom = math.degrees((p1_odom[2] - p0_odom[2] + math.pi) % (2 * math.pi) - math.pi)
        print('\n    (参考) EKF 的 odom->base 净转角: %+.2f deg (与陀螺仪差 %+.2f deg)'
              % (d_odom, d_odom - d_gyro))
        print('    这一路本来就是陀螺仪喂出来的, 差值应该接近 0; 不接近说明 EKF 配置有问题。')
    print('\n位移(map系): dx=%+.3f dy=%+.3f  合计 %.3f m (指令 %.3f m)'
          % (dx, dy, math.hypot(dx, dy), math.hypot(a.speed, a.vy) * a.dur))

    if abs(a.wz) > 1e-6:
        print('\n================ 陀螺仪符号判定 ================')
        print('  指令 angular.z = %+.2f rad/s (ROS 约定: 正 = 逆时针 = yaw 增大)' % a.wz)
        print('  陀螺仪积分  %+.2f deg   hector %+.2f deg' % (d_gyro, d_map))
        print('  静止时 accel z = %+.2f m/s2 (ROS z轴朝上约定下应为 +9.8)' % dc.last_acc_z)
        d_raw = math.degrees(dc.gyro_yaw_raw)
        print('  (原始未修正的陀螺仪积分: %+.2f deg)' % d_raw)
        same = (d_raw > 0) == (a.wz > 0)
        if same:
            print('  ==> 陀螺仪符号与 ROS 约定**一致**, 不用取反。')
        else:
            print('  ==> ⚠️ 陀螺仪符号与 ROS 约定**相反** = IMU 是 z 轴朝下装的。')
            print('      喂 EKF 前必须把 angular_velocity.z 取反, 否则 EKF 的朝向会朝错误方向走,')
            print('      比不接 IMU 更糟。同时前面"hector 运动中误差 3.6 度"的结论要作废重算。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
