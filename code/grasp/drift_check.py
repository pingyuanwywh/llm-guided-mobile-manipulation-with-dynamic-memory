#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drift_check.py -- 测「原地旋转到底会不会让 hector 的位置估计滑移」(2026-07-28)

## 为什么要有这个脚本
07-23 测到纯旋转指令下 hector 位置估计滑了 0.42m, 07-24 一次 147 度掉头滑了 0.19m
=> 记忆里挂着一条"每转一次漂 0.2m"。**但那两次都是在已经跑过一阵、地图可能被涂花之后测的。**
07-26 在**全新地图**上测过旋转的**朝向**误差 = 0.01~0.32 度(准得惊人), 但**位置滑移从来没测过**。
朝向准 != 位置不滑, 这是两个量, 之前混着谈了。

## 做法
连续做 N 次相同角度的原地旋转(**用陀螺仪闭环**, 保证每次真实转角一样), 每次之后记 map->base_footprint。
默认 5x72 度 = 转满一整圈 => **车物理上回到出发姿态**, 此时 map 里若有净位移, 那就是纯估计漂移
(原地自转本身不该让车心移动)。同时看滑移是否**逐次放大** —— 放大 = 地图退化正反馈,
不放大 = 每次转固定代价。这两种情况解法完全不同。

⚠️ 车会真的原地转。转之前确认车周围没人没线。全程 linear.x/y = 0, 车心不动, 只需要转身的空间。
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


class DriftCheck:
    def __init__(self, flip=True):
        self.flip = flip          # 这颗 IMU z 轴朝下装(07-26 坐实), 原始 gz 要取反才符合 ROS 约定
        self.gyro = 0.0           # 陀螺仪积分的累计转角(rad)
        self.bias = 0.0
        self.samples = []
        self.last_t = None
        self.last_wall = 0.0      # 最后一条 IMU 消息的墙上时间, 给旋转看门狗用
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
        self.last_wall = time.time()
        self.samples.append(gz)

    def on_scan(self, msg):
        self.scan = msg

    def calib(self, secs):
        """静止求陀螺仪零偏。零偏会温漂(15 分钟变 15%), 必须每次现测, 不能硬编码。"""
        self.samples = []
        rospy.sleep(secs)
        if len(self.samples) > 10:
            self.bias = sum(self.samples) / len(self.samples)
        self.last_t = None
        self.gyro = 0.0
        return self.bias

    def pose(self, parent='map', child='base_footprint'):
        try:
            tr = self.buf.lookup_transform(parent, child, rospy.Time(0), rospy.Duration(0.5))
            t = tr.transform.translation
            return t.x, t.y, yaw_from_q(tr.transform.rotation)
        except Exception:
            return None

    def clearance(self):
        """车周围最近障碍 —— 原地转只需要转身空间, 但太贴墙也会蹭。"""
        s = self.scan
        if s is None:
            return None
        good = [r for r in s.ranges if s.range_min < r < s.range_max and not math.isnan(r)]
        return min(good) if good else None

    def rotate(self, target_rad, wz):
        """陀螺仪闭环原地转 target_rad。近终点减速, 减少过冲, 让每次真实转角尽量一致。

        ⚠️ 这是个**靠传感器决定何时停**的循环 => IMU 一死就永远转不到目标。
        2026-07-28 实测踩到: 板子串口中途挂掉, /board/imu_raw 断流, 车按旧的宽超时
        空转了几十秒, hector 彻底跑飞。所以下面两道闸都必须有:
          (1) IMU 断流看门狗 —— 0.7 秒没新消息立刻停车退出, 别等超时;
          (2) 超时按"理论用时 x1.6 + 4s"算, 不是拍脑袋的大数。
        """
        sign = 1.0 if target_rad > 0 else -1.0
        start = self.gyro
        m = Twist()
        r = rospy.Rate(20)
        t0 = time.time()
        limit = abs(target_rad) / max(wz, 0.05) * 1.6 + 4.0
        while not rospy.is_shutdown():
            done = abs(self.gyro - start)
            if done >= abs(target_rad):
                break
            if time.time() - self.last_wall > 0.7:
                self.cmd.publish(Twist())
                print('  !! IMU 断流 (>0.7s 没消息) —— 已停车。板子串口大概率挂了, '
                      '复位 USB 再来: 见 project_jetrover_board_comm 记忆。')
                raise RuntimeError('imu stalled')
            if time.time() - t0 > limit:
                print('  !! 旋转超时 %.0fs, 停止(只转到 %.1f deg)' % (limit, math.degrees(done)))
                break
            frac = done / abs(target_rad)
            m.angular.z = sign * (wz if frac < 0.75 else max(0.12, wz * 0.45))
            self.cmd.publish(m)
            r.sleep()
        self.cmd.publish(Twist())
        return self.gyro - start


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--steps', type=int, default=5, help='转几次(默认 5 x 72 = 360 度回到原点)')
    p.add_argument('--angle', type=float, default=72.0, help='每次转多少度')
    p.add_argument('--wz', type=float, default=0.4, help='转速 rad/s')
    p.add_argument('--settle', type=float, default=2.5, help='每次转完静置几秒再读 TF')
    p.add_argument('--cal', type=float, default=6.0, help='开跑前静止标定零偏几秒')
    p.add_argument('--tag', default='', help='给这一轮起个名字, 打印在标题里')
    a = p.parse_args(rospy.myargv()[1:])

    if not subprocess.run(['pgrep', '-f', 'cmd_vel_to_motor'],
                          stdout=subprocess.PIPE).stdout.strip():
        print('!! cmd_vel_to_motor.py 没在跑 => 电机收不到指令, 车不会动, 测了是空的。')
        return 1

    rospy.init_node('drift_check', anonymous=True)
    d = DriftCheck()
    rospy.sleep(1.5)

    if d.pose() is None:
        print('!! 拿不到 map->base_footprint, hector 起了吗?')
        return 1
    cl = d.clearance()
    print('周围最近障碍 %.2f m' % cl if cl else '拿不到 /scan')
    if cl is not None and cl < 0.30:
        print('!! 太贴障碍(<0.30m), 原地转可能蹭到。挪开点再来。')
        return 1

    print('静止标定陀螺仪零偏 %.0fs ...' % a.cal)
    b = d.calib(a.cal)
    print('  bias = %+.5f rad/s (%+.4f deg/s)' % (b, math.degrees(b)))

    p0 = d.pose()
    g0 = d.gyro
    print('\n起点 map: x=%+.3f y=%+.3f yaw=%+.2f deg' % (p0[0], p0[1], math.degrees(p0[2])))
    print('每步转 %.0f deg x %d 次 = %.0f deg%s\n'
          % (a.angle, a.steps, a.angle * a.steps,
             ' (转满一圈, 车物理上回到出发姿态)' if abs(a.angle * a.steps - 360) < 1 else ''))
    hdr = ('步 | 真实转角(陀螺) | hector转角 | 朝向误差 | 本步位移 dx     dy    |d| | '
           '累计位移 |cum| | 累计朝向误差')
    print(hdr)
    print('-' * len(hdr))

    prev = p0
    rows = []
    for i in range(1, a.steps + 1):
        try:
            true_step = d.rotate(math.radians(a.angle), a.wz)
        except RuntimeError:
            d.cmd.publish(Twist())
            print('!! 传感器断流, 本轮作废(已跑的 %d 步数据在上面, 仍可用)' % (i - 1))
            return 2
        rospy.sleep(a.settle)
        cur = d.pose()
        if cur is None:
            print('!! TF 丢了')
            return 1
        dyaw_h = wrap(cur[2] - prev[2])
        dx, dy = cur[0] - prev[0], cur[1] - prev[1]
        cdx, cdy = cur[0] - p0[0], cur[1] - p0[1]
        cum_true = d.gyro - g0
        cum_h = wrap(cur[2] - p0[2])
        # 累计朝向误差要按"转过的总角度"比, 不能用 wrap 后的值直接减(转满一圈会绕回去)
        cum_err = math.degrees(wrap(cum_h - cum_true)) if abs(math.degrees(cum_true)) < 350 \
            else math.degrees(wrap(cum_h - cum_true))
        rows.append((i, math.degrees(true_step), math.degrees(dyaw_h),
                     dx, dy, math.hypot(dx, dy), math.hypot(cdx, cdy), cum_err))
        print('%2d | %+13.2f | %+10.2f | %+8.2f | %+6.3f %+6.3f %5.3f | %11.3f | %+12.2f'
              % (i, math.degrees(true_step), math.degrees(dyaw_h),
                 math.degrees(wrap(dyaw_h - true_step)),
                 dx, dy, math.hypot(dx, dy), math.hypot(cdx, cdy), cum_err))
        prev = cur

    p1 = d.pose()
    print('\n================ 结论用的数 ================')
    print('终点 map: x=%+.3f y=%+.3f yaw=%+.2f deg' % (p1[0], p1[1], math.degrees(p1[2])))
    print('陀螺仪累计真实转角: %+.2f deg' % math.degrees(d.gyro - g0))
    print('map 里的净位移: %.3f m  (dx=%+.3f dy=%+.3f)'
          % (math.hypot(p1[0] - p0[0], p1[1] - p0[1]), p1[0] - p0[0], p1[1] - p0[1]))
    steps_d = [r[5] for r in rows]
    print('每步位移: ' + ' '.join('%.3f' % s for s in steps_d))
    if len(steps_d) >= 4:
        h1 = sum(steps_d[:len(steps_d) // 2]) / (len(steps_d) // 2)
        h2 = sum(steps_d[len(steps_d) // 2:]) / (len(steps_d) - len(steps_d) // 2)
        print('前半程每步均值 %.3f m -> 后半程 %.3f m  (%s)'
              % (h1, h2, '逐次放大 = 地图退化正反馈' if h2 > h1 * 1.6
                 else '没有明显放大 = 每次转是固定代价'))
    print('\n判读: 原地自转车心不该移动 => 上面的位移基本就是估计滑移。')
    print('      抓取的容错预算约 +-0.10m, 所以单次 >0.05m 就值得治, 累计 >0.10m 必然毁任务。')
    if abs(a.angle * a.steps - 360) < 1:
        print('      转满一圈: 请肉眼确认车是不是转回了出发朝向 —— 若是, 上面的净位移全是漂移。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
