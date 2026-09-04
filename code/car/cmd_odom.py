#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""cmd_odom.py -- 把"指令速度"当成一路里程计喂给 EKF (2026-07-26)

**为什么需要它**: EKF 要输出车的位姿, 就得知道车走了多远。但车没有轮式编码器,
而 IMU 的加速度计双积分算位移是出了名的垃圾(误差被积分两次, 几秒漂几米), 用不了。

**退而求其次**: 我们知道自己命令车走多快。cmd_vel_to_motor.py 把 /cmd_vel 换算成四轮
转速, 系数 LIN_MS_PER_RPS=0.286 是标定过的, 逼近环实测执行率 83~108%。所以"指令速度"
是一个有偏但可用的速度观测 —— 没有编码器时这是标准做法, 叫"命令速度航位推算"。

**只发速度, 不发位置**: 位置由 EKF 自己积分。也**不发偏航角速度** —— 那一维交给陀螺仪,
比指令值精确得多(指令 0.25 rad/s 时实际可能是 0.2, 陀螺仪却是直接测出来的)。

**打滑/堵转时它会偏**, 所以协方差给得大; 长期误差由 hector 在 map->odom 那一层兜底修正,
这正是 map->odom / odom->base 两段式分工的意义。

⚠️ **必须和 cmd_vel_to_motor 的 watchdog 行为一致**: 那边 CMD_TIMEOUT=0.5s 收不到
   /cmd_vel 就把电机停掉。这边也必须 0.5s 后当作速度归零 —— 否则一条命令发完之后,
   EKF 会拿一个早就失效的速度一直往前积分, 车明明停着, 估计位置却一路飞出去。

用法: python3 ~/cmd_odom.py [--scale 1.0] [--rate 30]
"""
import argparse

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CMD_TIMEOUT = 0.5      # 和 cmd_vel_to_motor.py:29 保持一致, 改那边这边也要改


class CmdOdom:
    def __init__(self, args):
        self.args = args
        self.vx = 0.0
        self.vy = 0.0
        self.last = rospy.Time(0)
        self.pub = rospy.Publisher(args.out, Odometry, queue_size=10)
        rospy.Subscriber(args.cmd, Twist, self.on_cmd, queue_size=1)

    def on_cmd(self, msg):
        # scale: 指令->实际 的执行率。实测 83~108%, 默认 1.0 不修正 ——
        # 反正 hector 会在 map->odom 层把系统性偏差吸收掉, 这里改小反而可能过度补偿。
        self.vx = msg.linear.x * self.args.scale
        self.vy = msg.linear.y * self.args.scale
        self.last = rospy.Time.now()

    def tick(self, _evt):
        now = rospy.Time.now()
        stale = (now - self.last).to_sec() > CMD_TIMEOUT
        vx = 0.0 if stale else self.vx
        vy = 0.0 if stale else self.vy

        o = Odometry()
        o.header.stamp = now
        o.header.frame_id = self.args.odom_frame
        o.child_frame_id = self.args.base_frame
        o.twist.twist.linear.x = vx
        o.twist.twist.linear.y = vy
        o.twist.twist.angular.z = 0.0

        # 协方差 = "这个数我有多不信"。EKF 配置里只启用了 vx/vy 两项(见 ekf.yaml 的
        # odom0_config), 其余项填大数只是防呆。
        # var=0.02 => std≈0.14 m/s, 相对 0.08~0.15 m/s 的行驶速度是很松的 ——
        # 故意的: 指令速度会因打滑/负载/电量而偏, 宁可让 EKF 少信它一点。
        big = 1e6
        c = [big] * 36
        c[0] = self.args.var        # vx
        c[7] = self.args.var        # vy
        c[14] = big                 # vz
        c[21] = big                 # vroll
        c[28] = big                 # vpitch
        c[35] = big                 # vyaw —— 明确不参与, 偏航全听陀螺仪的
        o.twist.covariance = c
        o.pose.covariance = [big if i % 7 == 0 else 0.0 for i in range(36)]
        self.pub.publish(o)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cmd', default='/cmd_vel')
    p.add_argument('--out', default='/odom_cmd')
    p.add_argument('--odom-frame', dest='odom_frame', default='odom')
    p.add_argument('--base-frame', dest='base_frame', default='base_footprint')
    p.add_argument('--scale', type=float, default=1.0, help='指令->实际 执行率修正')
    p.add_argument('--var', type=float, default=0.02, help='速度观测方差 (m/s)^2')
    p.add_argument('--rate', type=float, default=30.0)
    args = p.parse_args(rospy.myargv()[1:])

    rospy.init_node('cmd_odom')
    rospy.loginfo('cmd_odom: %s -> %s @ %.0fHz (scale=%.2f, watchdog=%.1fs)',
                  args.cmd, args.out, args.rate, args.scale, CMD_TIMEOUT)
    co = CmdOdom(args)
    rospy.Timer(rospy.Duration(1.0 / args.rate), co.tick)
    rospy.spin()


if __name__ == '__main__':
    main()
