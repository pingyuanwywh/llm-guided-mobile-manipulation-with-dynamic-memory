#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""odom_drytest.py -- 干测 cmd_odom -> EKF 的位移通路 (2026-07-26)

**车不会动**: 前提是 cmd_vel_to_motor.py **没有在跑**。那样 /cmd_vel 只会被 cmd_odom.py
收到, 没有任何东西会写 /board/set_motor, 电机拿不到指令。跑之前脚本自己会检查一遍。

测什么: 发一段已知的速度指令, 看 EKF 推算出来的 odom->base_footprint 位移对不对得上。
  对得上 => cmd_odom 的协方差/坐标系/watchdog 和 EKF 的 odom0_config 全都接对了。
  对不上 => 十有八九是 odom0_config 那 15 个开关勾错位, 或者 frame 名字对不上。

用法: python3 ~/odom_drytest.py    (先确认 cmd_vel_to_motor 没跑!)
"""
import subprocess
import sys
import time

import rospy
import tf2_ros
from geometry_msgs.msg import Twist

SPEED = 0.10
DUR = 3.0


def main():
    out = subprocess.run(['pgrep', '-f', 'cmd_vel_to_motor'],
                         stdout=subprocess.PIPE).stdout.decode().strip()
    if out:
        print('!! cmd_vel_to_motor.py 正在跑 (pid %s) —— 这时候发 cmd_vel 车会真的动!' % out)
        print('!! 干测请先把它停掉。中止。')
        return 1

    rospy.init_node('odom_drytest', anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.5)
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    rospy.sleep(0.5)

    def pos():
        t = buf.lookup_transform('odom', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
        tr = t.transform.translation
        return tr.x, tr.y

    def run(vx, vy, label):
        x0, y0 = pos()
        m = Twist()
        m.linear.x, m.linear.y = vx, vy
        t0 = time.time()
        r = rospy.Rate(20)
        while time.time() - t0 < DUR and not rospy.is_shutdown():
            pub.publish(m)
            r.sleep()
        pub.publish(Twist())
        rospy.sleep(1.2)          # 等 watchdog 归零 + EKF 收敛
        x1, y1 = pos()
        want = (vx * DUR, vy * DUR)
        got = (x1 - x0, y1 - y0)
        err = max(abs(got[0] - want[0]), abs(got[1] - want[1]))
        print('%-10s 指令 vx=%+.2f vy=%+.2f x %.1fs' % (label, vx, vy, DUR))
        print('           期望位移 dx=%+.3f dy=%+.3f' % want)
        print('           EKF 实测 dx=%+.3f dy=%+.3f   误差 %.3f m  %s'
              % (got[0], got[1], err, 'OK' if err < 0.06 else '<== 对不上!'))
        return err < 0.06

    print('cmd_vel_to_motor 没在跑, 电机通路断开, 车不会动。开测。\n')
    ok1 = run(SPEED, 0.0, '前进')
    print()
    ok2 = run(0.0, SPEED, '左横移')
    print('\n结论: %s' % ('位移通路接对了' if (ok1 and ok2) else
                          '有问题 —— 查 ekf.yaml 的 odom0_config 开关位置 / frame 名字'))
    return 0 if (ok1 and ok2) else 1


if __name__ == '__main__':
    sys.exit(main())
