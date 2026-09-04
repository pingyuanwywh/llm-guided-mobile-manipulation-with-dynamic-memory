#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""键盘遥控: 发布 /cmd_vel。w=前 s=后 a=左转 d=右转 x=停 q=退出。
默认 linear=0.25 m/s, angular=0.10 rad/s (角速度是保护 hector 的硬上限, 别提)。
两个都能用环境变量当场覆盖, 不用改文件: LIN=0.20 python3 ~/teleop.py
在车上跑(需 board_node + cmd_vel_to_motor.py 已启动)。"""
import os, sys, termios, tty, select
import rospy
from geometry_msgs.msg import Twist

# LIN 0.08->0.16 (2026-08-01, 用户嫌慢): 07-28 实测 TEB 以 0.144 m/s 跑 + 180 度掉头后,
#   hector 与陀螺仪只差 0.17~0.25 度、距离 104~105%, 与全新地图一致 => 快跑不炸图。
#   唯一代价是单帧扫描运动畸变 0.2cm/圈 -> 1.1cm/圈(本栈无 scan de-skew), 小场地可忽略。
# LIN 0.16->0.25 (2026-08-27, 用户要建图快点): 雷达 13.6Hz 下每帧走 1.8cm, 而 hector
#   update_dist=0.1 => 每次地图更新仍有 5 帧以上扫描叠进去, 匹配约束够。
#   依据仍是 07-28 那条: 地图退化雪崩的触发条件是**反复原地转**, 不是快跑。
# ANG 保持 0.10 不动: 07-08 实测 0.20 是「真会炸图」不是「可能炸」, 当初就是为此降到 0.10 的。
#   08-27 下午的反例也记一笔: teleop 卡在 0.10 照样炸过一次图(真病因是有人开电梯门),
#   所以 0.10 不是万灵药 —— 但也没有任何理由往上提。
LIN = float(os.environ.get("LIN", 0.25))
ANG = float(os.environ.get("ANG", 0.10))
if ANG > 0.10:
    print("⚠️ ANG=%.2f 超过 0.10 硬上限(07-08 实测 0.20 真会炸图), 已压回 0.10" % ANG)
    ANG = 0.10

HELP = ("遥控就绪: w=前 s=后 a=左转 d=右转 x=停 q=退出\n"
        "(设定后持续运动,按 x 停;angular<=0.10 以保护 hector)\n"
        "当前 LIN=%.2f m/s  ANG=%.2f rad/s")


def get_key(timeout):
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        r, _, _ = select.select([sys.stdin], [], [], timeout)
        return sys.stdin.read(1) if r else ''
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def main():
    rospy.init_node("teleop_key", anonymous=True)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    print(HELP % (LIN, ANG))
    lin = ang = 0.0
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        k = get_key(0.05)
        if   k == 'w': lin, ang = LIN, 0.0
        elif k == 's': lin, ang = -LIN, 0.0
        elif k == 'a': lin, ang = 0.0, ANG
        elif k == 'd': lin, ang = 0.0, -ANG
        elif k == 'x': lin, ang = 0.0, 0.0
        elif k == 'q': break
        t = Twist(); t.linear.x = lin; t.angular.z = ang
        pub.publish(t)
        rate.sleep()
    pub.publish(Twist())
    print("\n已停止,退出遥控。")


if __name__ == "__main__":
    main()
