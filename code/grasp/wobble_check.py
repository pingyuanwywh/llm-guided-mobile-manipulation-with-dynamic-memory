#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""wobble_check.py -- 量导航时车头"一扭一扭"到底有多扭 (2026-07-26)

发一个正前方 N 米的 move_base 目标, 全程录 /cmd_vel, 然后给出摆动的量化指标。
**改 DWA 参数前后各跑一次, 数字才有意义** —— 单跑一次说明不了任何问题。

核心指标 = **angular.z 每米变号多少次**。
  扭 = DWA 反复往左往右拧车头。恒定偏一侧地修正**不是扭**(那是在修车的物理偏航,
  正常且必要); 只有**反复变号**才是过冲震荡。所以变号率比"平均角速度"更能抓住扭。

其余指标:
  |wz| 均值/最大  —— 修正的幅度
  d(wz)/dt 均方根 —— 角速度指令的"抖动烈度", 直接对应 acc_lim_theta 允许的甩动
  |wz|>0.05 占比  —— 有多少时间在明显地拧车头

⚠️ 车会真的开出去 N 米。move_base 会避开地图里的障碍, 但 recovery 全禁,
   遇到未建图障碍会 ABORT 而不是绕。跑之前确认前方通畅。

用法: python3 ~/wobble_check.py [--dist 1.2] [--timeout 90] [--tag 基线]
"""
import argparse
import math
import sys
import time

import actionlib
import rospy
import tf2_ros
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


def yaw_from_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--dist', type=float, default=1.2, help='往正前方开多远(米)')
    p.add_argument('--timeout', type=float, default=90.0)
    p.add_argument('--tag', default='', help='这一跑的标签, 只用于打印')
    a = p.parse_args(rospy.myargv()[1:])

    rospy.init_node('wobble_check', anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.5)

    rec = []          # (t, vx, vy, wz)
    def on_cmd(m):
        rec.append((time.time(), m.linear.x, m.linear.y, m.angular.z))
    rospy.Subscriber('/cmd_vel', Twist, on_cmd, queue_size=50)

    def pose():
        t = buf.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_q(t.transform.rotation)

    x0, y0, yaw0 = pose()
    gx = x0 + a.dist * math.cos(yaw0)
    gy = y0 + a.dist * math.sin(yaw0)
    print('当前位姿 (%.2f, %.2f) 朝向 %.0f deg' % (x0, y0, math.degrees(yaw0)))
    print('目标     (%.2f, %.2f)  = 正前方 %.1fm' % (gx, gy, a.dist))

    cli = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    cli.wait_for_server()
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = 'map'
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = gx
    g.target_pose.pose.position.y = gy
    g.target_pose.pose.orientation.z = math.sin(yaw0 / 2.0)
    g.target_pose.pose.orientation.w = math.cos(yaw0 / 2.0)

    rec[:] = []
    t0 = time.time()
    cli.send_goal(g)
    r = rospy.Rate(4)
    state = None
    while not rospy.is_shutdown():
        state = cli.get_state()
        if state in (3, 4, 5, 9):
            break
        if time.time() - t0 > a.timeout:
            cli.cancel_goal()
            print('!! 超时 %.0fs, 取消目标' % a.timeout)
            break
        r.sleep()
    dur = time.time() - t0
    rospy.sleep(0.5)
    x1, y1, yaw1 = pose()
    travelled = math.hypot(x1 - x0, y1 - y0)

    print('\n================ %s ================' % (a.tag or '结果'))
    print('move_base 状态 %s, 用时 %.1fs, 实际走了 %.2fm' %
          ({3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED', 9: 'LOST'}.get(state, state), dur, travelled))
    if len(rec) < 10:
        print('!! /cmd_vel 只录到 %d 条, 没法分析' % len(rec))
        return 1

    wz = [r_[3] for r_ in rec]
    vx = [r_[1] for r_ in rec]
    n = len(wz)

    # 变号次数: 只统计"幅度够大"的变号, 免得把 ±0.001 的数值噪声算成扭
    DEAD = 0.02
    sig = [(1 if w > DEAD else (-1 if w < -DEAD else 0)) for w in wz]
    flips = 0
    last = 0
    for s in sig:
        if s == 0:
            continue
        if last != 0 and s != last:
            flips += 1
        last = s

    absw = [abs(w) for w in wz]
    dwz = [abs(wz[i] - wz[i - 1]) for i in range(1, n)]
    rms_d = math.sqrt(sum(d * d for d in dwz) / len(dwz))
    big = sum(1 for w in absw if w > 0.05)

    print('/cmd_vel 采样 %d 条 (%.1f Hz)' % (n, n / max(dur, 1e-6)))
    print('  ★ angular.z 变号   %d 次 = **%.1f 次/米**  (%.2f 次/秒)'
          % (flips, flips / max(travelled, 1e-6), flips / max(dur, 1e-6)))
    print('    |wz| 均值 %.4f  最大 %.4f rad/s' % (sum(absw) / n, max(absw)))
    print('    |wz|>0.05 占比 %.0f%%' % (100.0 * big / n))
    print('    相邻指令 |Δwz| 均方根 %.4f rad/s  <- acc_lim_theta 允许的甩动幅度'
          % rms_d)
    print('    前进 vx 均值 %.4f  最大 %.4f m/s' % (sum(vx) / n, max(vx)))
    print('\n  判读: 变号率越低越不扭。同一条腿改参数前后对比才有意义。')
    return 0


if __name__ == '__main__':
    sys.exit(main())
