#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""obstacle_check.py -- 绕障对照: 同一个场地同一个目标, 换规划器跑两遍 (2026-07-28)

## 为什么不能只用 wobble_check
`wobble_check.py` 是空场量"扭"用的, 只录 /cmd_vel。绕障场景还必须回答两个它答不了的问题:
  ① **到底绕过去了没有** —— 光看 SUCCEEDED 不够, 得看轨迹有没有真的横向让开
  ② **离障碍最近到多少** —— 擦着过去和从容绕开是两回事, 这个数决定敢不敢上真实任务
所以这里在 wobble_check 的全部指标之外, 多录 map->base_footprint 轨迹 + 全程 /scan 最近距离。

## 判读
- 横向最大偏离 ~= 0 而 SUCCEEDED  => 目标其实没被挡, 这次测试无效, 换个摆法
- 横向最大偏离明显(>0.3m) 且最近距离 > robot_radius(0.16) => 真绕过去了, 且有余量
- ABORTED / 超时 => 规划器过不去。DWA 在"侧面贴障"时会原地摆死(07-26 实测 90s 只挪 0.55m),
  这时看 |wz| 变号率就知道是在摆还是在等。

⚠️ 车会真的开出去。跑之前先 `python3 ~/scan_probe.py --fine --front-deg 180` 看清楚前方。

用法: python3 ~/obstacle_check.py --dist 1.3 --tag TEB
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
from sensor_msgs.msg import LaserScan


def yaw_from_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--dist', type=float, default=1.3, help='往正前方开多远(米)')
    p.add_argument('--goal-xy', dest='goal_xy', default=None,
                   help='直接指定 map 系目标 "x,y"(覆盖 --dist)。跑完对照要把车开回起点时用, '
                        '比让人搬车更可复现。')
    p.add_argument('--timeout', type=float, default=90.0)
    p.add_argument('--tag', default='', help='这一跑的标签, 只用于打印')
    a = p.parse_args(rospy.myargv()[1:])

    rospy.init_node('obstacle_check', anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    rec = []       # (t, vx, vy, wz)
    near = []      # 全程每帧扫描的最近距离

    rospy.Subscriber('/cmd_vel', Twist,
                     lambda m: rec.append((time.time(), m.linear.x, m.linear.y, m.angular.z)),
                     queue_size=50)

    def on_scan(s):
        good = [r for r in s.ranges
                if s.range_min < r < s.range_max and not math.isnan(r)]
        if good:
            near.append(min(good))
    rospy.Subscriber('/scan', LaserScan, on_scan, queue_size=5)
    rospy.sleep(1.5)

    def pose():
        t = buf.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(2.0))
        tr = t.transform.translation
        return tr.x, tr.y, yaw_from_q(t.transform.rotation)

    x0, y0, yaw0 = pose()
    if a.goal_xy:
        gx, gy = [float(v) for v in a.goal_xy.split(',')]
        # 横向偏离要沿"起点->目标"这条线量, 不是沿车头方向
        yaw0 = math.atan2(gy - y0, gx - x0)
    else:
        gx = x0 + a.dist * math.cos(yaw0)
        gy = y0 + a.dist * math.sin(yaw0)
    print('起点 (%.2f, %.2f) 朝向 %.0f deg' % (x0, y0, math.degrees(yaw0)))
    print('目标 (%.2f, %.2f) = 正前方 %.2fm' % (gx, gy, a.dist))

    cli = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    cli.wait_for_server()
    g = MoveBaseGoal()
    g.target_pose.header.frame_id = 'map'
    g.target_pose.header.stamp = rospy.Time.now()
    g.target_pose.pose.position.x = gx
    g.target_pose.pose.position.y = gy
    g.target_pose.pose.orientation.z = math.sin(yaw0 / 2.0)
    g.target_pose.pose.orientation.w = math.cos(yaw0 / 2.0)

    del rec[:], near[:]
    path = [(x0, y0)]
    t0 = time.time()
    cli.send_goal(g)
    r = rospy.Rate(10)
    state = None
    while not rospy.is_shutdown():
        state = cli.get_state()
        if state in (3, 4, 5, 9):
            break
        if time.time() - t0 > a.timeout:
            cli.cancel_goal()
            print('!! 超时 %.0fs, 取消目标' % a.timeout)
            break
        try:
            path.append(pose()[:2])
        except Exception:
            pass
        r.sleep()
    dur = time.time() - t0
    rospy.sleep(0.5)
    x1, y1, _ = pose()
    path.append((x1, y1))

    # 横向偏离 = 每个轨迹点到"起点->目标"这条直线的带符号距离(正=左)
    ux, uy = math.cos(yaw0), math.sin(yaw0)
    lat = [(-(px - x0) * uy + (py - y0) * ux) for px, py in path]
    plen = sum(math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
               for i in range(1, len(path)))
    straight = math.hypot(x1 - x0, y1 - y0)

    print('\n================ %s ================' % (a.tag or '结果'))
    print('move_base 状态 %s, 用时 %.1fs' %
          ({3: 'SUCCEEDED', 4: 'ABORTED', 5: 'REJECTED', 9: 'LOST'}.get(state, state), dur))
    print('  直线位移 %.2fm | 路径长度 %.2fm | 绕行系数 %.2f'
          % (straight, plen, plen / max(straight, 1e-6)))
    if lat:
        lo, hi = min(lat), max(lat)
        side = '左' if abs(hi) > abs(lo) else '右'
        print('  ★ 横向偏离: 最左 %+.2fm / 最右 %+.2fm  => 主要从**%s**边绕 (幅度 %.2fm)'
              % (hi, lo, side, max(abs(hi), abs(lo))))
    if near:
        # ⚠️ 这是**雷达**读数, 不是车体边缘到障碍的距离。这台车雷达装在 base_footprint
        # 前方 0.10m(见 nav_hector_bfc.launch), 正前方的障碍离车体中心 = 读数 + 0.10,
        # 侧面的约等于读数。所以**不能**直接拿它和 robot_radius=0.16 比大小 ——
        # 2026-07-28 我第一版就是这么写的, 会把"还有余量"误报成"擦上了"。
        # 正确用法: 只做**两次跑之间的相对比较**(谁留的余量大)。
        print('  ★ 全程离障碍最近 %.2fm  <-- 雷达读数, 相对比较用; 换算到车体中心要 +0~0.10m'
              % min(near))

    if len(rec) < 10:
        print('!! /cmd_vel 只录到 %d 条, 摆动指标没法算' % len(rec))
        return 1
    wz = [r_[3] for r_ in rec]
    vx = [r_[1] for r_ in rec]
    vy = [r_[2] for r_ in rec]
    n = len(wz)
    DEAD = 0.02
    sig = [(1 if w > DEAD else (-1 if w < -DEAD else 0)) for w in wz]
    flips, last = 0, 0
    for s in sig:
        if s == 0:
            continue
        if last != 0 and s != last:
            flips += 1
        last = s
    absw = [abs(w) for w in wz]
    print('  angular.z 变号 %d 次 = %.1f 次/米 (%.2f 次/秒)'
          % (flips, flips / max(plen, 1e-6), flips / max(dur, 1e-6)))
    print('  |wz| 均值 %.4f 最大 %.4f | vx 均值 %.4f | vy 均值 %.4f (麦轮横移用了没)'
          % (sum(absw) / n, max(absw), sum(vx) / n, sum(abs(v) for v in vy) / n))
    return 0


if __name__ == '__main__':
    sys.exit(main())
