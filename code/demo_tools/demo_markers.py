#!/usr/bin/env python3
"""
demo_markers.py -- 【本机, 需 source ROS + /usr/bin/python3】把「罐子/站/几何门状态」画进 rviz。

## 为什么要有它
罐子只有 12cm 高、不进 costmap ⇒ **rviz 上帝视角里那片地是空的**，观众看着车在空地上
拐来拐去毫无理由；几何门把哪个罐子判成"暂时去不了"更是只存在于文字日志里。
这个节点把任务真正的主角画出来，配合 `rosbag play` 离线重渲染，**不用重拍**。

## 数据来源全部是真日志, 不编
- 罐子/站坐标、颜色: mission_state_*.yaml
- 每条腿的开始与"抓住"时刻: mission 运行日志里的 `===== canN ... phase=` 与 `GRAB: success=True`
  前最近的 `[INFO] [epoch]`
- 几何门状态: 日志里的 `⛔ canN 暂时去不了(净空 X, 被 Y 挡)` —— 按"规划时刻"分段生效

## 用法
  roscore &
  rosparam set /use_sim_time true
  /usr/bin/python3 ~/demo_markers.py --state ~/mission_state_0808c.yaml --log <运行日志>
  rosbag play --clock <bag>
"""
import argparse
import math
import re

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

CAN_R = 0.033        # 罐子半径 (Ø66mm)
CAN_H = 0.12         # 罐高
WARN = 0.50          # 告警线, 画成罐子周围那个圈


def parse_log(path):
    """返回 (legs, blocked_phases)。
    legs = [{'can','t_start','t_grab'}]  按时间序
    blocked_phases = [(t_from, {can: (净空, 挡路者)})]  规划时刻生效, 直到下一个规划时刻
    """
    legs, phases = [], []
    pend_block, cur, last_epoch, pending_start = {}, None, None, False
    for ln in open(path):
        m = re.search(r'⛔ (can\d) 暂时去不了\(净空 ([\d.]+), 被 (\S+?) 挡\)', ln)
        if m:
            pend_block[m.group(1)] = (float(m.group(2)), m.group(3))
            continue
        m = re.search(r'=+\s+(can\d) \(\w+\) -> \w+\s+phase=', ln)
        if m:
            cur = m.group(1)
            pending_start = True
            legs.append({'can': cur, 't_start': None, 't_grab': None})
            # 这一轮规划算出来的 blocked, 从这条腿开始生效
            phases.append([None, dict(pend_block)])
            pend_block = {}
            continue
        m = re.search(r'\[INFO\] \[(\d+\.\d+)\]', ln)
        if m:
            last_epoch = float(m.group(1))
            if pending_start:
                legs[-1]['t_start'] = last_epoch
                phases[-1][0] = last_epoch
                pending_start = False
        if cur and 'GRAB: success=True' in ln and legs and legs[-1]['t_grab'] is None:
            legs[-1]['t_grab'] = last_epoch
    return legs, [(t, b) for t, b in phases if t is not None]


def circle(cx, cy, r, n=48, z=0.005):
    return [Point(cx + r * math.cos(2 * math.pi * i / n),
                  cy + r * math.sin(2 * math.pi * i / n), z) for i in range(n + 1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--state', required=True)
    ap.add_argument('--log', required=True)
    ap.add_argument('--frame', default='map')
    a = ap.parse_args()

    S = yaml.safe_load(open(a.state))
    cans = {n: (c['x'], c['y'], c.get('color', 'red')) for n, c in S['cans'].items()}
    stations = {n: (s['x'], s['y']) for n, s in S['stations'].items()}
    legs, phases = parse_log(a.log)
    rospy.loginfo('腿 %d 条, 几何门分段 %d 个', len(legs), len(phases))
    for L in legs:
        rospy.loginfo('  %s  start=%.0f grab=%.0f', L['can'], L['t_start'] or 0, L['t_grab'] or 0)

    tfbuf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfbuf)
    pub = rospy.Publisher('/demo/markers', MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(10)

    RED = ColorRGBA(0.90, 0.15, 0.15, 1.0)
    GREEN = ColorRGBA(0.15, 0.80, 0.25, 1.0)
    GREY = ColorRGBA(0.45, 0.45, 0.45, 0.35)
    BLOCK = ColorRGBA(1.0, 0.25, 0.25, 0.95)     # 被挡: 红圈
    CLEAR = ColorRGBA(0.20, 0.95, 0.35, 0.85)    # 可去: 绿圈
    TARGET = ColorRGBA(1.0, 0.85, 0.10, 1.0)     # 当前目标: 黄
    STCOL = ColorRGBA(0.20, 0.55, 1.00, 0.85)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = now.to_sec()
        if t <= 0:
            rate.sleep()
            continue

        blocked = {}
        for tf, b in phases:
            if t >= tf:
                blocked = b
        active = None
        for L in legs:
            if L['t_start'] and t >= L['t_start'] and (L['t_grab'] is None or t < L['t_grab'] + 45):
                active = L['can']
        collected = {L['can'] for L in legs if L['t_grab'] and t >= L['t_grab']}
        # 挡路者一旦进了夹爪, 那条阻塞就没了 —— 这是推断, 但**下一次规划的日志正好确认**
        # (leg2 前 can1/can2 被 can3 挡; can3 收走后 leg3 起日志里再没有 ⛔)。
        # 不这样做的话画面会出现"被 can3 挡"而 can3 已经变灰的自相矛盾。
        blocked = {k: v for k, v in blocked.items() if v[1] not in collected}

        ma = MarkerArray()
        mid = 0

        def add(m):
            nonlocal mid
            m.header.frame_id = a.frame
            m.header.stamp = now
            m.ns = 'demo'
            m.id = mid
            mid += 1
            m.action = Marker.ADD
            ma.markers.append(m)

        for n, (x, y, col) in sorted(cans.items()):
            done = n in collected
            # 罐身
            m = Marker(type=Marker.CYLINDER)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, CAN_H / 2
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = CAN_R * 2
            m.scale.z = CAN_H
            m.color = GREY if done else (GREEN if col == 'green' else RED)
            add(m)
            # 底盘色块: 真罐子只有 6.6cm, 在上帝视角里 ~10px 看不见; 这个盘只是**标识**, 不代表尺寸
            disc = Marker(type=Marker.CYLINDER)
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.004
            disc.pose.orientation.w = 1.0
            disc.scale.x = disc.scale.y = 0.30
            disc.scale.z = 0.008
            c0 = GREY if done else (GREEN if col == 'green' else RED)
            disc.color = ColorRGBA(c0.r, c0.g, c0.b, 0.30 if done else 0.55)
            add(disc)
            # 名字(挪到罐子上方, 别压住本体)
            tm = Marker(type=Marker.TEXT_VIEW_FACING)
            tm.pose.position.x, tm.pose.position.y, tm.pose.position.z = x, y + 0.30, 0.34
            tm.pose.orientation.w = 1.0
            tm.scale.z = 0.17
            tm.color = GREY if done else ColorRGBA(1, 1, 1, 1)
            tm.text = n + (' ✓' if done else '')
            add(tm)
            if done:
                continue
            # 几何门状态圈: 被挡=红, 可去=绿, 当前目标=黄
            ring = Marker(type=Marker.LINE_STRIP)
            ring.pose.orientation.w = 1.0
            ring.scale.x = 0.035
            ring.points = circle(x, y, WARN)
            ring.color = TARGET if n == active else (BLOCK if n in blocked else CLEAR)
            add(ring)
            if n in blocked:
                cl, who = blocked[n]
                bt = Marker(type=Marker.TEXT_VIEW_FACING)
                bt.pose.position.x, bt.pose.position.y, bt.pose.position.z = x, y - 0.32, 0.20
                bt.pose.orientation.w = 1.0
                bt.scale.z = 0.13
                bt.color = BLOCK
                bt.text = 'BLOCKED %.2fm by %s' % (cl, who)
                add(bt)

        try:
            tr = tfbuf.lookup_transform(a.frame, 'base_footprint', rospy.Time(0),
                                        rospy.Duration(0.2)).transform
            rx, ry = tr.translation.x, tr.translation.y
            rb = Marker(type=Marker.CYLINDER)
            rb.pose.position.x, rb.pose.position.y, rb.pose.position.z = rx, ry, 0.01
            rb.pose.orientation.w = 1.0
            rb.scale.x = rb.scale.y = 0.32          # 车体直径(真值 2*0.16)
            rb.scale.z = 0.02
            rb.color = ColorRGBA(0.10, 0.75, 1.00, 0.75)
            add(rb)
            ar = Marker(type=Marker.ARROW)
            ar.pose.position.x, ar.pose.position.y, ar.pose.position.z = rx, ry, 0.06
            ar.pose.orientation = tr.rotation
            ar.scale.x, ar.scale.y, ar.scale.z = 0.34, 0.06, 0.06
            ar.color = ColorRGBA(0.05, 0.55, 0.95, 1.0)
            add(ar)
        except Exception:
            pass

        for n, (x, y) in sorted(stations.items()):
            m = Marker(type=Marker.CUBE)
            m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.30
            m.scale.z = 0.10
            m.color = STCOL
            add(m)
            tm = Marker(type=Marker.TEXT_VIEW_FACING)
            tm.pose.position.x, tm.pose.position.y, tm.pose.position.z = x, y + 0.32, 0.30
            tm.pose.orientation.w = 1.0
            tm.scale.z = 0.18
            tm.color = ColorRGBA(1, 1, 1, 1)
            tm.text = n.replace('collect_', 'STATION ').upper()
            add(tm)

        # 清掉上一帧多出来的 id
        for extra in range(mid, mid + 12):
            d = Marker()
            d.header.frame_id = a.frame
            d.ns = 'demo'
            d.id = extra
            d.action = Marker.DELETE
            ma.markers.append(d)

        pub.publish(ma)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('demo_markers', anonymous=True)
    main()
