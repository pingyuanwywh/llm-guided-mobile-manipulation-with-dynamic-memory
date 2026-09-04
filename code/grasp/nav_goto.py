#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav_goto.py -- rotate-then-go 智能导航封装 (2026-07-25)

先把车原地单向转到大致朝向目标(破 DWA 近180度大掉头左右摆死), 再交给 move_base
直线行驶; 配合已开启的麦轮横移(linear.y)修侧偏、少拧车头 => 直、快、不卡。
若途中停滞(N秒无接近目标)自动 cancel + 重转 + 重发; 超时/ABORT 退出非0。

依赖: rotate_to.py(同目录, 单向比例控制转到 map yaw) / llm_nav_places.yaml / move_base。
用法: python3 ~/nav_goto.py --place canA [--thresh 50] [--stall 14] [--timeout 140]
退出码: 0=到达, 非0=失败(编排脚本据此判成败)。
"""
import sys, os, math, time, subprocess, argparse, yaml
import rospy, tf2_ros, actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

HOME = os.path.expanduser('~')
PLACES = HOME + '/llm_nav_places.yaml'
ROTATE = HOME + '/rotate_to.py'


def yaw_from_q(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def angdiff_deg(a, b):
    return (a - b + 180) % 360 - 180


class NavGoto:
    def __init__(self):
        self.buf = tf2_ros.Buffer()
        self.ls = tf2_ros.TransformListener(self.buf)
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo('nav_goto: waiting for move_base ...')
        self.client.wait_for_server()

    def pose(self):
        for _ in range(25):
            try:
                t = self.buf.lookup_transform('map', 'base_footprint', rospy.Time(0), rospy.Duration(1.0))
                tr = t.transform.translation
                return tr.x, tr.y, yaw_from_q(t.transform.rotation)
            except Exception:
                rospy.sleep(0.15)
        return None

    def send(self, tx, ty, tyaw):
        g = MoveBaseGoal()
        g.target_pose.header.frame_id = 'map'
        g.target_pose.header.stamp = rospy.Time.now()
        g.target_pose.pose.position.x = tx
        g.target_pose.pose.position.y = ty
        g.target_pose.pose.orientation.z = math.sin(tyaw / 2.0)
        g.target_pose.pose.orientation.w = math.cos(tyaw / 2.0)
        self.client.send_goal(g)

    def prerotate(self, tx, ty, thresh):
        p = self.pose()
        if p is None:
            return
        cx, cy, cyaw = p
        if math.hypot(tx - cx, ty - cy) < 0.15:
            return
        bearing = math.degrees(math.atan2(ty - cy, tx - cx))
        dyaw = angdiff_deg(bearing, math.degrees(cyaw))
        if abs(dyaw) > thresh:
            rospy.loginfo('nav_goto: pre-rotate to %.0f deg (dyaw %.0f)' % (bearing, dyaw))
            subprocess.call(['python3', ROTATE, '%.1f' % bearing])
        else:
            rospy.loginfo('nav_goto: skip pre-rotate (dyaw %.0f <= %.0f)' % (dyaw, thresh))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--place', required=True)
    ap.add_argument('--thresh', type=float, default=50.0, help='转角>此度数才先 rotate_to')
    ap.add_argument('--timeout', type=float, default=140.0)
    ap.add_argument('--stall', type=float, default=14.0, help='秒内无接近目标=停滞')
    ap.add_argument('--retries', type=int, default=2)
    ap.add_argument('--face', action='store_true', help='到点后再原地转到录点朝向(对准罐子/桶)')
    a = ap.parse_args()

    rospy.init_node('nav_goto', anonymous=True)
    places = yaml.safe_load(open(PLACES))['places']
    if a.place not in places:
        print('unknown place: %s' % a.place)
        sys.exit(2)
    d = places[a.place]
    tx, ty, tyaw = float(d['x']), float(d['y']), float(d.get('yaw', 0.0))

    ng = NavGoto()
    tries = 0
    ng.prerotate(tx, ty, a.thresh)
    ng.send(tx, ty, tyaw)
    t0 = time.time()
    tprog = time.time()
    best = None
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
        st = ng.client.get_state()  # 1 active, 3 succeeded, 4 aborted, 5 rejected, 9 lost
        p = ng.pose()
        dist = float('nan')
        if p is not None:
            dist = math.hypot(tx - p[0], ty - p[1])
            if best is None or dist < best - 0.03:
                best = dist
                tprog = time.time()

        if st == 3 or (p is not None and dist < 0.12):
            if a.face:
                rospy.loginfo('nav_goto: reached, face recorded yaw %.0f deg' % math.degrees(tyaw))
                subprocess.call(['python3', ROTATE, '%.1f' % math.degrees(tyaw)])
            print('SUCCEEDED place=%s pose=(%.2f,%.2f) dist=%.2f' % (a.place, p[0], p[1], dist))
            sys.exit(0)
        if st in (4, 5, 9):
            print('ABORTED place=%s move_base_state=%d' % (a.place, st))
            ng.client.cancel_goal()
            sys.exit(1)

        now = time.time()
        if now - tprog > a.stall:
            if tries < a.retries:
                tries += 1
                rospy.logwarn('nav_goto: stall %.0fs -> retry %d (cancel+re-rotate+re-send)' % (now - tprog, tries))
                ng.client.cancel_goal()
                rospy.sleep(0.3)
                ng.prerotate(tx, ty, a.thresh)
                ng.send(tx, ty, tyaw)
                tprog = time.time()
                best = None
            else:
                rospy.logerr('nav_goto: stall + retries exhausted')
                ng.client.cancel_goal()
                sys.exit(1)
        if now - t0 > a.timeout:
            rospy.logerr('nav_goto: overall timeout %.0fs' % (now - t0))
            ng.client.cancel_goal()
            sys.exit(1)
        rate.sleep()


if __name__ == '__main__':
    main()
