#!/usr/bin/env python3
"""
demo_markers_dynamic.py -- Render demo markers with cans revealed over time.

This variant is for the 2026-08-15 gateB run: the first task target is known
from the start, later cans appear when the drone reveal timeline says they were
visually discovered. If no timeline is supplied, it falls back to mission-log
write times.
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

CAN_R = 0.033
CAN_H = 0.12
WARN = 0.50


def parse_log(path):
    legs, phases, reveal = [], [], {}
    pend_block, cur, last_epoch, pending_start = {}, None, None, False
    pending_reveal = None
    first_revealed = None

    for ln in open(path):
        m = re.search(r'已写入 (can\d):', ln)
        if m:
            can = m.group(1)
            if first_revealed is None:
                first_revealed = can
                reveal[can] = 0.0
            else:
                pending_reveal = can
            continue

        m = re.search(r'⛔ (can\d) 暂时去不了\(净空 ([\d.]+), 被 (\S+?) 挡\)', ln)
        if m:
            pend_block[m.group(1)] = (float(m.group(2)), m.group(3))
            continue

        m = re.search(r'=+\s+(can\d) \(\w+\) -> \w+\s+phase=', ln)
        if m:
            cur = m.group(1)
            pending_start = True
            legs.append({'can': cur, 't_start': None, 't_grab': None})
            phases.append([None, dict(pend_block)])
            pend_block = {}
            continue

        m = re.search(r'\[INFO\] \[(\d+\.\d+)\]', ln)
        if m:
            last_epoch = float(m.group(1))
            if pending_reveal:
                reveal[pending_reveal] = last_epoch
                pending_reveal = None
            if pending_start:
                legs[-1]['t_start'] = last_epoch
                phases[-1][0] = last_epoch
                pending_start = False

        if cur and 'GRAB: success=True' in ln and legs and legs[-1]['t_grab'] is None:
            legs[-1]['t_grab'] = last_epoch

    return legs, [(t, b) for t, b in phases if t is not None], reveal


def parse_reveal_timeline(path):
    data = yaml.safe_load(open(path)) or {}
    bag_start = (data.get('sync') or {}).get('bag_start_epoch_s')
    reveals = {}
    for can, item in (data.get('reveals') or {}).items():
        if not isinstance(item, dict):
            continue
        t = item.get('ros_epoch_s')
        if t is None:
            bag_time = item.get('bag_time_s')
            if bag_time is not None and bag_start is not None:
                t = float(bag_start) + float(bag_time)
        if t is None:
            continue
        reveals[can] = float(t)
    return reveals


def circle(cx, cy, r, n=48, z=0.005):
    return [Point(cx + r * math.cos(2 * math.pi * i / n),
                  cy + r * math.sin(2 * math.pi * i / n), z) for i in range(n + 1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--state', required=True)
    ap.add_argument('--log', required=True)
    ap.add_argument('--reveal-timeline')
    ap.add_argument('--frame', default='map')
    a = ap.parse_args()

    state = yaml.safe_load(open(a.state))
    cans = {n: (c['x'], c['y'], c.get('color', 'red')) for n, c in state['cans'].items()}
    stations = {n: (s['x'], s['y']) for n, s in state['stations'].items()}
    legs, phases, reveal = parse_log(a.log)
    if a.reveal_timeline:
        timeline_reveal = parse_reveal_timeline(a.reveal_timeline)
        if timeline_reveal:
            reveal = timeline_reveal

    rospy.loginfo('legs=%d blocked_phases=%d reveal_events=%d', len(legs), len(phases), len(reveal))
    for n in sorted(reveal, key=lambda k: reveal[k]):
        rospy.loginfo('  reveal %s at %.3f', n, reveal[n])
    for leg in legs:
        rospy.loginfo('  %s start=%.3f grab=%.3f',
                      leg['can'], leg['t_start'] or 0, leg['t_grab'] or 0)

    tfbuf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tfbuf)
    pub = rospy.Publisher('/demo/markers', MarkerArray, queue_size=1, latch=True)
    rate = rospy.Rate(10)

    red = ColorRGBA(0.90, 0.15, 0.15, 1.0)
    green = ColorRGBA(0.15, 0.80, 0.25, 1.0)
    grey = ColorRGBA(0.45, 0.45, 0.45, 0.35)
    block = ColorRGBA(1.0, 0.25, 0.25, 0.95)
    clear = ColorRGBA(0.20, 0.95, 0.35, 0.85)
    target = ColorRGBA(1.0, 0.85, 0.10, 1.0)
    station_color = ColorRGBA(0.20, 0.55, 1.00, 0.85)

    while not rospy.is_shutdown():
        now = rospy.Time.now()
        t = now.to_sec()
        if t <= 0:
            rate.sleep()
            continue

        known = {n for n, rt in reveal.items() if rt == 0.0 or t >= rt}
        collected = {leg['can'] for leg in legs if leg['t_grab'] and t >= leg['t_grab']}

        blocked = {}
        for tf, phase_block in phases:
            if t >= tf:
                blocked = phase_block
        blocked = {k: v for k, v in blocked.items()
                   if k in known and k not in collected and v[1] not in collected}

        active = None
        for leg in legs:
            if leg['t_start'] and t >= leg['t_start'] and (
                    leg['t_grab'] is None or t < leg['t_grab'] + 45):
                active = leg['can']
        if active not in known or active in collected:
            active = None

        marker_array = MarkerArray()
        mid = 0

        def add(marker):
            nonlocal mid
            marker.header.frame_id = a.frame
            marker.header.stamp = now
            marker.ns = 'demo_dynamic'
            marker.id = mid
            mid += 1
            marker.action = Marker.ADD
            marker_array.markers.append(marker)

        for n, (x, y, color_name) in sorted(cans.items()):
            if n not in known:
                continue
            done = n in collected
            can_color = grey if done else (green if color_name == 'green' else red)

            body = Marker(type=Marker.CYLINDER)
            body.pose.position.x, body.pose.position.y, body.pose.position.z = x, y, CAN_H / 2
            body.pose.orientation.w = 1.0
            body.scale.x = body.scale.y = CAN_R * 2
            body.scale.z = CAN_H
            body.color = can_color
            add(body)

            disc = Marker(type=Marker.CYLINDER)
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, 0.004
            disc.pose.orientation.w = 1.0
            disc.scale.x = disc.scale.y = 0.30
            disc.scale.z = 0.008
            disc.color = ColorRGBA(can_color.r, can_color.g, can_color.b, 0.30 if done else 0.55)
            add(disc)

            label = Marker(type=Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y + 0.30, 0.34
            label.pose.orientation.w = 1.0
            label.scale.z = 0.17
            label.color = grey if done else ColorRGBA(1, 1, 1, 1)
            label.text = n + (' done' if done else '')
            add(label)

            if done:
                continue

            ring = Marker(type=Marker.LINE_STRIP)
            ring.pose.orientation.w = 1.0
            ring.scale.x = 0.035
            ring.points = circle(x, y, WARN)
            ring.color = target if n == active else (block if n in blocked else clear)
            add(ring)

            if n in blocked:
                clearance, who = blocked[n]
                block_label = Marker(type=Marker.TEXT_VIEW_FACING)
                block_label.pose.position.x = x
                block_label.pose.position.y = y - 0.32
                block_label.pose.position.z = 0.20
                block_label.pose.orientation.w = 1.0
                block_label.scale.z = 0.13
                block_label.color = block
                block_label.text = 'BLOCKED %.2fm by %s' % (clearance, who)
                add(block_label)

        try:
            tr = tfbuf.lookup_transform(a.frame, 'base_footprint', rospy.Time(0),
                                        rospy.Duration(0.2)).transform
            rx, ry = tr.translation.x, tr.translation.y

            robot = Marker(type=Marker.CYLINDER)
            robot.pose.position.x, robot.pose.position.y, robot.pose.position.z = rx, ry, 0.01
            robot.pose.orientation.w = 1.0
            robot.scale.x = robot.scale.y = 0.32
            robot.scale.z = 0.02
            robot.color = ColorRGBA(0.10, 0.75, 1.00, 0.75)
            add(robot)

            arrow = Marker(type=Marker.ARROW)
            arrow.pose.position.x, arrow.pose.position.y, arrow.pose.position.z = rx, ry, 0.06
            arrow.pose.orientation = tr.rotation
            arrow.scale.x, arrow.scale.y, arrow.scale.z = 0.34, 0.06, 0.06
            arrow.color = ColorRGBA(0.05, 0.55, 0.95, 1.0)
            add(arrow)
        except Exception:
            pass

        for n, (x, y) in sorted(stations.items()):
            station = Marker(type=Marker.CUBE)
            station.pose.position.x, station.pose.position.y, station.pose.position.z = x, y, 0.05
            station.pose.orientation.w = 1.0
            station.scale.x = station.scale.y = 0.30
            station.scale.z = 0.10
            station.color = station_color
            add(station)

            label = Marker(type=Marker.TEXT_VIEW_FACING)
            label.pose.position.x, label.pose.position.y, label.pose.position.z = x, y + 0.32, 0.30
            label.pose.orientation.w = 1.0
            label.scale.z = 0.18
            label.color = ColorRGBA(1, 1, 1, 1)
            label.text = n.replace('collect_', 'STATION ').upper()
            add(label)

        for extra in range(mid, mid + 32):
            delete = Marker()
            delete.header.frame_id = a.frame
            delete.ns = 'demo_dynamic'
            delete.id = extra
            delete.action = Marker.DELETE
            marker_array.markers.append(delete)

        pub.publish(marker_array)
        rate.sleep()


if __name__ == '__main__':
    rospy.init_node('demo_markers_dynamic', anonymous=True)
    main()
