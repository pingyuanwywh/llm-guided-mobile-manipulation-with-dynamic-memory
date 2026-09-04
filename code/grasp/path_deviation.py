#!/usr/bin/env python3
"""
path_deviation.py -- 【车上】量 TEB 实际开出来的轨迹, 偏离 move_base 全局路径多少 (2026-08-01)。

## 为什么要量这个
08-01 撞倒 can2 之后我断定"直线近似不了实际路径", 但实测**全局路径和直线只差 5~16cm**,
解释不了那次撞车(直线净空 0.48m ⇒ 全局路径也有 0.32~0.46m, 远高于必撞线 0.193)。
⇒ **剩下最大的嫌疑是 TEB(局部规划器)相对全局路径的偏离, 这一段完全没测过。**
这个数直接决定几何门的告警线该设多少 —— 现在的 0.50 是拍脑袋拍出来的。

## 做什么
① 先问 make_plan 要"车当前位置 → 目标点"的全局路径
② 跑 nav_goto 过去, 全程 10Hz 采 map->base_footprint 的真实位姿
③ 报: 实际轨迹偏离全局路径的 最大/均值/RMS; 实际轨迹离每个罐子最近多少; 与全局路径的对比

用法: python3 ~/path_deviation.py --place can1 [--cans-from ~/llm_nav_places.yaml] [--tag t1]
      不动机械臂, 不抓东西, 就是开过去。
"""
import argparse
import json
import math
import os
import subprocess
import threading
import time

import rospy
import tf2_ros
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan

CAN_AHEAD = 0.45


def stamped(x, y, yaw):
    p = PoseStamped()
    p.header.frame_id = "map"
    p.header.stamp = rospy.Time(0)
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def seg_pt_dist(a, b, p):
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def poly_dist(poly, p):
    if len(poly) < 2:
        return math.hypot(poly[0][0] - p[0], poly[0][1] - p[1]) if poly else None
    return min(seg_pt_dist(poly[i], poly[i + 1], p) for i in range(len(poly) - 1))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--place", required=True)
    ap.add_argument("--places-file", default=os.path.expanduser("~/llm_nav_places.yaml"))
    ap.add_argument("--tag", default="dev")
    a = ap.parse_args()

    places = yaml.safe_load(open(a.places_file))["places"]
    tgt = places[a.place]
    cans = {n: (v["x"] + CAN_AHEAD * math.cos(v["yaw"]),
                v["y"] + CAN_AHEAD * math.sin(v["yaw"]))
            for n, v in places.items() if n.startswith("can")}

    rospy.init_node("path_deviation", anonymous=True, disable_signals=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    time.sleep(2.0)

    def pose():
        t = buf.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(2.0))
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z))

    sx, sy, syaw = pose()
    print("起点 (%+.3f, %+.3f, %+.0f deg) -> %s (%+.3f, %+.3f, %+.0f deg)"
          % (sx, sy, math.degrees(syaw), a.place, tgt["x"], tgt["y"], math.degrees(tgt["yaw"])))

    rospy.wait_for_service("/move_base/make_plan", timeout=10)
    mp = rospy.ServiceProxy("/move_base/make_plan", GetPlan)
    resp = mp(start=stamped(sx, sy, syaw),
              goal=stamped(tgt["x"], tgt["y"], tgt["yaw"]), tolerance=0.10)
    gp = [(p.pose.position.x, p.pose.position.y) for p in resp.plan.poses]
    if not gp:
        print("!! make_plan 给不出路径, 停")
        return
    glen = sum(math.hypot(gp[i + 1][0] - gp[i][0], gp[i + 1][1] - gp[i][1])
               for i in range(len(gp) - 1))
    print("全局路径 %d 点 / %.2f m; 直线 %.2f m"
          % (len(gp), glen, math.hypot(tgt["x"] - sx, tgt["y"] - sy)))

    # 采样线程
    traj, stop = [], threading.Event()

    def sampler():
        r = rospy.Rate(10)
        while not stop.is_set():
            try:
                x, y, _ = pose()
                traj.append((x, y))
            except Exception:
                pass
            r.sleep()

    th = threading.Thread(target=sampler)
    th.daemon = True
    th.start()

    env = dict(os.environ)
    t0 = time.time()
    rc = subprocess.call(["python3", "-u", os.path.expanduser("~/nav_goto.py"),
                          "--place", a.place, "--face", "--thresh", "181"], env=env)
    dt = time.time() - t0
    stop.set()
    th.join(timeout=3)
    print("nav_goto 退出码 %d, 用时 %.1fs, 采到 %d 个位姿" % (rc, dt, len(traj)))
    if len(traj) < 5:
        print("!! 采样太少, 结论不可信")
        return

    devs = [poly_dist(gp, p) for p in traj]
    devs = [d for d in devs if d is not None]
    sl = [(sx, sy), (tgt["x"], tgt["y"])]
    devs_line = [seg_pt_dist(sl[0], sl[1], p) for p in traj]

    out = {
        "tag": a.tag, "place": a.place, "rc": rc, "secs": round(dt, 1),
        "偏离全局路径": {"max": round(max(devs), 3),
                        "mean": round(sum(devs) / len(devs), 3),
                        "rms": round(math.sqrt(sum(d * d for d in devs) / len(devs)), 3)},
        "偏离直线": {"max": round(max(devs_line), 3),
                     "mean": round(sum(devs_line) / len(devs_line), 3)},
        "实际轨迹离各罐子最近": {},
        "全局路径离各罐子最近": {},
    }
    for n, xy in cans.items():
        out["实际轨迹离各罐子最近"][n] = round(poly_dist(traj, xy), 3)
        out["全局路径离各罐子最近"][n] = round(poly_dist(gp, xy), 3)

    print(json.dumps(out, ensure_ascii=False, indent=1))
    p = os.path.expanduser("~/path_dev_%s.json" % a.tag)
    json.dump({"summary": out, "global": gp, "actual": traj}, open(p, "w"))
    print("轨迹已存 %s" % p)


if __name__ == "__main__":
    main()
