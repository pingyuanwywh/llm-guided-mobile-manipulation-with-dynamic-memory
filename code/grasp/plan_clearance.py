#!/usr/bin/env python3
"""
plan_clearance.py -- 【车上】拿 move_base 的**真实全局路径**去算每条腿离罐子多近 (2026-08-01)。

## 为什么要有这个
罐子只有 12cm 高, 在雷达平面以下 => costmap 里根本不存在 => 会不会撞只能自己算。
原来是拿**直线**近似车的路径 —— 08-01 实测证明这个近似不够:
一条直线净空 **0.48m** 的腿, 车实际把罐子撞倒了。因为 TEB 是全向的、走曲线、还会主动横移绕障,
07-28 测绕障时量到过横向最大偏离 **0.48m**, 和净空同一个量级。
=> 改成**问真正的规划器**: `/move_base/make_plan` 返回 Navfn 算出来的实际折线, 拿它去量。

⚠️ 仍不是完美的: make_plan 给的是**全局**路径, TEB 这个局部规划器执行时还会有自己的偏差。
   所以告警线同时从 0.30 抬到 0.50。这两件事是互补的, 不是二选一。

## 用法(stdin 喂 JSON, stdout 吐 JSON)
  echo '{"legs":[{"id":"a","start":[0,0,0],"goal":[-1,0.5,1.57],"exclude":"can1"}],
         "cans":{"can2":[-2.29,0.48]}}' | python3 ~/plan_clearance.py

  出: {"ok":true,"legs":{"a":{"min":0.83,"blocker":"can2","target_min":0.46,"pts":42}}}
     min        = 离**其他**罐子最近多少 (判 WARN 0.50)
     target_min = 离**这条腿要去接的那个**罐子最近多少 (判 SELF_MIN 0.30, 防止从它身上压过去)
     服务不在就出 {"ok":false,"error":"..."}, 调用方自己退回直线模型。
"""
import json
import math
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan


def quat_z(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def stamped(x, y, yaw, frame="map"):
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = rospy.Time(0)
    p.pose.position.x = x
    p.pose.position.y = y
    z, w = quat_z(yaw)
    p.pose.orientation.z = z
    p.pose.orientation.w = w
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


def path_pt_dist(pts, p):
    """点到折线的最近距离。只取端点会漏掉两点之间穿过去的情况, 所以逐段算。"""
    if not pts:
        return None
    if len(pts) == 1:
        return math.hypot(pts[0][0] - p[0], pts[0][1] - p[1])
    return min(seg_pt_dist(pts[i], pts[i + 1], p) for i in range(len(pts) - 1))


def main():
    req = json.load(sys.stdin)
    legs = req.get("legs", [])
    cans = req.get("cans", {})
    srv_name = req.get("service", "/move_base/make_plan")
    tol = float(req.get("tolerance", 0.10))

    rospy.init_node("plan_clearance", anonymous=True, disable_signals=True)
    try:
        rospy.wait_for_service(srv_name, timeout=float(req.get("timeout", 8.0)))
    except rospy.ROSException as e:
        print(json.dumps({"ok": False, "error": "%s 不可用: %s" % (srv_name, e)}))
        return
    make_plan = rospy.ServiceProxy(srv_name, GetPlan)

    import time as _t
    out = {}
    for leg in legs:
        sx, sy, syaw = leg["start"]
        gx, gy, gyaw = leg["goal"]
        need0 = math.hypot(gx - sx, gy - sy)
        pts, err = [], None
        for attempt in range(3):            # 密集连调会偶发垃圾, 重试两次
            _t.sleep(0.15)
            try:
                resp = make_plan(start=stamped(sx, sy, syaw),
                                 goal=stamped(gx, gy, gyaw), tolerance=tol)
                pts = [(p.pose.position.x, p.pose.position.y) for p in resp.plan.poses]
            except rospy.ServiceException as e:
                err = str(e); pts = []; continue
            if len(pts) >= 2:
                plen = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                           for i in range(len(pts) - 1))
                if plen >= 0.9 * need0 and math.hypot(pts[0][0] - sx, pts[0][1] - sy) <= 0.30:
                    break                    # 这次结果通过粗检, 收
        if err and not pts:
            out[leg["id"]] = {"error": err}
            continue

        if not pts:
            # 规划器给不出路径 —— 目标可能落在障碍里或不可达。这本身就是该拦下来的信号。
            out[leg["id"]] = {"min": None, "blocker": None, "pts": 0,
                              "note": "make_plan 返回空路径, 目标可能不可达"}
            continue

        # ⚠️⚠️ 08-01 实测: 一个请求里连发 10 条腿时, make_plan **偶尔返回垃圾**
        # (出现过"起点到终点直线 3.25m, 却返回一条 0.55m 的路径" —— 物理上不可能)。
        # 同一条腿单发 3 次全部正确 => 密集连调时不可靠。**静默的坏数据比报错危险得多**,
        # 所以下面三道校验必须有, 不过就重试, 再不过就让调用方退回直线模型。
        need = math.hypot(gx - sx, gy - sy)
        bad = None
        if len(pts) < 2:
            bad = "只有 %d 个点" % len(pts)
        elif math.hypot(pts[0][0] - sx, pts[0][1] - sy) > 0.30:
            bad = "首点(%.2f,%.2f)离传入起点太远" % pts[0]
        elif math.hypot(pts[-1][0] - gx, pts[-1][1] - gy) > 0.30 + tol:
            bad = "末点(%.2f,%.2f)离目标太远" % pts[-1]
        else:
            plen = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                       for i in range(len(pts) - 1))
            if plen < 0.9 * need:
                bad = "路径 %.2fm 短于直线 %.2fm(不可能)" % (plen, need)
        if bad:
            out[leg["id"]] = {"min": None, "blocker": None, "pts": len(pts),
                              "unreliable": bad}
            continue

        # exclude: 这条腿跑的时候**已经收走**的罐子 —— 它们真的不在场上了, 完全不算。
        # target : 这条腿要去接的那个罐子。⚠️⚠️ **它不能简单排除**:
        #   08-01 实测 —— can2 的 standoff 录在罐子靠 A 那一侧, 车却从 B 出发,
        #   于是去程直线**从罐子身上正面压过去(离罐心 0.039m)**, 而我把 can2 排除了,
        #   那条腿报的 1.08m 其实是离 can1 的距离, can2 压根没参与计算 => 罐子被辗倒。
        #   正确做法: 目标罐照样量, 只是用一条**更宽松的自身阈值**(SELF_MIN) 单独报。
        #   干净的接近: 全程离目标罐 >=0.45m(standoff 就在 0.45m 处) => 轻松过 0.30。
        #   压过去的:   最近 0.0~0.1m => 一定被拦下。
        skip = leg.get("exclude") or []
        if isinstance(skip, str):
            skip = [skip]
        skip = set(skip)
        tgt = leg.get("target")
        worst, who = float("inf"), None
        tmin = None
        for name, xy in cans.items():
            d = path_pt_dist(pts, tuple(xy))
            if d is None:
                continue
            if name == tgt:
                tmin = round(d, 3)
                continue
            if name in skip:
                continue
            if d < worst:
                worst, who = d, name
        length = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                     for i in range(len(pts) - 1))
        out[leg["id"]] = {"min": None if worst == float("inf") else round(worst, 3),
                          "blocker": who, "pts": len(pts), "len": round(length, 3),
                          "target_min": tmin}      # 离"要去接的那个罐子"最近多少

    print(json.dumps({"ok": True, "legs": out}, ensure_ascii=False))


if __name__ == "__main__":
    main()
