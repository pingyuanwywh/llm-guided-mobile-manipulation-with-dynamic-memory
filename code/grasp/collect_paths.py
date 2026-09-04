#!/usr/bin/env python3
"""
collect_paths.py -- 【车上】把 /move_base/make_plan 返回的**整条折线**存下来 (2026-08-08)。

## 为什么不直接用 plan_clearance.py
`plan_clearance.py` 吐的是结论("最近 0.83m, 挡路的是 can2")。结论一旦存下来, 换个阈值、
换个车体半径、换一批"哪些罐子已经收走"就得重新开车再问一遍。
**折线是原始量** —— 存了它, 上面那些问题全部离线可算, 车可以关机。

## 这份数据要回答的问题(08-01 撞车留下的)
一条直线净空 0.48m 的腿, 车把罐子撞倒了 => "直线近似"到底差多少?
把同一批腿的**直线**和**真实路径**配对存下来, 才量得出该留多少余量。

## 用法(stdin 喂 JSON, stdout 吐 JSONL, 一行一条腿)
  echo '{"legs":[{"id":"a","start":[0,0,0],"goal":[-1,0.5,1.57]}]}' | python3 ~/collect_paths.py
  出: {"id":"a","ok":true,"start":[...],"goal":[...],"path":[[x,y],...],"n":42}

不动车、不发速度指令, 只调规划服务。服务不在就每条腿吐 ok=false, 不会挂。
"""
import json
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.srv import GetPlan

SRV = "/move_base/make_plan"


def stamped(x, y, yaw, frame="map"):
    import math
    p = PoseStamped()
    p.header.frame_id = frame
    p.header.stamp = rospy.Time(0)
    p.pose.position.x = x
    p.pose.position.y = y
    p.pose.orientation.z = math.sin(yaw / 2.0)
    p.pose.orientation.w = math.cos(yaw / 2.0)
    return p


def main():
    req = json.load(sys.stdin)
    legs = req.get("legs", [])
    tol = float(req.get("tolerance", 0.10))

    rospy.init_node("collect_paths", anonymous=True, disable_signals=True)
    try:
        rospy.wait_for_service(SRV, timeout=10)
    except Exception as e:
        for L in legs:
            print(json.dumps({"id": L.get("id"), "ok": False, "error": str(e)}))
        return
    plan = rospy.ServiceProxy(SRV, GetPlan)

    for L in legs:
        sx, sy, syaw = L["start"]
        gx, gy, gyaw = L["goal"]
        rec = {"id": L.get("id"), "start": [sx, sy, syaw], "goal": [gx, gy, gyaw]}
        try:
            resp = plan(stamped(sx, sy, syaw), stamped(gx, gy, gyaw), tol)
            pts = [[round(p.pose.position.x, 4), round(p.pose.position.y, 4)]
                   for p in resp.plan.poses]
            rec.update({"ok": True, "path": pts, "n": len(pts)})
        except Exception as e:
            rec.update({"ok": False, "error": str(e)})
        print(json.dumps(rec))
        sys.stdout.flush()


if __name__ == "__main__":
    main()
