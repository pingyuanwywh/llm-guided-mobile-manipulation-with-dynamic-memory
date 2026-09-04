#!/usr/bin/env python3
"""
clearance_scan.py -- 跑前全布局净空扫描 (2026-08-01, 从今天撞倒 can2 那次教训来的)。

罐子只有 12cm 高 => 在雷达平面以下 => **costmap 里根本不存在** => move_base 会规划一条
直接穿过它的路。所以每条腿会不会擦到别的罐子, 只能自己算。

跑前把**全部罐子都已知**的完整布局扫一遍, 列出所有紧的走廊。有两个用途:
  ① 全信息模式(所有坐标一次给全): 这些腿会被几何门挡掉, 看一眼有没有把某个罐子彻底封死;
  ② 流式模式(无人机陆续报): 这张表就是**"如果这条腿在对应罐子被报出来之前跑, 就会撞"**的预警清单
     —— 08-01 就是这么撞的: can5 那条回程腿在 +90s 规划, can2 +180s 才报出来。

用法: python3 ~/clearance_scan.py --state ~/mission_state_2st.yaml
      python3 ~/clearance_scan.py --state ... --cans-from-feed ~/drone_feed.jsonl
"""
import argparse
import json
import math
import os
import sys

import yaml

sys.path.insert(0, os.path.expanduser("~"))
import plan_next as P


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", default=os.path.expanduser("~/mission_state_2st.yaml"))
    ap.add_argument("--cans-from-feed", default=None,
                    help="从无人机 feed 读罐子(忽略 reveal_at, 当成全部已知)")
    a = ap.parse_args()

    state = yaml.safe_load(open(a.state)) or {}
    if a.cans_from_feed:
        state["cans"] = {}
        for line in open(a.cans_from_feed):
            r = json.loads(line.strip())
            state["cans"][r["id"]] = {"x": r["x"], "y": r["y"], "color": r.get("color", "red"),
                                      "source": "drone", "status": "pending", "attempts": 0}
    stations = P.stations_of(state)
    cans = {n: (c["x"], c["y"]) for n, c in (state.get("cans") or {}).items()}
    ref = P.approach_ref(state, stations)

    print("=== 全布局净空扫描 (必撞线 %.3f = 车体 %.2f + 罐子 %.3f / 告警线 %.2f) ==="
          % (P.HARD, P.ROBOT_R, P.CAN_R, P.WARN))
    print("    %d 个罐子, %d 个站。每条腿 = 某个站 <-> 某个罐子的 standoff。\n"
          % (len(cans), len(stations)))

    tight = []
    for cn, cxy in sorted(cans.items()):
        for sn, s in stations.items():
            dest = s[:2]
            others = [(o, cans[o]) for o in cans if o != cn]
            yaw, stand, worst, blocker, dev = P.pick_yaw(cxy, dest, dest, others, ref)
            trip = (math.hypot(dest[0] - stand[0], dest[1] - stand[1])
                    + math.hypot(stand[0] - dest[0], stand[1] - dest[1]))
            if worst < P.HARD:
                tag, lvl = "⛔ 必撞", 2
            elif worst < P.WARN:
                tag, lvl = "⚠️  紧(会被几何门挡)", 1
            else:
                tag, lvl = "✅", 0
            if lvl:
                tight.append((cn, sn, worst, blocker, lvl))
            print("  %-10s <-> %-6s  净空 %5.2f m  过 %-6s  往返 %5.2f m  %s"
                  % (sn, cn, worst if worst < 90 else float("inf"),
                     blocker or "-", trip, tag))

    print("\n=== 结论 ===")
    if not tight:
        print("  没有紧的走廊, 任意顺序都不会擦到别的罐子。")
    else:
        for cn, sn, w, b, lvl in sorted(tight, key=lambda t: t[2]):
            print("  %s %s <-> %s 只有 %.2f m (被 %s 挡) —— %s"
                  % ("⛔" if lvl == 2 else "⚠️ ", sn, cn, w, b,
                     "先收走 %s 这条路就空了" % b))
    # 有没有罐子被彻底封死(所有站都进不去)
    dead = [cn for cn in cans
            if all(any(t[0] == cn and t[1] == sn and t[4] for t in tight)
                   for sn in stations)]
    if dead:
        print("\n  🚨 这些罐子**从任何站出发都进不去**(要先收走挡路的): %s" % ", ".join(dead))
    else:
        print("\n  每个罐子至少有一个站可以安全往返。")


if __name__ == "__main__":
    main()
