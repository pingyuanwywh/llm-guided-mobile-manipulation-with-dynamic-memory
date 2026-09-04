#!/usr/bin/env python3
"""
mission_review.py -- 跑完之后复盘 (2026-08-01)。

回答三个问题, 每个都有数:
  ① **算出来的 standoff 和手工录的差多少** —— 手工录的是 ground truth(逼近环收敛过、抓成功过)。
     位置差 + 朝向差。⚠️ 朝向差大不等于错: 罐身通体是色的, 从哪边接近都行,
     算出来的 standoff 按构造一定是"离罐 0.45m 正对着它"。所以要看的是**抓取结果**, 不是角度差本身。
  ② **LLM 走的路 vs 离线最优 vs 在线贪心** —— 离线最优是上帝视角(知道全部罐子)的理论下界,
     在线的永远赢不了它, 那个差距 = "不知道未来的代价"。
  ③ **每一次决策 LLM 看到了什么、选了什么、为什么** —— 从 mission_log.jsonl 逐条列出。

用法: python3 ~/mission_review.py [--state ~/mission_state_2st.yaml] [--log ~/mission_log.jsonl]
"""
import argparse
import json
import math
import os
from itertools import permutations

import yaml

# 08-01 手工录的 standoff (ground truth)
RECORDED = {
    "can1": (-0.6497, 0.8083, 1.736),
    "can2": (-1.8538, 0.6039, -2.8621),
    "can3": (-1.9941, -0.6793, -1.5024),
    "can4": (-3.5983, -0.5356, -2.799),
    "can5": (-3.5039, 1.1584, 1.8211),
}
CAN_AHEAD = 0.45


def d(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def angdiff(a, b):
    return abs((a - b + math.pi) % (2 * math.pi) - math.pi)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", default=os.path.expanduser("~/mission_state_2st.yaml"))
    ap.add_argument("--log", default=os.path.expanduser("~/mission_log.jsonl"))
    a = ap.parse_args()

    state = yaml.safe_load(open(a.state)) or {}
    cans = state.get("cans") or {}
    st = {n: (s["x"], s["y"]) for n, s in (state.get("stations") or {}).items()}
    rows = [json.loads(l) for l in open(a.log)] if os.path.exists(a.log) else []

    # ---- ① 算出来的 standoff vs 手工录的 ----
    print("=== ① 算出来的 standoff vs 手工录的 (ground truth) ===")
    print("  %-6s %-22s %-22s %8s %8s  %s" % ("罐", "手工录", "算出来", "位置差", "朝向差", "抓取结果"))
    picks = {}
    for r in rows:
        dec = r.get("decision", {})
        if r.get("stage") == "abort?":
            continue
        n = dec.get("next")
        if n and n not in ("done", "continue"):
            picks[n] = dec
    for n in sorted(RECORDED):
        rx, ry, ryaw = RECORDED[n]
        c = cans.get(n)
        if not c:
            print("  %-6s (无人机没报到 / 没跑到)" % n)
            continue
        # 从记忆里的罐子位置反推那次实际用的 standoff 要重算一遍, 简单起见按"正对罐子"复原
        cx, cy = c["x"], c["y"]
        # 手工录的那个 standoff 指向的罐子位置(用来交叉验证坐标一致性)
        mx = rx + CAN_AHEAD * math.cos(ryaw)
        my = ry + CAN_AHEAD * math.sin(ryaw)
        pos_err = d((mx, my), (cx, cy))
        status = c.get("status", "?")
        mark = {"collected": "✅ 收进 " + str(c.get("station", "")),
                "failed": "❌ " + str(c.get("note", ""))[:24],
                "stuck": "🛑 卡住"}.get(status, status)
        print("  %-6s (%+.2f,%+.2f,%+4.0f°)   罐子位置一致性 %.3fm   %s   试 %d 次"
              % (n, rx, ry, math.degrees(ryaw), pos_err, mark, c.get("attempts", 0)))
    print("  (罐子位置一致性 = 手工 standoff 前推 0.45m 得到的罐子位置, 与无人机报的差多少;"
          " 今天无人机坐标就是这么造的, 所以应该 ~0, 非 0 说明中间算错了)")

    # ---- ② 路程对照 ----
    order = [r["decision"]["next"] for r in rows
             if r.get("stage") != "abort?" and r.get("decision", {}).get("next")
             not in (None, "done", "continue")]
    seen, real_order = set(), []
    for n in order:
        if n not in seen and cans.get(n, {}).get("status") == "collected":
            seen.add(n)
            real_order.append(n)
    live = {n: (c["x"], c["y"]) for n, c in cans.items()}
    if len(live) >= 2 and st:
        A = st.get("collect_a") or list(st.values())[0]

        def cost(o):
            tot, cur = 0.0, A
            for i, n in enumerate(o):
                tot += d(cur, live[n])
                nxt = live[o[i + 1]] if i + 1 < len(o) else None
                best = None
                for s in st.values():
                    c = d(live[n], s) + (d(s, nxt) if nxt else 0)
                    if best is None or c < best:
                        best, cur = c, s
                tot += d(live[n], cur)
            return tot

        def greedy():
            left, o, cur = set(live), [], A
            while left:
                n = min(left, key=lambda k: d(cur, live[k]))
                o.append(n)
                left.discard(n)
                rest = [live[k] for k in left]
                cur = min(st.values(), key=lambda s: d(live[n], s) + (
                    min(d(s, r) for r in rest) if rest else 0))
            return o

        print("\n=== ② 路程: LLM 在线 vs 离线最优 vs 在线贪心 ===")
        print("  ⚠️ 这是**直线距离**, 不是车实际走的路(TEB 会绕障)。用来比顺序好坏, 不是真实里程。")
        if len(real_order) == len(live):
            P = list(permutations(live))
            best = min(P, key=cost)
            g = greedy()
            cb, cg, cl = cost(best), cost(g), cost(real_order)
            print("  离线最优(上帝视角) %-30s %.2f m" % (" ".join(best), cb))
            print("  在线贪心(最简基线) %-30s %.2f m  (+%.0f%%)"
                  % (" ".join(g), cg, 100 * (cg - cb) / cb))
            print("  **LLM 在线**       %-30s %.2f m  (+%.0f%%)"
                  % (" ".join(real_order), cl, 100 * (cl - cb) / cb))
            print("  ⇒ 不知道未来的代价 = %.2f m" % (cl - cb))
        else:
            print("  只收了 %d/%d, 顺序 %s —— 没收全就不做全排列对照(不可比)"
                  % (len(real_order), len(live), " ".join(real_order)))

    # ---- ③ 决策逐条 ----
    print("\n=== ③ 决策逐条 ===")
    k = 0
    for r in rows:
        dec = r.get("decision", {})
        if r.get("stage") == "abort?":
            print("  [途中被打断询问] %s -> %s" % (
                json.dumps(r.get("ctx", {}), ensure_ascii=False), dec.get("abort")))
            print("        理由: %s" % dec.get("reason"))
            continue
        if dec.get("next") in (None,):
            continue
        k += 1
        print("  [%d] 可选 %s%s" % (k, r.get("options"),
                                    ("  被挡: " + ",".join(r["blocked"])) if r.get("blocked") else ""))
        print("      选 %s%s" % (dec.get("next"),
                                 ("  送 " + dec["station"]) if dec.get("station") else ""))
        print("      理由: %s" % dec.get("reason"))


if __name__ == "__main__":
    main()
