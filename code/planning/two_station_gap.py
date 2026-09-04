#!/usr/bin/env python3
"""
two_station_gap.py -- 加第二个垃圾站到底带来多大优化空间? 用 07-29 那趟真实罐子坐标算。

三种设定各算 贪心(近视) / 最优(暴力全排列) 两条路程, 差值就是"规划能挣到的钱"。
  ① 单站            —— 现状。总路程 = 2*sum|collect-can|, **与顺序无关**, 优化空间恒为 0。
  ② 双站·自由选站   —— 放完可选任一站。顺序和选站耦合。
  ③ 双站·按颜色分类 —— 红进A绿进B(站是约束不是选择)。顺序仍自由, 且**颜色交替会来回横穿场地**。

车不用开机。
"""
import math
from itertools import permutations

# 07-29 晚那趟实测 standoff 反推的罐子位置(geom_check.py 那张表, 沿录点朝向前 0.45m)
CANS = {
    "can1": (0.379, 0.020),
    "can2": (1.686, -0.115),
    "can3": (0.836, -1.488),
    "can4": (2.362, -1.436),
    "can5": (0.555, -2.620),
}
# 07-29 的真实颜色是 can3 绿其余全红。分类实验要颜色均衡些, 这里按"两绿三红"配。
COLOR = {"can1": "red", "can2": "red", "can3": "green", "can4": "red", "can5": "green"}

A = (-1.169, -1.136)          # 站A = 07-29 那个真实收集盒位置
B = (2.600, -2.600)           # 站B = 场地对角另一头(建议位置, 摆之前可改这里再算)
START = A                     # 车从站A起步


def d(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def cost_single(order):
    """单站: 每罐一次往返。"""
    tot = 0.0
    cur = START
    for n in order:
        tot += d(cur, CANS[n]) + d(CANS[n], A)
        cur = A
    return tot


def cost_free(order):
    """双站自由选: 给定顺序, 每步贪心选那个'去它再去下一个罐子'最短的站(这一步本身是最优的)。"""
    tot = 0.0
    cur = START
    for i, n in enumerate(order):
        tot += d(cur, CANS[n])
        nxt = CANS[order[i + 1]] if i + 1 < len(order) else None
        best, cur = None, None
        for s in (A, B):
            c = d(CANS[n], s) + (d(s, nxt) if nxt else 0.0)
            if best is None or c < best:
                best, cur = c, s
        tot += d(CANS[n], cur)
        if nxt:
            tot -= 0          # 去下个罐子的路在下一轮循环里加
    return tot


def cost_sorted(order):
    """双站分类: 站由颜色定死, 只有顺序自由。"""
    tot = 0.0
    cur = START
    for n in order:
        s = A if COLOR[n] == "red" else B
        tot += d(cur, CANS[n]) + d(CANS[n], s)
        cur = s
    return tot


def greedy(costfn):
    """近视基线: 每次挑'从当前位置到它最近'的罐子。这就是那个'五行贪心'。"""
    left = set(CANS)
    order, cur = [], START
    while left:
        n = min(left, key=lambda k: d(cur, CANS[k]))
        order.append(n)
        left.discard(n)
        if costfn is cost_single:
            cur = A
        elif costfn is cost_sorted:
            cur = A if COLOR[n] == "red" else B
        else:
            rest = [CANS[k] for k in left]
            cur = min((A, B), key=lambda s: d(CANS[n], s) + (
                min(d(s, r) for r in rest) if rest else 0))
    return order


def report(title, costfn, note=""):
    perms = list(permutations(CANS))
    best = min(perms, key=costfn)
    worst = max(perms, key=costfn)
    g = greedy(costfn)
    cb, cw, cg = costfn(best), costfn(worst), costfn(g)
    print("\n=== %s ===" % title)
    if note:
        print("    %s" % note)
    print("  最优顺序  %-32s %.2f m" % (" ".join(best), cb))
    print("  贪心近视  %-32s %.2f m   (比最优多 %.2f m = %+.0f%%)"
          % (" ".join(g), cg, cg - cb, 100 * (cg - cb) / cb))
    print("  最差顺序  %-32s %.2f m   (最优~最差跨度 %.2f m = %.0f%%)"
          % (" ".join(worst), cw, cw - cb, 100 * (cw - cb) / cb))


print("罐子(07-29 实测反推) / 站A%s 站B%s" % (A, B))
for n in sorted(CANS):
    print("  %-5s %-5s (%+.2f, %+.2f)" % (n, COLOR[n], CANS[n][0], CANS[n][1]))

report("① 单站(现状)", cost_single,
       "总路程 = 2*sum|collect-can|, 与顺序无关 ⇒ 三行必然完全相同, 优化空间 = 0")
report("② 双站·自由选站", cost_free,
       "放完可去任一站; 选站决定下一条腿从哪起步")
report("③ 双站·按颜色分类(红→A 绿→B)", cost_sorted,
       "站由颜色定死; 颜色交替会来回横穿场地, 批处理同色能省")
