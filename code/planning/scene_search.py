#!/usr/bin/env python3
"""
scene_search.py -- 搜"能让近视贪心吃亏"的场地布局。

动机: two_station_gap.py 发现加第二个站虽然造出 ~20% 的顺序优化空间,
但在 07-29 那个罐子布局上**五行贪心已经等于最优** ⇒ LLM 无从证明自己。
⇒ 与其争论要不要加站, 不如直接搜: 罐子摆成什么样, 贪心才会比最优明显差。

搜到的模式再交给用户摆场地。车不用开机。

用法: python3 ~/scene_search.py [试多少个布局, 默认 4000]
"""
import math
import random
import sys
from itertools import permutations

N = 5
FIELD_X = (0.0, 3.2)      # 参照 07-29 那块场地(约 3m x 2.2m), 留点余量
FIELD_Y = (-3.0, 0.2)
MIN_SEP = 0.55            # 罐子之间最小间距: 太挤的话导航腿会互相挡(geom_check 那条约束)
TRIALS = int(sys.argv[1]) if len(sys.argv) > 1 else 4000

A = (-0.8, 0.4)           # 站A 左上角
B = (3.6, -3.2)           # 站B 右下角(对角, 拉开距离才有"横穿场地"的代价)
START = A
PERMS = list(permutations(range(N)))


def d(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def cost_sorted(order, cans, colors):
    """站由颜色定死(红→A 绿→B), 只有顺序自由。"""
    tot, cur = 0.0, START
    for i in order:
        s = A if colors[i] == "red" else B
        tot += d(cur, cans[i]) + d(cans[i], s)
        cur = s
    return tot


def greedy_order(cans, colors):
    """近视基线: 从当前位置挑最近的罐子。"""
    left = set(range(N))
    order, cur = [], START
    while left:
        i = min(left, key=lambda k: d(cur, cans[k]))
        order.append(i)
        left.discard(i)
        cur = A if colors[i] == "red" else B
    return order


def sample_layout(rng):
    cans = []
    while len(cans) < N:
        p = (rng.uniform(*FIELD_X), rng.uniform(*FIELD_Y))
        if all(d(p, q) >= MIN_SEP for q in cans):
            cans.append(p)
    # 颜色: 2 绿 3 红(和分类任务对得上, 且两边都有活干)
    colors = ["red"] * 3 + ["green"] * 2
    rng.shuffle(colors)
    return cans, colors


def main():
    rng = random.Random(20260801)
    results = []
    for _ in range(TRIALS):
        cans, colors = sample_layout(rng)
        best = min(PERMS, key=lambda o: cost_sorted(o, cans, colors))
        cb = cost_sorted(best, cans, colors)
        g = greedy_order(cans, colors)
        cg = cost_sorted(g, cans, colors)
        results.append(((cg - cb) / cb, cg - cb, cans, colors, best, g))

    results.sort(reverse=True)
    gaps = [r[0] for r in results]
    nonzero = [x for x in gaps if x > 0.01]
    print("试了 %d 个布局(5 罐, 3红2绿, 站A%s 站B%s)" % (TRIALS, A, B))
    print("  贪心就是最优的比例      : %.0f%%" % (100 * (1 - len(nonzero) / len(gaps))))
    print("  贪心亏 >5%% 的比例       : %.0f%%" % (100 * sum(g > 0.05 for g in gaps) / len(gaps)))
    print("  贪心亏 >10%% 的比例      : %.0f%%" % (100 * sum(g > 0.10 for g in gaps) / len(gaps)))
    print("  最惨的那个布局亏        : %.0f%% (%.2f m)" % (100 * gaps[0], results[0][1]))

    print("\n=== 贪心亏最多的三个布局(照这个摆, LLM 才有得证明) ===")
    for rank, (gap, absgap, cans, colors, best, g) in enumerate(results[:3], 1):
        print("\n  【第%d名】贪心多走 %.2f m (+%.0f%%)" % (rank, absgap, 100 * gap))
        for i in range(N):
            print("     can%d %-5s (%+.2f, %+.2f)" % (i + 1, colors[i], cans[i][0], cans[i][1]))
        print("     最优顺序: %s" % " ".join("can%d(%s)" % (i + 1, colors[i][0]) for i in best))
        print("     贪心顺序: %s" % " ".join("can%d(%s)" % (i + 1, colors[i][0]) for i in g))
        print("     颜色序列 最优 %s  vs  贪心 %s"
              % ("".join(colors[i][0] for i in best), "".join(colors[i][0] for i in g)))


if __name__ == "__main__":
    main()
