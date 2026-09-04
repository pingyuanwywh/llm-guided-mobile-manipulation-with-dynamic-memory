#!/usr/bin/env python3
# 跑前几何检查: 罐子矮 => 雷达扫不到 => costmap 里根本没有它们 => 车会直接撞飞。
# 所以必须先算"每条腿的直线路径会不会从别的罐子旁边太近掠过"。
# 罐子位置按 standoff 位姿反推: 沿录点朝向前 0.45m (逼近环收敛到 0.35 后又退了 0.08~0.10)。
import math
from itertools import permutations

STANDOFF = {
    'can1':    (-0.0618, -0.0697,  0.2015),
    'can2':    ( 1.3320, -0.3930,  0.6663),
    'can3':    ( 0.4282, -1.2955, -0.4416),
    'can4':    ( 2.1060, -1.0666, -0.9657),
    'can5':    ( 0.1371, -2.4539, -0.3784),
    'collect': (-1.1689, -1.1364,  2.9815),
}
CAN_AHEAD = 0.45          # standoff 点前方多远是罐子
ROBOT_R   = 0.16          # 车体半径
CAN_R     = 0.033         # 罐子半径 (Ø66mm)
HARD      = ROBOT_R + CAN_R   # 0.193 => 必撞
WARN      = 0.30              # 留余量的告警线

cans = {}
for n, (x, y, yaw) in STANDOFF.items():
    if n == 'collect':
        continue
    cans[n] = (x + CAN_AHEAD * math.cos(yaw), y + CAN_AHEAD * math.sin(yaw))


def seg_pt_dist(a, b, p):
    """点 p 到线段 ab 的最近距离"""
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def check_order(order, verbose=False):
    """返回 (最小净空, 危险腿列表)。每收一个罐 = 去 canN 再回 collect。"""
    col = STANDOFF['collect'][:2]
    worst = 9.9
    bad = []
    remaining = set(order)
    for name in order:
        stand = STANDOFF[name][:2]
        for leg in ((col, stand), (stand, col)):
            for other in remaining:
                if other == name:
                    continue          # 目标罐自己不算(本来就要开过去)
                d = seg_pt_dist(leg[0], leg[1], cans[other])
                if d < worst:
                    worst = d
                if d < WARN:
                    bad.append((name, other, d))
                if verbose:
                    print('  %-8s腿  过 %-5s  %.2f m' % (name, other, d))
        remaining.discard(name)       # 收走之后这条路就空了
    return worst, bad


print('=== 反推的罐子位置 (map 系) ===')
for n in sorted(cans):
    print('  %-5s (%+.2f, %+.2f)' % (n, cans[n][0], cans[n][1]))

cur = ['can1', 'can2', 'can4', 'can3', 'can5']
print('\n=== 脚本当前顺序 %s ===' % ' '.join(cur))
worst, bad = check_order(cur, verbose=True)
print('最小净空 %.2f m   (必撞线 %.2f / 告警线 %.2f)' % (worst, HARD, WARN))
if bad:
    for a, b, d in bad:
        print('  !! %s 那条腿会从 %s 旁 %.2f m 掠过' % (a, b, d))
else:
    print('  OK: 没有腿会贴到别的罐子')

print('\n=== 全排列里最安全的几个顺序 ===')
scored = []
for p in permutations(['can1', 'can2', 'can3', 'can4', 'can5']):
    w, b = check_order(list(p))
    scored.append((w, list(p)))
scored.sort(reverse=True)
for w, p in scored[:5]:
    print('  %.2f m   %s' % (w, ' '.join(p)))
print('  ...')
for w, p in scored[-3:]:
    print('  %.2f m   %s   <-- 最差' % (w, ' '.join(p)))
