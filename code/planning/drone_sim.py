#!/usr/bin/env python3
"""
drone_sim.py -- 模拟无人机陆续上报罐子坐标 (2026-08-01)。

**不起后台进程**: 一次性把整个 feed 连同"什么时候该被看见"(reveal_at 绝对时刻)写进
JSONL, mission_run.py 每轮只捞 reveal_at <= now 的。这样没有竞态、可复现、可重放。
真无人机接进来时做的事一模一样 —— 往同一个文件 append 一行, reveal_at 填当前时刻。

feed 每行:
  {"id":"can3","x":0.84,"y":-1.49,"color":"green","frame":"map","reveal_at":1785332697.2}
  frame 可以是 map(已对齐) 或 drone(未对齐, 由 mission_run 套变换)。

用法:
  python3 ~/drone_sim.py --scenario five_staggered --start-in 5
  python3 ~/drone_sim.py --list
"""
import argparse
import json
import math
import os
import time

FEED = os.path.expanduser("~/drone_feed.jsonl")

# 07-29 晚那趟实测 standoff 反推的真实罐子位置(geom_check.py 那张表)
REAL = {
    "can1": (0.379, 0.020, "red"),
    "can2": (1.686, -0.115, "red"),
    "can3": (0.836, -1.488, "green"),
    "can4": (2.362, -1.436, "red"),
    "can5": (0.555, -2.620, "red"),
}

SCENARIOS = {}


def scen(fn):
    SCENARIOS[fn.__name__] = fn
    return fn


@scen
def all_at_once():
    """五个罐子同时报完 —— 退化成静态规划, 当对照组。"""
    return [(0, n) for n in REAL]


@scen
def five_staggered():
    """五个罐子每隔 40 秒报一个 —— 车收第一个的时候还不知道后面有什么。"""
    return [(i * 40, n) for i, n in enumerate(REAL)]


@scen
def two_then_burst():
    """先报 2 个, 车干起来之后第 70 秒一口气再来 3 个(测去抖 + 重规划)。"""
    return [(0, "can1"), (0, "can3"), (70, "can2"), (72, "can4"), (74, "can5")]


@scen
def blocker_arrives_late():
    """车正开向 can4 时(第 45 秒), 报来一个正好挡在 can4 路上的新罐子。
    这是**单站下唯一有正当理由改道**的情况, 也是最该录进视频的镜头。"""
    out = [(0, "can1"), (0, "can4"), (0, "can5")]
    return out + [(45, "can6")]


@scen
def interrupt_demo():
    """**打断镜头专用**: 起手只知道 can4(远)和 can5。车开向 can4 的途中(第 25 秒)
    无人机报来 can6, 正好挡在 can4 路上 ⇒ can4 当场变成去不了, 车必须掉头先收 can6。"""
    return [(0, "can4"), (0, "can5"), (25, "can6")]


@scen
def abort_demo():
    """**该掉头的那一半**: 起手只知道 can4(远)。车开向它的途中(第 25 秒)无人机报来 can6,
    正好挡在 can4 路上 => can4 当场变成去不了, 必须放弃当前目标先收 can6。"""
    return [(0, "can4"), (25, "can6")]


def resolve(name):
    if name in REAL:
        x, y, c = REAL[name]
        return x, y, c
    if name == "can6":
        # 挡在 collect_a -> can4 连线上 62% 处
        ax, ay = -1.169, -1.136
        cx, cy = REAL["can4"][:2]
        return ax + 0.62 * (cx - ax), ay + 0.62 * (cy - ay), "red"
    raise SystemExit("不认识的罐子: %s" % name)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", default="five_staggered")
    ap.add_argument("--start-in", type=float, default=0.0, help="第一条几秒后可见")
    ap.add_argument("--speed", type=float, default=1.0, help="时间压缩倍数(调试用, 2=快一倍)")
    ap.add_argument("--noise", type=float, default=0.0, help="给坐标加高斯噪声(米), 模拟定位误差")
    ap.add_argument("--out", default=FEED)
    ap.add_argument("--list", action="store_true")
    a = ap.parse_args()

    if a.list:
        for n, f in SCENARIOS.items():
            print("  %-22s %s" % (n, f.__doc__.strip().replace("\n", " ")))
        return
    if a.scenario not in SCENARIOS:
        raise SystemExit("没有这个场景, 用 --list 看")

    import random
    rng = random.Random(20260801)
    t0 = time.time() + a.start_in
    lines = []
    for delay, name in SCENARIOS[a.scenario]():
        x, y, color = resolve(name)
        if a.noise:
            x += rng.gauss(0, a.noise)
            y += rng.gauss(0, a.noise)
        lines.append({"id": name, "x": round(x, 4), "y": round(y, 4), "color": color,
                      "frame": "map", "reveal_at": round(t0 + delay / a.speed, 2)})

    with open(a.out, "w") as f:
        for L in lines:
            f.write(json.dumps(L, ensure_ascii=False) + "\n")

    print("场景 %s -> %s  (%d 个罐子%s)"
          % (a.scenario, a.out, len(lines), ", 坐标噪声 σ=%.2fm" % a.noise if a.noise else ""))
    for L in lines:
        print("  +%5.0fs  %-5s %-5s (%+.2f, %+.2f)"
              % (L["reveal_at"] - t0, L["id"], L["color"], L["x"], L["y"]))


if __name__ == "__main__":
    main()
