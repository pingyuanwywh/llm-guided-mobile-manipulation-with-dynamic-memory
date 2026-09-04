#!/usr/bin/env python3
"""
mission_sim.py -- 离线回放: 拿 07-29 那趟真实录的六个点当"无人机流", 造几种局面测 plan_next.py。

**车不用开机, 一分钱电不花。** 罐子坐标是从 07-29 晚那趟 5/5 的 standoff 点反推的真实值
(geom_check.py 里那张表), 不是我编的数。

用法: python3 ~/mission_sim.py [场景名...]
"""
import math
import os
import subprocess
import sys

import yaml

SCRATCH = os.environ.get("JETROVER_SCRATCH", "/tmp/jetrover_sim")

# 07-29 晚那趟实测录的 standoff (x, y, yaw) —— 来源 geom_check.py
STANDOFF = {
    "can1":    (-0.0618, -0.0697,  0.2015),
    "can2":    ( 1.3320, -0.3930,  0.6663),
    "can3":    ( 0.4282, -1.2955, -0.4416),
    "can4":    ( 2.1060, -1.0666, -0.9657),
    "can5":    ( 0.1371, -2.4539, -0.3784),
    "collect": (-1.1689, -1.1364,  2.9815),
}
COLORS = {"can1": "red", "can2": "red", "can3": "green", "can4": "red", "can5": "red"}
CAN_AHEAD = 0.45

CANS = {n: (x + CAN_AHEAD * math.cos(t), y + CAN_AHEAD * math.sin(t))
        for n, (x, y, t) in STANDOFF.items() if n != "collect"}
COLLECT = STANDOFF["collect"][:2]


def can_entry(name, xy=None, **kw):
    x, y = xy if xy else CANS[name]
    d = {"x": round(x, 4), "y": round(y, 4), "color": COLORS.get(name, "red"),
         "source": "drone", "status": "pending", "attempts": 0}
    d.update(kw)
    return d


def base(cans, active=None, holding=None):
    return {
        "frame": "map",
        "collect": {"x": COLLECT[0], "y": COLLECT[1]},
        "robot": {"x": COLLECT[0], "y": COLLECT[1], "yaw": 2.98, "holding": holding},
        "active": active,
        "cans": cans,
    }


SCENARIOS = {}


def scen(fn):
    SCENARIOS[fn.__name__] = fn
    return fn


@scen
def s1_cold_start():
    """无人机刚起飞, 只报了 2 个罐子。车空闲。"""
    return base({"can1": can_entry("can1"), "can3": can_entry("can3")})


@scen
def s2_all_five():
    """五个全报完了, 车空闲 —— 相当于 07-29 那趟的起点。"""
    return base({n: can_entry(n) for n in CANS})


@scen
def s3_blocked():
    """几何门: 把 can2 挪到 collect->can4 的连线上, can4 应该被挡住、只能先收 can2。"""
    cx, cy = CANS["can4"]
    mid = (COLLECT[0] + 0.55 * (cx - COLLECT[0]), COLLECT[1] + 0.55 * (cy - COLLECT[1]))
    return base({"can2": can_entry("can2", xy=mid), "can4": can_entry("can4")})


@scen
def s4_blocker_gone():
    """接上一场: can2 收走了 —— can4 这条路就该空出来。"""
    st = s3_blocked()
    st["cans"]["can2"]["status"] = "collected"
    return st


def _midleg(frac):
    """车空手在 collect->can4 这条腿上走到 frac 处; 此刻无人机报了个近得多的 can1。"""
    st = base({n: can_entry(n) for n in ("can4", "can5")}, active="can4")
    st["cans"]["can1"] = can_entry("can1", seen_at="刚刚")
    tx, ty = CANS["can4"]
    st["robot"]["x"] = round(COLLECT[0] + frac * (tx - COLLECT[0]), 3)
    st["robot"]["y"] = round(COLLECT[1] + frac * (ty - COLLECT[1]), 3)
    return st


@scen
def s5a_newcan_just_left():
    """中途插队(刚出发): 去 can4 的路才走了 8%, 此时报来近得多的 can1。改道几乎不亏。"""
    return _midleg(0.08)


@scen
def s5b_newcan_almost_there():
    """中途插队(快到了): 去 can4 的路已经走了 85%, 此时报来 can1。改道要白走一大段。"""
    return _midleg(0.85)


@scen
def s5c_target_got_blocked():
    """真该改道: 车在去 can4 的路上, 无人机报的新罐子正好落在 can4 的必经之路上
    ⇒ can4 进了 blocked, 只能先收挡路的那个。"""
    st = _midleg(0.35)
    cx, cy = CANS["can4"]
    mid = (COLLECT[0] + 0.62 * (cx - COLLECT[0]), COLLECT[1] + 0.62 * (cy - COLLECT[1]))
    st["cans"]["can6"] = can_entry("can2", xy=mid, seen_at="刚刚")
    st["cans"]["can6"]["color"] = "red"
    return st


@scen
def s6_failed_retry():
    """can3 空夹过一次。还有两个没试过的。该先收别的还是马上重试?"""
    st = base({n: can_entry(n) for n in ("can1", "can3", "can5")})
    st["cans"]["can3"].update(status="failed", attempts=1,
                              note="空夹 418, 罐子被碰倒过, 疑似标签背着镜头")
    return st


@scen
def s7_only_failed_left():
    """只剩那个失败过的了 —— 没有别的选择, 该重试还是收工?"""
    st = s6_failed_retry()
    for n in ("can1", "can5"):
        st["cans"][n]["status"] = "collected"
    return st


def main():
    os.makedirs(SCRATCH, exist_ok=True)
    want = sys.argv[1:] or list(SCENARIOS)
    for name in want:
        if name not in SCENARIOS:
            print("没有这个场景: %s" % name)
            continue
        path = os.path.join(SCRATCH, "state_%s.yaml" % name)
        with open(path, "w") as f:
            yaml.safe_dump(SCENARIOS[name](), f, allow_unicode=True,
                           default_flow_style=False, sort_keys=False)
        print("\n" + "=" * 78, flush=True)
        print("【%s】 %s" % (name, SCENARIOS[name].__doc__.strip()), flush=True)
        print("=" * 78, flush=True)
        subprocess.run([sys.executable, os.path.expanduser("~/plan_next.py"),
                        "--state", path])


if __name__ == "__main__":
    main()
