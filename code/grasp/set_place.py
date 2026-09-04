#!/usr/bin/env python3
"""
set_place.py -- 把一个**算出来的** standoff 位姿写进 ~/llm_nav_places.yaml (2026-08-01)。

这是无人机这条线和已验证导航栈的**唯一接口**:
  无人机给罐子 x/y -> planner 合成 standoff 位姿 -> 本脚本写进 yaml
  -> `nav_goto.py --place NAME` 照常读它 -> 下游一个字都不用改。

手工录点(standoff.sh -> record_place_once.py)写的是同一个文件同一个格式,
所以两种来源可以混用 —— 正好用来做对照: 手工录的当 ground truth, 算出来的看差多少。

用法: python3 ~/set_place.py can3 1.234 -0.567 -12.5      # x y yaw(度)
      python3 ~/set_place.py --drop can3                   # 删掉
"""
import math
import os
import sys

import yaml

F = os.path.expanduser("~/llm_nav_places.yaml")


def load():
    if not os.path.exists(F):
        return {"places": {}}
    d = yaml.safe_load(open(F)) or {}
    d.setdefault("places", {})
    return d


def save(d):
    with open(F, "w") as f:
        yaml.safe_dump(d, f, default_flow_style=False)


def main():
    a = sys.argv[1:]
    if not a:
        print(__doc__)
        sys.exit(1)
    d = load()
    if a[0] == "--drop":
        if d["places"].pop(a[1], None) is None:
            print("没有这个点: %s" % a[1])
        else:
            save(d)
            print("已删除 %s" % a[1])
        return
    name, x, y, yaw_deg = a[0], float(a[1]), float(a[2]), float(a[3])
    d["places"][name] = {"x": x, "y": y, "yaw": math.radians(yaw_deg)}
    save(d)
    # 回读确认 —— 落盘类操作必须回读, 静默失败 + 乐观提示是最坏组合(07-28 教训)
    back = load()["places"][name]
    print("已写入 %s: x=%+.3f y=%+.3f yaw=%+.1f deg"
          % (name, back["x"], back["y"], math.degrees(back["yaw"])))


if __name__ == "__main__":
    main()
