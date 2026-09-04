#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从车上 llm_nav_places.yaml 拉取录好的瓶子航点坐标,按类型分配表生成"模拟无人机探测清单"。
输出本机 ~/bottles_detected.json,给 bottle_run.py 用。

用法:
    python3 make_detections.py                 # 用默认类型分配(b1,b3,b5=可乐; b2,b4=矿泉水)
"""
import os
import json, os, subprocess, sys

def _env(key, default=""):
    """读现场参数：优先环境变量，其次 ~/.jetrover_env（见仓库 jetrover_env.example）。"""
    v = os.environ.get(key)
    if v:
        return v
    p = os.path.expanduser("~/.jetrover_env")
    if os.path.isfile(p):
        for line in open(p):
            line = line.strip()
            if line.startswith("export %s=" % key):
                val = line.split("=", 1)[1].split("#")[0].strip().strip("\"'")
                pre = "${%s:-" % key          # 认 export CAR_IP="${CAR_IP:-10.x.x.x}" 这种写法
                if val.startswith(pre) and val.endswith("}"):
                    val = val[len(pre):-1]
                return val
    if default:
        return default
    raise SystemExit("缺少 %s：请配置 ~/.jetrover_env（参考仓库 jetrover_env.example）" % key)

CAR = "%s@%s" % (_env("CAR_USER", "uavg"), _env("CAR_IP"))
OUT = os.path.expanduser("~/bottles_detected.json")

# 类型分配:模拟无人机识别结果(没有真瓶子,类型由此表决定)。可自行改。
TYPE_MAP = {
    "b1": "可乐",
    "b2": "矿泉水",
    "b3": "可乐",
    "b4": "矿泉水",
    "b5": "可乐",
}


def fetch_places():
    """SSH 读车上 llm_nav_places.yaml,返回 {name: {x,y,yaw}}。"""
    proc = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", CAR,
         "cat ~/llm_nav_places.yaml"],
        capture_output=True, text=True)
    if proc.returncode != 0:
        raise SystemExit(f"[中止] 读 llm_nav_places.yaml 失败:{proc.stderr.strip()}")
    try:
        import yaml
        data = yaml.safe_load(proc.stdout) or {}
    except ImportError:
        raise SystemExit("[中止] 本机缺 pyyaml:pip install pyyaml")
    return data.get("places", {})


def main():
    places = fetch_places()
    bottles = []
    missing = []
    for name in sorted(TYPE_MAP):
        if name not in places:
            missing.append(name); continue
        p = places[name]
        bottles.append({"id": name, "type": TYPE_MAP[name],
                        "x": round(float(p["x"]), 4), "y": round(float(p["y"]), 4)})
    if missing:
        print(f"[警告] llm_nav_places.yaml 里缺这些点(没录到?): {missing}")
    if not bottles:
        raise SystemExit("[中止] 一个瓶子点都没读到,先录点。")

    with open(OUT, "w", encoding="utf-8") as f:
        json.dump({"bottles": bottles}, f, ensure_ascii=False, indent=2)
    print(f"[生成] {OUT}({len(bottles)} 瓶):")
    for b in bottles:
        print(f"        {b['id']}: {b['type']} @ ({b['x']},{b['y']})")


if __name__ == "__main__":
    main()
