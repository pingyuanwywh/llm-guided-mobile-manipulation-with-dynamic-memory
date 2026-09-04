#!/usr/bin/env python3
"""一键把当前 map->base_footprint 位姿录成一个航点(不用交互 input, 避开方向键乱码坑)。
用法: python3 record_point.py <name> [--file ~/llm_nav_places.yaml] [--fresh]
  --fresh: 先备份并清空原文件, 只留本次录的点。
"""
import argparse, math, os, shutil, sys, time
import rospy, tf2_ros, yaml

DEFAULT_FILE = os.path.expanduser("~/llm_nav_places.yaml")

def get_yaw(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("name")
    ap.add_argument("--file", default=DEFAULT_FILE)
    ap.add_argument("--fresh", action="store_true")
    a = ap.parse_args()

    rospy.init_node("record_point", anonymous=True, disable_signals=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)

    places = {}
    if os.path.exists(a.file) and not a.fresh:
        with open(a.file) as f:
            places = (yaml.safe_load(f) or {}).get("places", {}) or {}
    elif os.path.exists(a.file) and a.fresh:
        shutil.copy(a.file, a.file + ".bak." + time.strftime("%H%M%S"))

    try:
        t = buf.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(3.0))
    except Exception as e:
        print("TF FAIL:", e); sys.exit(2)

    p = t.transform.translation
    yaw = get_yaw(t.transform.rotation)
    places[a.name] = {"x": round(p.x, 4), "y": round(p.y, 4), "yaw": round(yaw, 4)}
    with open(a.file, "w") as f:
        yaml.dump({"places": places}, f, allow_unicode=True, default_flow_style=False)
    print("RECORDED [%s]: x=%.3f y=%.3f yaw=%.1fdeg  (total %d: %s)"
          % (a.name, p.x, p.y, math.degrees(yaw), len(places), list(places.keys())))

if __name__ == "__main__":
    main()
