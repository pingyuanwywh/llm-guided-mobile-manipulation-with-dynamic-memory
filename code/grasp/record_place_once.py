#!/usr/bin/env python3
"""One-shot: record current map->base_footprint as a named place in llm_nav_places.yaml.
Usage: python3 record_place_once.py NAME [--file ~/llm_nav_places.yaml]
"""
import argparse, math, os, sys
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
    args = ap.parse_args()

    rospy.init_node("record_place_once", anonymous=True)
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)

    t = None
    for _ in range(60):
        try:
            t = buf.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(0.2))
            break
        except Exception:
            rospy.sleep(0.1)
    if t is None:
        print("ERROR: no TF map->base_footprint")
        sys.exit(1)

    x = t.transform.translation.x
    y = t.transform.translation.y
    yaw = get_yaw(t.transform.rotation)

    data = {}
    if os.path.exists(args.file):
        with open(args.file) as f:
            data = yaml.safe_load(f) or {}
    places = data.get("places", {}) or {}
    places[args.name] = {"x": round(float(x), 4), "y": round(float(y), 4), "yaw": round(float(yaw), 4)}
    with open(args.file, "w") as f:
        yaml.dump({"places": places}, f, allow_unicode=True, default_flow_style=False)

    print("RECORDED %s: x=%.4f y=%.4f yaw=%.4f (%.1f deg)" % (args.name, x, y, yaw, math.degrees(yaw)))
    print("places now: %s" % sorted(places.keys()))


if __name__ == "__main__":
    main()
