#!/usr/bin/env python3
"""Rotate the base in place to a target map yaw (deg) using TF feedback.
Single-direction proportional control -> no DWA-style oscillation.
Usage: python3 rotate_to.py TARGET_YAW_DEG
"""
import sys, math, rospy, tf2_ros
from geometry_msgs.msg import Twist


def get_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def main():
    rospy.init_node("rotate_to", anonymous=True)
    target = math.radians(float(sys.argv[1]))
    buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(buf)
    rospy.sleep(1.0)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rate = rospy.Rate(15)
    tol = math.radians(7)
    maxw = 0.20
    t0 = rospy.Time.now()
    yaw = 0.0
    while not rospy.is_shutdown():
        try:
            t = buf.lookup_transform("map", "base_footprint", rospy.Time(0), rospy.Duration(0.2))
        except Exception:
            rate.sleep(); continue
        yaw = get_yaw(t.transform.rotation)
        err = math.atan2(math.sin(target - yaw), math.cos(target - yaw))
        if abs(err) < tol:
            break
        if (rospy.Time.now() - t0).to_sec() > 35:
            print("TIMEOUT"); break
        w = max(-maxw, min(maxw, 1.2 * err))
        if 0 < abs(w) < 0.09:
            w = 0.09 * (1 if w > 0 else -1)
        tw = Twist(); tw.angular.z = w; pub.publish(tw); rate.sleep()
    pub.publish(Twist())
    rospy.sleep(0.3); pub.publish(Twist())
    print("done yaw=%.1f target=%.1f" % (math.degrees(yaw), math.degrees(target)))


if __name__ == "__main__":
    main()
