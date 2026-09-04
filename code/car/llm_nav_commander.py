#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM navigation commander for ROS1 move_base.

The LLM is only allowed to produce a small JSON command. This node validates the
command and then talks to move_base, so free-form model text never reaches motor
control directly.
"""
import argparse
import json
import math
import os
import sys

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Quaternion
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

try:
    import yaml
except ImportError:
    yaml = None


DEFAULT_PLACES = os.path.expanduser("~/llm_nav_places.yaml")


def yaw_to_quaternion(yaw):
    half = yaw * 0.5
    return Quaternion(0.0, 0.0, math.sin(half), math.cos(half))


def normalize_angle(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def load_places(path):
    if not path or not os.path.exists(path):
        return {}
    with open(path, "r", encoding="utf-8") as f:
        text = f.read()
    if yaml:
        data = yaml.safe_load(text) or {}
    else:
        data = json.loads(text)
    return data.get("places", data) or {}


class LlmNavCommander:
    def __init__(self, places_path=DEFAULT_PLACES, frame_id="map"):
        self.frame_id = frame_id
        self.places = load_places(places_path)
        self.client = None

    def connect_move_base(self):
        if self.client is not None:
            return
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for /move_base action server...")
        if not self.client.wait_for_server(rospy.Duration(8.0)):
            raise RuntimeError("move_base action server is not available")
        rospy.loginfo("Connected to /move_base")

    def goal_from_pose(self, x, y, yaw=0.0):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.frame_id
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = float(x)
        goal.target_pose.pose.position.y = float(y)
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation = yaw_to_quaternion(normalize_angle(float(yaw)))
        return goal

    def send_goal(self, x, y, yaw=0.0, wait=False):
        self.connect_move_base()
        goal = self.goal_from_pose(x, y, yaw)
        self.client.send_goal(goal)
        rospy.loginfo("Sent goal: x=%.3f y=%.3f yaw=%.3f", x, y, yaw)
        if wait:
            self.client.wait_for_result()
            return self.status()
        return {"ok": True, "status": "sent", "x": x, "y": y, "yaw": yaw}

    def goto_place(self, name, wait=False):
        if name not in self.places:
            known = sorted(self.places.keys())
            return {"ok": False, "error": "unknown_place", "place": name, "known_places": known}
        pose = self.places[name]
        return self.send_goal(pose["x"], pose["y"], pose.get("yaw", 0.0), wait=wait)

    def stop(self):
        self.connect_move_base()
        self.client.cancel_all_goals()
        rospy.logwarn("Cancelled all move_base goals")
        return {"ok": True, "status": "stopped"}

    def status(self):
        self.connect_move_base()
        code = self.client.get_state()
        return {
            "ok": True,
            "state": code,
            "state_text": GoalStatus.to_string(code),
        }

    def execute(self, command):
        action = command.get("action")
        if action == "goto":
            if "place" in command:
                return self.goto_place(command["place"], wait=bool(command.get("wait", False)))
            if "x" in command and "y" in command:
                return self.send_goal(
                    command["x"],
                    command["y"],
                    command.get("yaw", 0.0),
                    wait=bool(command.get("wait", False)),
                )
            return {"ok": False, "error": "goto_requires_place_or_xy"}
        if action == "stop":
            return self.stop()
        if action == "status":
            return self.status()
        if action == "places":
            return {"ok": True, "places": sorted(self.places.keys())}
        return {"ok": False, "error": "unsupported_action", "action": action}


def parse_command(args):
    if args.json_command:
        return json.loads(args.json_command)
    if args.action == "goto":
        if args.place:
            return {"action": "goto", "place": args.place, "wait": args.wait}
        return {"action": "goto", "x": args.x, "y": args.y, "yaw": args.yaw, "wait": args.wait}
    return {"action": args.action}


def main():
    parser = argparse.ArgumentParser(description="Safe LLM command gateway for ROS1 move_base")
    parser.add_argument("action", nargs="?", choices=["goto", "stop", "status", "places"])
    parser.add_argument("--json", dest="json_command", help='LLM JSON, e.g. {"action":"goto","place":"dock"}')
    parser.add_argument("--place", help="Named place from ~/llm_nav_places.yaml")
    parser.add_argument("--x", type=float, help="Goal x in map frame")
    parser.add_argument("--y", type=float, help="Goal y in map frame")
    parser.add_argument("--yaw", type=float, default=0.0, help="Goal yaw in radians")
    parser.add_argument("--wait", action="store_true", help="Wait until move_base finishes")
    parser.add_argument("--places-file", default=DEFAULT_PLACES)
    args = parser.parse_args()

    if not args.json_command and not args.action:
        parser.error("provide an action or --json")
    if args.action == "goto" and not args.place and (args.x is None or args.y is None):
        parser.error("goto requires --place or both --x and --y")

    command = parse_command(args)
    if command.get("action") == "places":
        places = load_places(args.places_file)
        print(json.dumps({"ok": True, "places": sorted(places.keys())}, ensure_ascii=False, sort_keys=True))
        return 0

    rospy.init_node("llm_nav_commander", anonymous=True)
    commander = LlmNavCommander(places_path=args.places_file)
    result = commander.execute(command)
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0 if result.get("ok") else 2


if __name__ == "__main__":
    sys.exit(main())
