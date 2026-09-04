#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""teach_place.py -- 手把手教"投篮"姿势 (2026-07-25)

走 board_node topic/service, 不抢串口。
  --limp     卸掉手臂舵机(1-5)扭矩 => 变软可手摆 (夹爪10不动, 仍夹持)
  --read     读回 1-5 当前角度, 存 ~/place_dunk_pose.txt, 并在当前位置重新上扭矩(锁住)
  --replay [--grip hold|open] [--duration D]
             按存的姿势移动手臂(用于 place); grip open 则移动到位后再张爪投下
典型: 车停桶前 -> (扶住臂) teach_place.py --limp -> 手摆到投篮位 -> teach_place.py --read
"""
import sys, os, argparse, rospy
from ros_robot_controller.msg import BusServoState, SetBusServoState, GetBusServoCmd
from ros_robot_controller.srv import GetBusServoState, GetBusServoStateRequest

ARM = [1, 2, 3, 4, 5]
GRIP = 10
SAVE = os.path.expanduser('~/place_dunk_pose.txt')


def state(servo_id, position=None, torque=None, stop=False):
    s = BusServoState()
    s.present_id = [1, int(servo_id)]
    s.target_id = [0, 0]
    s.position = [1, int(position)] if position is not None else [0, 0]
    s.offset = [0, 0]
    s.voltage = [0, 0]
    s.temperature = [0, 0]
    s.position_limit = [0, 0, 0]
    s.voltage_limit = [0, 0, 0]
    s.max_temperature_limit = [0, 0]
    s.enable_torque = [1, int(torque)] if torque is not None else [0, 0]
    s.save_offset = [0]
    s.stop = [1] if stop else [0]
    return s


def publish(pub, duration, states):
    m = SetBusServoState()
    m.duration = float(duration)
    m.state = states
    for _ in range(3):
        pub.publish(m)
        rospy.sleep(0.05)


def read_arm(get):
    out = {}
    for sid in ARM + [GRIP]:
        c = GetBusServoCmd()
        c.id = sid
        c.get_position = 1
        try:
            r = get(GetBusServoStateRequest(cmd=[c]))
            out[sid] = int(r.state[0].position[0]) if (r.success and r.state and len(r.state[0].position) > 0) else None
        except Exception:
            out[sid] = None
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--limp', action='store_true')
    ap.add_argument('--read', action='store_true')
    ap.add_argument('--replay', action='store_true')
    ap.add_argument('--set', dest='setpose', default=None, help='"p1,p2,p3,p4,p5" 命令手臂到该姿势(锁住), 用于 jog 对位')
    ap.add_argument('--grip-set', dest='gripset', type=int, default=None, help='夹爪(10)到指定值: 张开~63 夹罐合到~500(被罐挡在~250)')
    ap.add_argument('--grip', choices=['hold', 'open'], default='hold')
    ap.add_argument('--duration', type=float, default=2.5)
    a = ap.parse_args()

    rospy.init_node('teach_place', anonymous=True)
    pub = rospy.Publisher('/board/bus_servo/set_state', SetBusServoState, queue_size=1)
    get = rospy.ServiceProxy('/board/bus_servo/get_state', GetBusServoState)
    get.wait_for_service(timeout=10)
    rospy.sleep(0.5)

    if a.limp:
        # 先发 stop 打断"运动到目标并保持"状态(否则光卸扭矩压不住), 再卸扭矩
        publish(pub, 0.0, [state(s, stop=True) for s in ARM])
        rospy.sleep(0.15)
        publish(pub, 0.0, [state(s, torque=0, stop=True) for s in ARM])
        print('手臂舵机 1-5 已 stop+卸扭矩, 现在可用手摆(夹爪10未动)。摆好后运行 --read')
        return

    if a.gripset is not None:
        publish(pub, 0.8, [state(GRIP, position=a.gripset, torque=1)])
        rospy.sleep(1.0)
        cur = read_arm(get)
        print('夹爪 -> %d, 读回 10 = %s (夹住易拉罐约250, 空爪约418, 张开约63)' % (a.gripset, cur.get(GRIP)))
        return

    if a.setpose:
        pulses = [int(x) for x in a.setpose.replace(' ', ',').split(',') if x != '']
        publish(pub, a.duration, [state(s, position=pulses[i], torque=1) for i, s in enumerate(ARM)])
        print('已命令手臂到 %s (duration=%.1f)' % (pulses, a.duration))
        return

    if a.read:
        cur = read_arm(get)
        pulses = [cur[s] for s in ARM]
        if any(p is None for p in pulses):
            rospy.sleep(0.3)
            cur = read_arm(get)
            pulses = [cur[s] for s in ARM]
        print('读到 1-5 =', pulses, ' 夹爪10 =', cur.get(GRIP))
        if any(p is None for p in pulses):
            print('!! 有舵机没读到, 别存, 重试'); sys.exit(1)
        with open(SAVE, 'w') as f:
            f.write(' '.join(str(p) for p in pulses) + '\n')
        publish(pub, 0.0, [state(s, position=pulses[i], torque=1) for i, s in enumerate(ARM)])
        print('已存 %s 并在当前位置重新上扭矩(锁住): %s' % (SAVE, pulses))
        return

    if a.replay:
        with open(SAVE) as f:
            pulses = [int(x) for x in f.read().split()]
        publish(pub, a.duration, [state(s, position=pulses[i], torque=1) for i, s in enumerate(ARM)])
        print('回放投篮姿势 %s (duration=%.1f)' % (pulses, a.duration))
        if a.grip == 'open':
            rospy.sleep(a.duration + 0.5)
            publish(pub, 0.6, [state(GRIP, position=63, torque=1)])
            print('夹爪张开(63), 罐子应已投下')
        return

    print('要指定 --limp / --read / --replay 之一')


if __name__ == '__main__':
    main()
