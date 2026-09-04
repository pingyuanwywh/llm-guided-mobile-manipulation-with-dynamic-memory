#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
/cmd_vel (geometry_msgs/Twist)  ->  /board/set_motor (麦克纳姆四轮 rps)

把标准 ROS 速度指令翻译成四个轮子的转速，让 move_base / 键盘遥控等
任何发 /cmd_vel 的节点都能驱动小车。

依赖 board_node 运行 (本节点只发 ROS 话题，由 board_node 写串口)。
轮子映射与标定来自 car_control.py 的实测结果。

运行: 先开 board_node，再
    source ~/ros_car/devel/setup.bash
    python3 ~/cmd_vel_to_motor.py
"""
import rospy
from geometry_msgs.msg import Twist
from ros_robot_controller.msg import MotorsState, MotorState

# ---- 标定换算 (2026-06-01 实测) ----
# 线速度: 28.6 cm/(rps·s) -> 0.286 m/s per rps
# 角速度: 82.1 deg/(rps·s) -> 1.433 rad/s per rps
LIN_MS_PER_RPS = 0.286
ROT_RADS_PER_RPS = 1.433

# 轮子: 1=左前 2=左后 3=右前 4=右后; 右侧镜像安装
FWD_SIGN = {1: +1, 2: +1, 3: -1, 4: -1}

CMD_TIMEOUT = 0.5   # 秒: 超时未收到 cmd_vel 就停车(安全)


class CmdVelBridge:
    def __init__(self):
        self.pub = rospy.Publisher('/board/set_motor', MotorsState, queue_size=1)
        self.last_cmd = rospy.Time.now()
        self.stopped = True
        rospy.Subscriber('/cmd_vel', Twist, self._on_cmd, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self._watchdog)

    def _on_cmd(self, msg):
        self.last_cmd = rospy.Time.now()
        self.stopped = False
        f = msg.linear.x / LIN_MS_PER_RPS    # 前进分量
        s = msg.linear.y / LIN_MS_PER_RPS    # 左移分量
        r = msg.angular.z / ROT_RADS_PER_RPS  # 左转(CCW)分量
        self._send(f, s, r)

    def _send(self, f, s, r):
        roll = {
            1: f - s - r,   # 左前 FL
            3: f + s + r,   # 右前 FR
            2: f + s - r,   # 左后 RL
            4: f - s + r,   # 右后 RR
        }
        m = MotorsState()
        m.data = [MotorState(id=i, rps=FWD_SIGN[i] * roll[i]) for i in (1, 2, 3, 4)]
        self.pub.publish(m)

    def _watchdog(self, _evt):
        if not self.stopped and (rospy.Time.now() - self.last_cmd).to_sec() > CMD_TIMEOUT:
            self._send(0.0, 0.0, 0.0)
            self.stopped = True


if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_motor')
    CmdVelBridge()
    rospy.loginfo('cmd_vel_to_motor 桥接已启动: /cmd_vel -> /board/set_motor')
    rospy.spin()
