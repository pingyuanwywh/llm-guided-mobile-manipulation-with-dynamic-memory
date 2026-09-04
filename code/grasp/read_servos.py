#!/usr/bin/env python3
# encoding: utf-8
'''读全臂真实舵机位置(走 board_node, 不抢串口)。只读, 不动臂。'''
import rospy
from ros_robot_controller.msg import GetBusServoCmd
from ros_robot_controller.srv import GetBusServoState, GetBusServoStateRequest

rospy.init_node('read_servos', anonymous=True)
srv = rospy.ServiceProxy('/board/bus_servo/get_state', GetBusServoState)
srv.wait_for_service(timeout=10)
out = []
for sid in (1, 2, 3, 4, 5, 10):
    cmd = GetBusServoCmd()
    cmd.id = sid
    cmd.get_position = 1
    try:
        res = srv(GetBusServoStateRequest(cmd=[cmd]))
        # 注意: 返回的 position 只有 1 个元素, 用 position[0]
        pos = int(res.state[0].position[0]) if (res.success and res.state
                                                and len(res.state[0].position) > 0) else None
    except Exception as e:
        pos = 'ERR:%s' % e
    out.append('%s=%s' % (sid, pos))
print('  '.join(out))
print('观察姿势应为 1=500 2=720 3=100 4=120 5=500;  夹爪 10: 张开~63 夹住~250 空夹~418')
