#!/usr/bin/env python3
# encoding: utf-8
# 只算不动: 检查收集区一排落点的 IK 可达性。
# place 对每个落点实际会走三个位姿: pre_place(z+0.05) -> target -> retreat(x-0.07)
# 三个都得有解, 这个落点才真能用。
import sys
import rospy
from hiwonder_kinematics import kinematics_control

rospy.init_node('check_place_row', anonymous=True)

X = float(sys.argv[1]) if len(sys.argv) > 1 else 0.38
Z = float(sys.argv[2]) if len(sys.argv) > 2 else 0.09
PITCH = float(sys.argv[3]) if len(sys.argv) > 3 else 10
YS = [-0.16, -0.08, 0.0, 0.08, 0.16]

print('落点排布检查: x=%.2f z=%.2f pitch=%s' % (X, Z, PITCH))
print('%-8s %-22s %-22s %-22s %s' % ('y', 'pre_place(z+5cm)', 'target', 'retreat(x-7cm)', '结论'))

for i, y in enumerate(YS):
    poses = [
        ('pre_place', [X, y, Z + 0.05]),
        ('target',    [X, y, Z]),
        ('retreat',   [X - 0.07, y, Z]),
    ]
    cells, ok_all = [], True
    for name, p in poses:
        ret = kinematics_control.set_pose_target(p, PITCH)
        ok = len(ret[1]) > 0
        if ok:
            rpy = ret[3]
            cells.append('OK pitch=%.0f' % rpy[1])
        else:
            cells.append('无解')
            ok_all = False
    print('%-8.2f %-22s %-22s %-22s %s' % (
        y, cells[0], cells[1], cells[2], '可用' if ok_all else '<<< 不可用'))
