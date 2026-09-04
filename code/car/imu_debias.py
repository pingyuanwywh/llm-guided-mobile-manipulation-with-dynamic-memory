#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""imu_debias.py -- 给 EKF 喂一路"干净的" IMU (2026-07-26)

/board/imu_raw 不能直接喂给 robot_localization, 有两个坑, 这个节点就是来堵的:

坑1: **orientation 全是 0, 但 orientation_covariance 不是 -1**。
     board_node 只有裸 IMU(无磁力计), 算不出绝对姿态, 所以 orientation 恒为 (0,0,0,0)。
     ROS 的约定是"没有这个量就把协方差第 0 项设成 -1", board_node 没有遵守。
     robot_localization 看到协方差不是 -1, 就会**把那个假的零姿态当成真实测量吃进去**,
     结果是车头被死死拽向 yaw=0 —— 比不接 IMU 还糟。
     => 这里把 orientation_covariance[0] 改成 -1, 明确告诉 EKF "这一路没有姿态, 只有角速度"。

坑2: **陀螺仪有恒定零偏**。2026-07-26 实测这块芯片静止时 z 轴读数不是 0 而是
     -0.094 deg/s, 不补的话积分出来 -5.6 度/分钟。零偏随温度变, 所以**不能硬编码**,
     每次起栈现测。 => 启动时静止采 N 秒求平均, 之后每帧减掉。

坑3: **这颗 IMU 是 z 轴朝下装的, 符号和 ROS 约定相反**(2026-07-26 实测坐实)。
     三条独立证据:
       a) 静止时 linear_acceleration.z = **-9.78**。ROS(REP-103)是 z 轴朝上的右手系,
          静止的加速度计在自己坐标系里应该读 **+9.8**。**重力不是约定**, 读到 -g
          就说明这颗芯片的 z 轴物理上朝下。这条是决定性的, 不依赖任何其他子系统。
       b) 发 angular.z=+0.15(ROS 约定: 正=逆时针) 转 3 秒, hector 报 +20.4 度(方向对),
          陀螺仪报 **-21.4 度**(方向反, 大小几乎一样)。
       c) 车走直线 0.48m: 陀螺仪 +1.86 / hector -1.74; 倒回来: 陀螺仪 -1.83 / hector +1.81。
          hector 恒等于陀螺仪的相反数 —— 干净的镜像, 不是噪声该有的样子。
     z 轴朝下 = 绕 z 的转动符号相反。**不取反就喂 EKF, 朝向会朝错误方向走, 比不接 IMU 更糟。**
     => --flip(默认开) 按"绕 x 轴转 180 度"把 y/z 取反(x 不变)。
     ⚠️ 到底是绕 x 还是绕 y 转的 180 度, 光靠水平静置的数据分不出来(两者都让 z 翻转),
        但我们只用 z, 且 two_d_mode 下 roll/pitch 角速度根本不参与, 所以这个歧义无所谓。

启动时会自检"车真的没动"(角速度标准差 + 加速度模长稳定), 检出在动就**拒绝标定**、
改用 ~/imu_bias.yaml 里上次的值并告警 —— 免得在车晃着的时候把真实转动学成零偏。

用法: python3 ~/imu_debias.py [--cal 10] [--in /board/imu_raw] [--out /imu/data]
零偏落盘 ~/imu_bias.yaml (下次启动时若不能标定就读它)
"""
import os
import math
import argparse

import rospy
import yaml
from sensor_msgs.msg import Imu

BIAS_FILE = os.path.expanduser('~/imu_bias.yaml')

# 判"车没动"的门限。2026-07-26 实测静止时 gyro z 的 std = 0.00071 rad/s,
# 所以 0.01 rad/s(=0.57 deg/s)留了十几倍余量, 但又远小于人手推车的量级(实测峰值 1.12 rad/s)。
STILL_GYRO_STD = 0.01      # rad/s
STILL_ACC_STD = 0.20       # m/s^2, 实测静止 0.0018


class Vec(object):
    """轻量三元组, 只为让 to_base() 的返回值能像 msg.angular_velocity 一样用 .x/.y/.z"""
    __slots__ = ('x', 'y', 'z')

    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


def stats(v):
    n = len(v)
    if n == 0:
        return 0.0, 0.0
    m = sum(v) / n
    sd = math.sqrt(sum((x - m) ** 2 for x in v) / n) if n > 1 else 0.0
    return m, sd


class ImuDebias:
    def __init__(self, args):
        self.args = args
        self.calibrating = True
        self.buf = []                     # [(gx,gy,gz,|a|)]
        self.bias = (0.0, 0.0, 0.0)
        self.gyro_var = args.gyro_var
        self.n_out = 0
        self.pub = rospy.Publisher(args.out, Imu, queue_size=20)
        rospy.Subscriber(args.inp, Imu, self.cb, queue_size=50)
        self.t0 = rospy.Time.now()

    # ---------- 坐标系 ----------
    def to_base(self, msg):
        '''把 IMU 自身坐标系的读数转到车体系(base_footprint)。

        坑3: 这颗 IMU z 轴朝下装 => 相当于绕 x 轴转了 180 度 => y 和 z 都取反, x 不变。
        标定在**转换之后**做, 所以存盘的零偏也是车体系的, 前后自洽。
        '''
        g, a = msg.angular_velocity, msg.linear_acceleration
        if self.args.flip:
            return (Vec(g.x, -g.y, -g.z), Vec(a.x, -a.y, -a.z))
        return (Vec(g.x, g.y, g.z), Vec(a.x, a.y, a.z))

    # ---------- 标定 ----------
    def finish_cal(self):
        gx = [b[0] for b in self.buf]
        gy = [b[1] for b in self.buf]
        gz = [b[2] for b in self.buf]
        aa = [b[3] for b in self.buf]
        mx, sx = stats(gx)
        my, sy = stats(gy)
        mz, sz = stats(gz)
        ma, sa = stats(aa)
        still = (max(sx, sy, sz) < STILL_GYRO_STD) and (sa < STILL_ACC_STD)

        if still:
            self.bias = (mx, my, mz)
            # 噪声方差直接从实测来, 但放大 10 倍留余量(白噪声之外还有零偏不稳定性),
            # 并设一个下限 —— 协方差给太小 = EKF 过度自信, 反而不肯听 hector 的修正。
            self.gyro_var = max(sz * sz * 10.0, 1e-6)
            try:
                with open(BIAS_FILE, 'w') as f:
                    yaml.safe_dump({'gyro_bias': [float(mx), float(my), float(mz)],
                                    'gyro_var': float(self.gyro_var),
                                    'stamp': rospy.get_time(),
                                    'samples': len(self.buf)}, f)
            except Exception as e:
                rospy.logwarn('imu_debias: 零偏写盘失败: %s', e)
            rospy.loginfo('imu_debias: 标定完成, %d 帧, bias_z = %+.5f rad/s (%+.3f deg/s), '
                          'noise std %.5f -> var %.2e',
                          len(self.buf), mz, math.degrees(mz), sz, self.gyro_var)
        else:
            rospy.logerr('imu_debias: **标定期间车在动** (gyro std %.4f / acc std %.3f), '
                         '拒绝标定', max(sx, sy, sz), sa)
            loaded = self.load_bias()
            if loaded:
                rospy.logwarn('imu_debias: 改用 %s 里上次的零偏 %+.5f rad/s',
                              BIAS_FILE, self.bias[2])
            else:
                rospy.logerr('imu_debias: 也没有历史零偏可用 => 零偏按 0 处理, '
                             '朝向会以约 %.1f deg/min 漂。请把车停稳后重启本节点。',
                             abs(math.degrees(mz)) * 60)
        self.calibrating = False
        self.buf = []

    def load_bias(self):
        try:
            with open(BIAS_FILE) as f:
                d = yaml.safe_load(f)
            b = d['gyro_bias']
            self.bias = (float(b[0]), float(b[1]), float(b[2]))
            self.gyro_var = float(d.get('gyro_var', self.args.gyro_var))
            return True
        except Exception:
            return False

    # ---------- 主回调 ----------
    def cb(self, msg):
        g, a = self.to_base(msg)
        if self.calibrating:
            self.buf.append((g.x, g.y, g.z,
                             math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)))
            if (rospy.Time.now() - self.t0).to_sec() >= self.args.cal:
                self.finish_cal()
            return

        out = Imu()
        out.header = msg.header
        if not out.header.frame_id:
            out.header.frame_id = self.args.frame
        else:
            out.header.frame_id = self.args.frame

        # --- 坑1: 明确声明"本消息没有姿态" ---
        # -1 是 ROS 的约定值, robot_localization 见到就会整段忽略 orientation。
        out.orientation = msg.orientation
        out.orientation_covariance = [-1.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0,
                                      0.0, 0.0, 0.0]

        # --- 坑2: 减掉零偏 ---
        out.angular_velocity.x = g.x - self.bias[0]
        out.angular_velocity.y = g.y - self.bias[1]
        out.angular_velocity.z = g.z - self.bias[2]
        # 只有 z(偏航)会被 EKF 采用, x/y 给大协方差表示"别太当真"
        out.angular_velocity_covariance = [self.gyro_var * 100, 0.0, 0.0,
                                           0.0, self.gyro_var * 100, 0.0,
                                           0.0, 0.0, self.gyro_var]

        # 加速度原样转发。EKF 配置里没有启用任何加速度分量 ——
        # 没有轮编码器时用加速度双积分算位移是出了名的垃圾(几秒漂几米),
        # 位移那一维交给 cmd_odom.py 的指令速度航位推算。
        # 逐分量赋值 —— a 是 to_base() 返回的轻量 Vec, 不是 geometry_msgs/Vector3,
        # 整个赋过去会在序列化时炸。
        out.linear_acceleration.x = a.x
        out.linear_acceleration.y = a.y
        out.linear_acceleration.z = a.z
        out.linear_acceleration_covariance = [0.05, 0.0, 0.0,
                                              0.0, 0.05, 0.0,
                                              0.0, 0.0, 0.05]
        self.pub.publish(out)
        self.n_out += 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--cal', type=float, default=10.0, help='启动时静止标定秒数')
    p.add_argument('--in', dest='inp', default='/board/imu_raw')
    p.add_argument('--out', default='/imu/data')
    p.add_argument('--frame', default='base_footprint',
                   help='输出消息的 frame_id。IMU 芯片装在板子上、和车体固连, '
                        '又没有单独发 base_footprint->imu_link 的静态TF, 所以直接标成车体系。')
    p.add_argument('--gyro-var', dest='gyro_var', type=float, default=1e-5,
                   help='标定失败时的兜底角速度方差 (rad/s)^2')
    p.add_argument('--no-flip', dest='flip', action='store_false', default=True,
                   help='别把 y/z 取反。默认是取反的 —— 这颗 IMU z 轴朝下装(见坑3)。'
                        '换了 IMU 或板子固件改了朝向, 用 drive_check.py --wz 0.15 重新判一次符号。')
    args = p.parse_args(rospy.myargv()[1:])

    rospy.init_node('imu_debias')
    rospy.loginfo('imu_debias: %s -> %s, 静止标定 %.0fs, 车别动...',
                  args.inp, args.out, args.cal)
    ImuDebias(args)
    rospy.spin()


if __name__ == '__main__':
    main()
