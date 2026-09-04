#!/bin/bash
# start_nav_ekf.sh -- 起"IMU/EKF 版"导航栈 (2026-07-26)
#
# 和 start_nav.sh 的区别: 位姿链拆成 map->odom(hector) + odom->base(EKF) 两段,
# 车头朝向来自陀螺仪推算的那段平滑变换 => 治 DWA 追噪声的"一扭一扭"。
# 详见 ~/nav_hector_ekf.launch 顶部注释。
#
# ⚠️ 起来的顺序有讲究: imu_debias 要先静止标定 10s, EKF 要先于 hector 出 TF
#    (否则 hector 算 map->odom 时找不到 odom->base, 会刷一堆警告)。
#
# 前置(本脚本不管): board_node / rplidar / cmd_vel_to_motor.py
# 回退: 用回 ~/start_nav.sh, 原 nav_hector.launch 一个字没动过。
# ⚠️ set -u 必须放在 source 之后: ROS 的 catkin profile 脚本
# (/opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh) 会引用未定义的 ROS_DISTRO,
# 在 set -u 下直接报 "unbound variable" 退出, 整个脚本一行都跑不到。
source /opt/ros/noetic/setup.bash
source ~/ros1_ws/devel/setup.bash
set -u

CAL=${CAL:-10}

cleanup() {
  echo "[start_nav_ekf] 收尾: 关掉 imu_debias / cmd_odom"
  [ -n "${PID_IMU:-}" ] && kill "$PID_IMU" 2>/dev/null
  [ -n "${PID_ODO:-}" ] && kill "$PID_ODO" 2>/dev/null
}
trap cleanup EXIT INT TERM

echo "[start_nav_ekf] 1/3 起 imu_debias (静止标定 ${CAL}s, **车别动**) ..."
python3 -u ~/imu_debias.py --cal "$CAL" >/tmp/imu_debias.log 2>&1 &
PID_IMU=$!

echo "[start_nav_ekf] 2/3 起 cmd_odom (指令速度航位推算) ..."
python3 -u ~/cmd_odom.py >/tmp/cmd_odom.log 2>&1 &
PID_ODO=$!

# 等标定跑完再起 EKF/hector: EKF 没有 IMU 输入也能跑(只是没有偏航观测),
# 但让它一开始就拿到干净数据更省事。多等 2s 留余量。
sleep $(python3 -c "print($CAL + 2)")

if ! grep -q "标定完成" /tmp/imu_debias.log; then
  echo "[start_nav_ekf] !! IMU 标定没成功, 看 /tmp/imu_debias.log:"
  tail -5 /tmp/imu_debias.log
  echo "[start_nav_ekf] !! 车停稳后重跑本脚本。仍要继续请 Ctrl-C 后手动 roslaunch。"
  exit 1
fi
grep "标定完成" /tmp/imu_debias.log | tail -1

echo "[start_nav_ekf] 3/3 起 EKF + hector + move_base ..."
exec roslaunch ~/nav_hector_ekf.launch "$@"
