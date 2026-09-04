#!/usr/bin/env bash
# cancel_nav.sh -- 打断"空手去接罐子"那段导航 (2026-08-01)。
#
# 这是**唯一能安全打断**的阶段: 手是空的, 没有状态要收拾。
# 逼近/抓取/夹着罐子回站/放置 四个阶段都不能这么干 ——
# 尤其 grab 中途打断会把手臂晾在预抓取姿势伸着(07-21 前科), 下次导航拿它撞东西。
#
# 做两件事: ① 杀掉 nav_goto.py 进程 ② 给 move_base 发 cancel(不然它会继续追目标)。
# ⚠️ set -u 必须放在 source 之后 —— ROS 的 catkin profile
# (/opt/ros/noetic/etc/catkin/profile.d/1.ros_distro.sh) 引用未定义的 ROS_DISTRO,
# set -u 在前会让整个脚本一行都跑不到。07-26 start_nav_ekf.sh 踩过, 08-01 我又犯了一次。
source /opt/ros/noetic/setup.bash
source /home/uavg/ros1_ws/devel/setup.bash
set -u

echo "+ pkill -f [n]av_goto.py"
pkill -f "[n]av_goto.py"

echo "+ rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- {}"
timeout 10 rostopic pub -1 /move_base/cancel actionlib_msgs/GoalID -- '{}' >/dev/null 2>&1

# 车可能还带着最后一条速度指令。cmd_vel_to_motor 有 0.5s watchdog 会自己停,
# 但显式发个 0 更快也更明确。
timeout 5 rostopic pub -1 /cmd_vel geometry_msgs/Twist -- '[0,0,0]' '[0,0,0]' >/dev/null 2>&1
echo "已打断导航"
