#!/usr/bin/env bash
# run_one_can.sh -- 收一个罐子。从 run_collect5.sh 的 run_can 函数**原样抽出来**(2026-08-01)。
#
# ## 相对 run_collect5.sh 只有三处改动, 每处都不碰已验证的命令行
#  ① 收集站变成参数(原来写死 collect) —— 双站实验要用。
#  ② 加退出码, 让上层 planner 知道到底是哪一步败的(原来只 echo 然后 return)。
#  ③ 加 --phase, 把"空手去接罐子的导航"和"抓+送"拆开 ——
#     那是**唯一能安全打断**的阶段(手是空的, actionlib cancel 一下车停住就完事)。
#     不传 --phase 就是 all = 和原来逐字等价的完整流程。
# ⚠️ 里面每一条 nav_goto / approach / read_servos / place 的参数**一个字都没改**。
#
# 用法:
#   bash ~/run_one_can.sh can3 green collect_b            # 完整一趟
#   bash ~/run_one_can.sh can3 green collect_b --phase nav    # 只开过去(可被打断)
#   bash ~/run_one_can.sh can3 green collect_b --phase rest   # 接着抓+送(不可打断)
#
# 退出码: 0=收进站了 / 2=去罐子导航失败 / 3=没抓住 / 4=回站导航失败(手里还夹着罐子!)
#         5=place 失败 / 10=--phase nav 正常走完
set -u

NAME="${1:?用法: run_one_can.sh NAME COLOR [STATION] [--phase nav|rest|all]}"
COLOR="${2:?缺颜色}"
STATION="${3:-collect}"
PHASE="all"
[ "${4:-}" = "--phase" ] && PHASE="${5:-all}"

NAV="source /opt/ros/noetic/setup.bash; source /home/uavg/ros1_ws/devel/setup.bash"
GRASP="export MACHINE_TYPE=JetRover_Mecanum; source /opt/ros/noetic/setup.bash; source /home/uavg/ros_car/devel/setup.bash; source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash"

echo "===================== $NAME ($COLOR) -> $STATION  phase=$PHASE  $(date +%H:%M:%S) ====================="

# 矮盒落点。幂等, 每趟设一次不花什么钱。
bash -c "$GRASP; rosparam set /track_and_grab/place_pose '[0.32, 0.0, 0.13]'"

if [ "$PHASE" = "all" ] || [ "$PHASE" = "nav" ]; then
  bash -c "$GRASP; rosservice call /track_and_grab/set_color \"data: $COLOR\"" >/dev/null 2>&1
  echo "--- nav to $NAME ---"
  bash -c "$NAV; python3 -u /home/uavg/nav_goto.py --place $NAME --face --thresh 181"
  if [ $? -ne 0 ]; then echo "!! $NAME nav FAILED"; exit 2; fi
  [ "$PHASE" = "nav" ] && { echo "--- 到位, 等下一步 ---"; exit 10; }
fi

echo "--- approach + grab $NAME ---"
bash -c "$GRASP; python3 -u /home/uavg/approach.py --then-grab --key x --target 0.35 --search-max 0.22 --creep-max 0.15"

# ⚠️ timeout 必须有: read_servos 走 board_node topic, 而官方 board.py 的
#    bus_servo_read_and_unpack 用 queue.get(block=True) **无超时**, 板子通信一错位就永久阻塞。
#    08-01 实测: 板子跑到一半自己挂 -> read_servos 永不返回 -> 整场任务挂死在这一行。
#    加了 timeout 之后最坏情况是"读不到当没抓上、跳下一个", 而不是全场卡死。
G=$(timeout 25 bash -c "$GRASP; python3 /home/uavg/read_servos.py" 2>/dev/null | grep -oE '10=[0-9]+' | head -1 | cut -d= -f2)
[ -z "$G" ] && echo "!! 夹爪读不到(25s 超时) —— 多半是板子通信又挂了, 见 project_jetrover_board_comm"
echo "--- gripper=$G (夹住罐~250 / 空~418 / 张开~63) ---"
if [ -z "$G" ] || [ "$G" -le 150 ] || [ "$G" -ge 360 ]; then
  echo "!! $NAME NOT grabbed (g=$G)"
  exit 3
fi

echo "--- grabbed, nav to $STATION ---"
bash -c "$NAV; python3 -u /home/uavg/nav_goto.py --place $STATION --face --thresh 181"
if [ $? -ne 0 ]; then echo "!! $STATION nav FAILED (still holding $NAME)"; exit 4; fi

echo "--- place into box ---"
bash -c "$GRASP; rosservice call /track_and_grab/place \"{}\"" || exit 5
echo "=== $NAME done -> $STATION  $(date +%H:%M:%S) ==="
