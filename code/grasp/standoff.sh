#!/bin/bash
# standoff.sh -- 录一个罐子的 standoff 点 (2026-07-28 第三版, 用户拍板的形态)
#
# 用法(一般通过别名调用, 见 standoff_cmds.sh): bash ~/standoff.sh can1 green
#      collect 这种不用检测的:                  bash ~/standoff.sh collect
#
# ## 设计约束(来自 07-28 的教训, 别再违反)
# 这个脚本**只做透明串联** —— 把 07-23 就在真机上验证过的三条命令按顺序执行, 每条**原样打印再跑**,
# 任何一条失败就停在那里、把工具自己的报错留在屏幕上。
# **不加任何自检门/甜点区判据/乐观提示**。第二版加了那些, 结果:
#   - 自作主张要求人肉眼摆 35cm(而人眼判不准横向, 实测偏 12~15cm)
#   - 解析逻辑塞在 heredoc 里被引号撞出 SyntaxError, 五次录点全部静默失败
#   - 屏幕上却先打印了"开录", 具有误导性
# 详见 feedback_use_existing_commands 记忆。
#
# 唯一保留的"额外动作"是最后**回读一次已录点** —— 那是核对, 不是判断, 正是为了让静默失败无处可藏。

NAME="$1"
COLOR="$2"

if [ -z "$NAME" ]; then
  echo "用法: bash ~/standoff.sh <点名> [颜色]"
  exit 1
fi

NAV="source /opt/ros/noetic/setup.bash"
GRASP="source /opt/ros/noetic/setup.bash; source /home/uavg/ros_car/devel/setup.bash; source /home/uavg/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash"
# nav 和 grasp 的环境必须分开 source, 混着会把包路径互相挤掉(run_demo.sh 的老教训)

readback() {
  echo "--- 回读确认 ---"
  bash -c "$NAV; python3 - <<'PY'
import os, math, yaml
f = os.path.expanduser('~/llm_nav_places.yaml')
d = (yaml.safe_load(open(f)) or {}).get('places', {}) if os.path.exists(f) else {}
print('已录 %d 个: %s' % (len(d), ', '.join(sorted(d)) or '(空)'))
need = ['can%d' % i for i in range(1, int(os.environ.get('N_CANS', 5)) + 1)]
need += ['collect_a','collect_b'] if os.environ.get('TWO_STATION') else ['collect']
miss = [n for n in need if n not in d]
print('还差: ' + (', '.join(miss) if miss else '无, %d 个点齐了' % len(need)))
PY"
}

# 录点期间 teleop 必须停 —— 它持续往 /cmd_vel 灌指令, 会和 approach 抢总线(07-25 老坑)
if pgrep -f "[t]eleop.py" >/dev/null; then
  echo "+ pkill -f teleop.py        # 它会和逼近抢 /cmd_vel"
  pkill -f "[t]eleop.py"
  sleep 1
fi

if [ -z "$COLOR" ]; then
  echo "+ python3 ~/record_place_once.py $NAME"
  bash -c "$NAV; python3 /home/uavg/record_place_once.py $NAME" || exit 1
  readback
  exit 0
fi

echo "+ rosservice call /track_and_grab/set_color \"data: $COLOR\""
bash -c "$GRASP; rosservice call /track_and_grab/set_color \"data: $COLOR\"" || exit 1

echo "+ python3 ~/approach.py --key x --target 0.35 --search-max 0.22"
bash -c "$GRASP; python3 -u /home/uavg/approach.py --key x --target 0.35 --search-max 0.22"
RC=$?
if [ $RC -ne 0 ]; then
  echo "^^ 逼近没成功(码 $RC), 没有录点。上面那行 FAIL 是 approach 自己报的原因。"
  echo "   继续开车: tel"
  exit $RC
fi

echo "+ python3 ~/approach.py --nudge -0.08 0"
bash -c "$GRASP; python3 -u /home/uavg/approach.py --nudge -0.08 0" || exit 1

echo "+ python3 ~/record_place_once.py $NAME"
bash -c "$NAV; python3 /home/uavg/record_place_once.py $NAME" || exit 1

readback
echo "继续开车: tel"
