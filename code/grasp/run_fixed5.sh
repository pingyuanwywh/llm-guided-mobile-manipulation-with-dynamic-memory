#!/usr/bin/env bash
# run_fixed5.sh -- 五罐闭环, 顺序和送站**写死** (2026-08-01)
#
# 为什么写死: LLM 规划层这天查出多个缺陷(选站串题 / trip 记账对不上), 短期内不可信。
# 把它从执行路径里摘出去, 先拿一次能跑通的示例。规划的事之后单独修。
#
# 顺序和送站不是拍脑袋, 是 plan_clearance.py 用 move_base 真实路径实测定的(2026-08-01 晚场布局):
#   全罐在场时: can1 ↔A 1.28 / ↔B **0.01 必撞**  => can1 只能 A
#               can4 ↔A 0.29 挡下 / ↔B 1.15      => can4 只能 B
#               can3 ↔A 0.93 / ↔B 0.49 挡下      => can3 走 A
#               can2 / can5 两边都可走, 按距离: can2→A(1.50 vs 2.02), can5→B(1.15 vs 2.98)
#   => can1/can2/can3 送 A, can4/can5 送 B (几何强制的和按距离选的完全一致)
# 顺序 can4→can5→can2→can1→can3:
#   ① 录完点车就停在 can4 的 standoff, 第一条腿路径只有 0.01m, 白捡
#   ② can4/can5 都在 B 侧先收完, 再一次性横穿到 A 侧, 只跨场地一趟
#   实测(每条腿只对当时还在场上的罐子算): **全程最小净空 0.83m**, 导航总路程 10.9m
#
# ⚠️ 用的是**手工录的 7 个点**, 不经过 set_place.py, 不经过 LLM。
set -u

mkdir -p ~/Log
LOG=~/Log/run_fixed5_$(date +%Y%m%d_%H%M%S).log

# --record TAG : 车载 RGB+深度 两路的开录/停录**都在脚本里**, 不靠人记得。
# ⚠️ 08-01 两次栽在这上面: ①成功那趟忘了开录, 5/5 一帧画面没有;
#    ②崩溃时忘了停录, record_cam 一直开到断电, H.264 没写完 moov 原子 => 两路都打不开。
# trap 覆盖正常结束 / 出错 / 被 Ctrl-C 或 SIGTERM 打断, 保证 mp4 一定收尾。
# (rviz 那一路在本机, 不归这个脚本管; 天花板和手机是用户自己开。)
REC=""
if [ "${1:-}" = "--record" ]; then
  REC="${2:?--record 后面要跟一个 tag}"; shift 2
  echo "+ bash ~/rec.sh start $REC"
  bash ~/rec.sh start "$REC"
  trap 'echo "--- 停止车载录制(trap) ---"; bash ~/rec.sh stop' EXIT INT TERM
fi

# 铁律: 导航前必杀 teleop(它持续灌 /cmd_vel, 会和 move_base 抢总线, 07-25 车速砍半)
pkill -f "[t]eleop.py" && echo "已杀 teleop" || true

# ⚠️ 全程 tee 落盘 —— 08-01 车机崩溃那次日志随 tmux 一起没了, 根因至今查不出来。
{
echo "===================== 开始 $(date +%H:%M:%S) ====================="
# 不带参数 = 跑全部五罐; 带参数 = 只跑指定的那几条(板子中途挂掉后补跑用)
#   bash ~/run_fixed5.sh "can2 green collect_a" "can1 red collect_a"
# 带录制: bash ~/run_fixed5.sh --record demo2st          (五罐全跑 + 车载两路自动开停)
DEFAULT=("can4 red collect_b" "can5 red collect_b" "can2 green collect_a"
         "can1 red collect_a" "can3 red collect_a")
if [ $# -gt 0 ]; then PLAN=("$@"); else PLAN=("${DEFAULT[@]}"); fi
for spec in "${PLAN[@]}"; do
  set -- $spec
  bash ~/run_one_can.sh "$1" "$2" "$3"
  echo "###### $1 -> $3 退出码 $?  (0=收进站 2=导航败 3=没抓住 4=夹着罐回不去 5=place败)"
done
echo "===================== ALL DONE $(date +%H:%M:%S) ====================="
} 2>&1 | tee "$LOG"
echo
echo "日志: $LOG"
