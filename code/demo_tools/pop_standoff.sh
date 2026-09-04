#!/bin/bash
# 本机弹一个终端窗口, ssh 到车上并加载录点别名 (tel / can1..can5 / collect / points / drop / h)。
# 多行 export 直接塞进 ssh 会被压成一行串味, 所以固定写成脚本文件再 bash 它(记忆里的老坑)。
# 交互 shell 加载函数用 --rcfile ~/.standoff_rc; 别用 --rcfile <(cat a b) 进程替换, ssh 下不可靠。
export DISPLAY=:1
export DISABLE_ROS1_EOL_WARNINGS=1
export DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus
[ -f "$HOME/.jetrover_env" ] && . "$HOME/.jetrover_env"   # 现场参数(车IP/本机IP)，见仓库 jetrover_env.example
CAR_IP="${CAR_IP:?未设置 CAR_IP：请配置 ~/.jetrover_env}"

exec gnome-terminal --title="JetRover 建图+录点" -- \
  ssh -t ${CAR_USER:-uavg}@${CAR_IP} "bash --rcfile ~/.standoff_rc"
