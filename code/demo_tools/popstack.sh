#!/bin/bash
# 一键弹 4 终端启动 JetRover 导航栈 (board->lidar->cmd_vel_to_motor->nav)
# 延时放在各窗口内部, 不阻塞调用方; 每窗口 ssh 进车跑对应组件。
[ -f "$HOME/.jetrover_env" ] && . "$HOME/.jetrover_env"   # 现场参数(车IP/本机IP)，见仓库 jetrover_env.example
CAR_IP="${CAR_IP:?未设置 CAR_IP：请配置 ~/.jetrover_env}"
CAR="${CAR_USER:-uavg}@${CAR_IP}"
POP=$HOME/pop.sh

bash "$POP" "1-board(master)" "ssh -t $CAR '~/start_board.sh'"
bash "$POP" "2-lidar" "sleep 5; ssh -t $CAR '~/start_lidar.sh'"
bash "$POP" "3-cmd_vel_to_motor" "sleep 9; ssh -t $CAR 'source /opt/ros/noetic/setup.bash && source ~/ros_car/devel/setup.bash && exec python ~/cmd_vel_to_motor.py'"
bash "$POP" "4-nav(hector+move_base)" "sleep 14; ssh -t $CAR '~/start_nav.sh'"

echo "已发起 4 个弹窗 (board / lidar / motor / nav)。"
