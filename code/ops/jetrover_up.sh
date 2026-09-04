#!/bin/bash
# jetrover_up.sh -- 【本机】开工一键起栈 + 逐项体检。2026-08-27 从当天真正跑通的步骤固化而来。
#
#   bash ~/jetrover_up.sh              # 默认红罐做抓取体检
#   COLOR=green bash ~/jetrover_up.sh  # 车前摆的是绿罐
#   CAR_IP=192.168.1.99 bash ~/jetrover_up.sh   # DHCP 变了
#
# ⚠️ 本脚本**不会让车移动**。抓取体检只读(读舵机/measure/capture_one), 真抓要自己敲。
# ⚠️ 每一步都有判据, 不过就停 —— 省掉"跑完一整趟才发现白跑"。

set -u
[ -f "$HOME/.jetrover_env" ] && . "$HOME/.jetrover_env"   # 现场参数(车IP/本机IP)，见仓库 jetrover_env.example
CAR_IP="${CAR_IP:?未设置 CAR_IP：请配置 ~/.jetrover_env（参考仓库 jetrover_env.example）}"
MY_IP="${MY_IP:?未设置 MY_IP：请配置 ~/.jetrover_env（参考仓库 jetrover_env.example）}"
COLOR="${COLOR:-red}"
SSH="ssh -o ConnectTimeout=10 ${CAR_USER:-uavg}@${CAR_IP}"
R="source /opt/ros/noetic/setup.bash"
GRASP="export MACHINE_TYPE=JetRover_Mecanum; $R; source ~/ros_car/devel/setup.bash; source ~/JetRover-Jetson_nano_ros1/ros_ws/devel/setup.bash"

ok(){ echo -e "  \033[32m✅ $*\033[0m"; }
bad(){ echo -e "  \033[31m❌ $*\033[0m"; }
warn(){ echo -e "  \033[33m⚠️  $*\033[0m"; }
step(){ echo; echo -e "\033[1m=== $* ===\033[0m"; }
die(){ bad "$*"; echo; echo "停在这一步。修好再跑一次 bash ~/jetrover_up.sh"; exit 1; }

step "0/7 连车  (${CAR_IP})"
ping -c 2 -W 2 "$CAR_IP" >/dev/null 2>&1 || die "ping 不通。车开了吗? IP 变了就 CAR_IP=... 重跑"
H=$($SSH 'echo $(hostname)' 2>/dev/null) || die "ssh 连不上"
ok "$H 在线, 本机 $MY_IP"
grep -q "^${CAR_IP} j40" /etc/hosts && ok "/etc/hosts 的 j40 指对了" \
  || warn "/etc/hosts 里 j40 不是 ${CAR_IP} —— 跨机 rviz 会收不到 /scan /map (要 sudo 改)"

step "1/7 抓取栈"
$SSH "bash ~/JetRover-Jetson_nano_ros1/start_final_grab.sh ${COLOR} 10 30" 2>&1 | tail -3
$SSH "$R; timeout 10 rosservice list 2>/dev/null | grep -c track_and_grab" | grep -qE '[1-9]' \
  && ok "track_and_grab 服务就绪 (目标色 ${COLOR})" || die "抓取栈没起来"

step "2/7 cmd_vel 桥接  (start_final_grab.sh 不含它, 少了逼近环平移全打空)"
$SSH "tmux has-session -t cmd_vel 2>/dev/null || tmux new-session -d -s cmd_vel \"$R; source ~/ros_car/devel/setup.bash; python3 ~/cmd_vel_to_motor.py\"; sleep 4"
$SSH "$R; timeout 10 rostopic info /cmd_vel 2>&1 | grep -c cmd_vel_to_motor" | grep -qE '[1-9]' \
  && ok "/cmd_vel 有订阅者" || die "cmd_vel_to_motor 没起来"

step "3/7 抓取体检  (只读, 车不动)"
$SSH "$GRASP; timeout 25 python3 ~/read_servos.py" 2>&1 | head -1
echo "     判据: 观察姿势 1=500 2=720 3=93~100 4=120 5=500 / 夹爪 63=张开"
M=$($SSH "$GRASP; timeout 30 rosservice call /track_and_grab/measure '{}'" 2>&1)
echo "$M" | grep -o '\\"x\\": [0-9.]*\|\\"z\\": [0-9.]*\|\\"dist\\": [0-9.]*' | tr '\n' ' '; echo
echo "     判据: x 落在 0.30~0.38 甜点区; z 只是相机位姿的仪表, 真判据是抓一次看 detected z"
$SSH "$GRASP; timeout 30 python3 ~/capture_one.py ${COLOR}" 2>&1 | grep -E "contours|FINAL"
echo "     判据: contours=1 单块 / bbox 宽高 ≈50~60 x 85~105 / 框心横偏 <10px"
echo "     (只圈到 20x22 = 光不够, 先补光; 08-26 夜间户外整场废在这)"

step "4/7 雷达"
$SSH "tmux has-session -t lidar 2>/dev/null || tmux new-session -d -s lidar 'bash ~/start_lidar.sh'; sleep 9"
HZ=$($SSH "$R; timeout 12 rostopic hz /scan 2>&1 | grep -o 'average rate: [0-9.]*' | tail -1")
if [ -z "$HZ" ]; then
  bad "/scan 没有消息"
  echo "     >>> 去物理拨雷达开关 OFF -> ON <<<  (充电后必现; 重启节点无效, 只有拨开关能救)"
  echo "     拨完重跑: bash ~/jetrover_up.sh"
  exit 1
fi
ok "$HZ  (健康 ≈13.5)"
$SSH "$R; timeout 10 rostopic echo -n1 /scan/scan_time 2>&1 | head -1" | \
  awk '{printf "  scan_time=%s (健康 ≈0.074)\n", $1}'

step "5/7 导航栈 (TEB + 麦轮横移)"
$SSH "tmux has-session -t nav 2>/dev/null || tmux new-session -d -s nav \"$R; source ~/ros1_ws/devel/setup.bash; roslaunch --wait ~/nav_teb_holo.launch\"; sleep 16"
N=$($SSH "$R; timeout 10 rosnode list 2>/dev/null | grep -cE 'hector_mapping|move_base'")
[ "$N" -ge 2 ] && ok "hector_mapping + move_base 都在" || die "导航栈没起全"
for p in base_local_planner TebLocalPlannerROS/max_vel_y TebLocalPlannerROS/weight_kinematics_nh TebLocalPlannerROS/max_vel_theta; do
  V=$($SSH "$R; timeout 8 rosparam get /move_base/$p" 2>/dev/null)
  echo "  $p = $V"
done
echo "     判据: teb_local_planner/TebLocalPlannerROS / 0.1 / 1.0 / 0.25"
$SSH "$R; timeout 10 rosrun tf tf_echo map base_footprint 2>&1 | grep -m1 Translation" \
  | sed 's/^/  地图原点(车当前位置) /'

step "6/7 清旧航点  (旧点在新图上是乱坐标, 不清会静默导航到错地方)"
$SSH "cat ~/llm_nav_places.yaml" | head -3
$SSH "cp ~/llm_nav_places.yaml ~/llm_nav_places.yaml.bak_\$(date +%m%d_%H%M) && printf 'places: {}\n' > ~/llm_nav_places.yaml && echo cleared" >/dev/null \
  && ok "已备份并清空 (备份名 llm_nav_places.yaml.bak_月日_时分)"

step "7/7 弹窗  (rviz 看建图 + 建图录点窗)"
pkill -f "[r]viz -d $HOME/.rviz/jetrover_nav.rviz" 2>/dev/null; sleep 1
nohup bash ~/start_rviz_map.sh >/tmp/rviz_map.log 2>&1 & sleep 10
pgrep -f "[r]viz -d" >/dev/null && ok "rviz 起来了" || warn "rviz 没起来, 看 /tmp/rviz_map.log"
nohup bash ~/pop_standoff.sh >/tmp/standoff.log 2>&1 & sleep 6
pgrep -f "[s]sh -t uavg" >/dev/null && ok "「JetRover 建图+录点」窗弹出来了" || warn "录点窗没弹出来"

cat <<'TIP'

========================================================================
全部就位。接下来在那个窗里:
  tel                遥控建图 (w前 s后 a左 d右 x停 q退)
  can1 / can1 green  开到罐子跟前录点 (红罐直接敲, 绿罐跟 green)
  collect_a / collect_b   两个站; 起点那个站**最后录**
  points             看还差哪些 / drop can3  删错的

建图纪律: 人别进雷达圈 / 别反复原地转 / 电梯门那种会变的墙别当主要参照
建完图先别录点 —— 让 Claude 跑一次 align_check.py 给图体检(车不动 10 秒)
========================================================================
TIP
