#!/bin/bash
# 【本机】把 rviz 跑在**虚拟屏**里(Xvfb)，专门给录 demo 视频用。
# 为什么要虚拟屏: x11grab 抓的是屏幕上那块区域, 真桌面上任何窗口盖上去都会被录进去
# (2026-07-29 首次试拍就被终端窗口盖了)。关进 :99 之后既不占你桌面, 也绝不会被遮。
# 虚拟屏 1440x960 比 rviz 窗口(1280x810 @ +100+100)大一圈, 否则抓取区域会越界。
#
#   bash ~/start_rviz_virtual.sh          起虚拟屏 + rviz
#   bash ~/start_rviz_virtual.sh stop     关掉
#   看一眼里面长什么样: ffmpeg -f x11grab -video_size 1280x810 -i :99 -frames:v 1 -y /tmp/peek.png
set -u
VD="${VD:-:99}"
GEO="${GEO:-1440x960x24}"
[ -f "$HOME/.jetrover_env" ] && . "$HOME/.jetrover_env"   # 现场参数(车IP/本机IP)，见仓库 jetrover_env.example
CAR_IP="${CAR_IP:?未设置 CAR_IP：请配置 ~/.jetrover_env（参考仓库 jetrover_env.example）}"
MY_IP="${MY_IP:?未设置 MY_IP：请配置 ~/.jetrover_env（参考仓库 jetrover_env.example）}"

if [ "${1:-start}" = "stop" ]; then
  pkill -f "[r]viz -d $HOME/.rviz/jetrover_demo.rviz"
  pkill -f "[X]vfb $VD"
  echo "已关闭虚拟屏 $VD 和里面的 rviz"
  exit 0
fi

if ! pgrep -f "[X]vfb $VD" > /dev/null; then
  echo "+ Xvfb $VD -screen 0 $GEO"
  setsid nohup Xvfb "$VD" -screen 0 "$GEO" > /tmp/xvfb.log 2>&1 < /dev/null &
  sleep 2
fi

echo "+ DISPLAY=$VD rviz -d ~/.rviz/jetrover_demo.rviz"
cat > /tmp/_rviz_virtual_env.sh <<EOF
export DISPLAY=$VD
export LIBGL_ALWAYS_SOFTWARE=1
export DISABLE_ROS1_EOL_WARNINGS=1
export ROS_MASTER_URI=http://$CAR_IP:11311
export ROS_IP=$MY_IP
source /opt/ros/noetic/setup.bash
exec rviz -d $HOME/.rviz/jetrover_demo.rviz
EOF
setsid nohup bash /tmp/_rviz_virtual_env.sh > /tmp/rviz_virtual.log 2>&1 < /dev/null &
sleep 12
DISPLAY="$VD" xwininfo -root -tree 2>/dev/null | grep -E '"[^"]*RViz[^"]*"' \
  && echo "rviz 已在虚拟屏 $VD 里跑起来" \
  || { echo "!! 没起来, 日志:"; tail -15 /tmp/rviz_virtual.log; }
