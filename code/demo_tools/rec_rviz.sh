#!/bin/bash
# 【本机跑, 不是车上】录 rviz 窗口 = demo 视频的"上帝视角"(地图/全局路径/雷达点/车位姿)。
# 用 ffmpeg x11grab 抓窗口那块屏幕区域, h264_nvenc 编码(走 3090 的硬件编码器, 几乎不占 CPU)。
#   bash ~/rec_rviz.sh start [tag]   开录, 自动找 rviz 窗口的位置和大小
#   bash ~/rec_rviz.sh stop          停录
#   bash ~/rec_rviz.sh status
# ⚠️ x11grab 抓的是屏幕上那块区域: 录制期间别把别的窗口盖在 rviz 上, 别挪动/改大小 rviz 窗口。
# ⚠️ 起录时会打印墙上时钟起始时刻, 和车上 record_cam 的 _meta.json 对齐用。

set -o pipefail
# 默认抓**虚拟屏 :99**(`start_rviz_virtual.sh` 起的那个) —— 真桌面上任何窗口盖住 rviz 都会被录进去,
# 虚拟屏没这个问题。要抓真桌面: DISPLAY_OVERRIDE=:1 bash ~/rec_rviz.sh start
export DISPLAY="${DISPLAY_OVERRIDE:-:99}"
OUT_DIR="${OUT_DIR:-$HOME/jetrover_demo}"
FPS="${FPS:-15}"
PIDFILE=/tmp/rec_rviz.pid

find_geom() {
  # 输出 "W H X Y"(屏幕绝对坐标); 找不到 rviz 窗口就返回非 0。
  # ⚠️ xwininfo -tree 每行的第一个几何量是"相对父窗口"的, 不能直接用;
  #    rviz 还有一堆同名子窗口 => 先按面积挑出最大的那个窗口 id, 再单独查它的绝对坐标。
  local id
  # 只认标题正好是 "rviz" 的子窗口 = 3D 渲染区(不含顶上工具栏和底下状态栏);
  # 顶层窗口标题是 "xxx.rviz - RViz", 会被这条过滤掉。
  id=$(xwininfo -root -tree 2>/dev/null | grep -E '"rviz": \("rviz" "rviz"\)' \
       | awk '{for(i=1;i<=NF;i++) if($i ~ /^[0-9]+x[0-9]+\+/){split($i,a,/[x+]/); ar=a[1]*a[2];
               if(ar>max){max=ar; wid=$1}}} END{print wid}')
  [ -z "$id" ] && return 1
  xwininfo -id "$id" 2>/dev/null | awk '/Absolute upper-left X/{x=$4} /Absolute upper-left Y/{y=$4}
                                        /^  Width:/{w=$2} /^  Height:/{h=$2} END{print w, h, x, y}'
}

case "${1:-status}" in
  start)
    TAG="${2:-rviz}"
    mkdir -p "$OUT_DIR"
    read -r W H X Y <<< "$(find_geom)"
    if [ -z "$W" ]; then
      echo "!! 没找到 rviz 窗口(DISPLAY=$DISPLAY)。先把 rviz 开起来, 或者用 FULLSCREEN=1 录整屏。"
      [ "$FULLSCREEN" = "1" ] || exit 1
      read -r W H <<< "$(xwininfo -root | awk '/Width:/{w=$2} /Height:/{h=$2} END{print w, h}')"
      X=0; Y=0
    fi
    # 渲染区左右各挂着一条 ~16px 的面板折叠把手(浅色窄边), 裁掉才干净
    TRIM="${TRIM:-17}"
    X=$((X + TRIM)); W=$((W - 2 * TRIM))
    W=$((W / 2 * 2)); H=$((H / 2 * 2))   # yuv420p 要求宽高是偶数
    STAMP=$(date +%Y%m%d_%H%M%S)
    OUT="$OUT_DIR/${STAMP}_${TAG}_rviz.mp4"
    echo "窗口 ${W}x${H} @ +${X}+${Y}  ->  $OUT"
    echo "+ ffmpeg -f x11grab -framerate $FPS -video_size ${W}x${H} -i ${DISPLAY}+${X},${Y} ..."
    setsid nohup ffmpeg -y -f x11grab -draw_mouse 0 -framerate "$FPS" \
      -video_size "${W}x${H}" -i "${DISPLAY}+${X},${Y}" \
      -c:v h264_nvenc -preset hq -rc vbr -cq 23 -b:v 0 -pix_fmt yuv420p "$OUT" \
      > /tmp/rec_rviz.log 2>&1 < /dev/null &
    echo $! > "$PIDFILE"
    sleep 2
    if kill -0 "$(cat $PIDFILE)" 2>/dev/null; then
      echo "起录 OK  wall_start=$(date +%s)  ($(date '+%F %T'))"
      echo "$OUT" > /tmp/rec_rviz.out
    else
      echo "!! ffmpeg 秒退, 日志:"; tail -15 /tmp/rec_rviz.log
    fi
    ;;
  stop)
    # 必须发 SIGINT(不是 KILL), ffmpeg 才会写 moov 收尾, 否则 mp4 播不了
    if [ -f "$PIDFILE" ]; then
      echo "+ kill -INT $(cat $PIDFILE)"
      kill -INT "$(cat $PIDFILE)" 2>/dev/null
      sleep 2
      rm -f "$PIDFILE"
    else
      pkill -INT -f "x11grab" && echo "+ pkill -INT x11grab"
      sleep 2
    fi
    tail -3 /tmp/rec_rviz.log
    [ -f /tmp/rec_rviz.out ] && ls -lh "$(cat /tmp/rec_rviz.out)"
    ;;
  status)
    pgrep -af "x11grab" && echo "==> 正在录" || echo "==> 没在录"
    ls -lht "$OUT_DIR" 2>/dev/null | head -5
    ;;
  *)
    echo "用法: bash ~/rec_rviz.sh {start [tag] | stop | status}"
    ;;
esac
