#!/usr/bin/env bash
# 2026-08-16 gateB six-view composite, v2.
#
# v1 -> v2 的三处改动(都是用户看片后提的, 每条都有实测依据):
#
#  1) 掐掉开头 11.5 秒。实测车在合成时间轴 t=12.8s 才第一次动
#     (车载 RGB 帧差 0.9 -> 12.06 的阶跃在 RGB 自身 17.0s, 减去 CAR_SS=4.189)。
#     在那之前画面里只有 rviz 在画全局路径、车纹丝不动 —— 而无人机 34.4s 才出现,
#     那段"还没探测就已经在规划"是全片唯一逻辑不自洽的地方。剪掉它, 后四罐的
#     "先看见再去拿"全都成立(实测各早 28/50/130/201 秒)。
#
#  2) 无人机那格改成 tpad 克隆首尾帧, 不再垫黑底。
#     无人机原片只有 118.3s, 而全片 426.5s ⇒ v1 里那格有 274 秒是纯黑(占 2/3)。
#
#  3) "Raw RVIZ path" 换成车载深度。原因: 它和 Dynamic RVIZ 内容重复;
#     而深度是真正不同的模态。⚠️ make_2view 脚本里"这趟深度基本全黑"的说法是
#     写过头了 —— 实测逐段亮度 149/0/128/87/0/132/140/0, 那几个 0 是夹着罐子时
#     (罐在爪里 <0.3m + 金属反光, 超出伪彩区间), 和 08-08 scene3 那趟一模一样,
#     而后者是上过五路成片的。
#
# 布局: 上排 Ceiling | Handheld | Onboard RGB
#       下排 Drone   | Dynamic RVIZ | Onboard depth

set -euo pipefail

D=$HOME/jetrover_demo
R="$D/raw/20260815_1858_gateB_2station-5can-SUCCESS-5of5"
FONT=/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc
MODE="${1:-full}"

CUT=11.5                      # 掐掉的开头长度(合成时间轴)

CEIL="$R/WIN_20260815_18_57_27_Pro.mp4";                    CEIL_SS=74.104
PHONE="$R/VID_20260815_185722.mp4";                         PHONE_SS=80.104
RGB="$R/20260815_185826_0815_gateB_rgb.mp4";                CAR_SS=15.689
DEPTH="$R/20260815_185826_0815_gateB_depth.mp4"
DRONE="$D/edited/20260815_1858_gateB_2station-5can-SUCCESS-5of5/intermediate/20260815_1858_gateB_drone_detect_boxes.mp4"
RVIZ="$D/edited/20260815_1858_gateB_2station-5can-SUCCESS-5of5/intermediate/20260815_1858_gateB_dynamic_rviz_full.mp4";  RVIZ_SS=13.500

DUR=415.0                     # 426.5 - CUT
DRONE_LEAD=22.896             # 34.396 - CUT, 这段克隆无人机第一帧
DRONE_TAIL=280                # 覆盖 415.0 - 22.896 - 118.334 = 273.8, 留余量后 trim
TARGET_HIGHLIGHT=75
SPEED_PTS=0.180723            # 75 / 415.0

OUTDIR="$D/edited/20260815_1858_gateB_2station-5can-SUCCESS-5of5/final"
FULL="$OUTDIR/20260815_1858_gateB_2station-5can_6view_v2_full.mp4"
HILITE="$OUTDIR/20260815_1858_gateB_2station-5can_6view_v2_highlight75s.mp4"

if [ -e /dev/nvidiactl ] || [ -e /dev/nvidia0 ]; then
  ENC=(-c:v h264_nvenc -preset hq -rc vbr -cq 23 -b:v 0)
else
  ENC=(-c:v libx264 -preset veryfast -crf 20)
fi

lbl() {
  echo "drawtext=fontfile=$FONT:text='$1':fontsize=$2:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=7:x=12:y=10"
}

mkdir -p "$OUTDIR"

if [ "$MODE" = "full" ] || [ "$MODE" = "all" ]; then
  ffmpeg -y -hide_banner -loglevel warning -stats \
    -ss "$CEIL_SS"  -t "$DUR" -i "$CEIL" \
    -ss "$PHONE_SS" -t "$DUR" -i "$PHONE" \
    -ss "$CAR_SS"   -t "$DUR" -i "$RGB" \
    -i "$DRONE" \
    -ss "$RVIZ_SS"  -t "$DUR" -i "$RVIZ" \
    -ss "$CAR_SS"   -t "$DUR" -i "$DEPTH" \
    -filter_complex "
      [0:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Ceiling' 22)[a];
      [1:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Handheld' 22)[b];
      [2:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Onboard RGB' 22)[c];
      [3:v]fps=15,scale=640:360:force_original_aspect_ratio=decrease,pad=640:360:(ow-iw)/2:(oh-ih)/2:color=0x101010,setsar=1,
           tpad=start_mode=clone:start_duration=$DRONE_LEAD:stop_mode=clone:stop_duration=$DRONE_TAIL,
           trim=duration=$DUR,setpts=PTS-STARTPTS,$(lbl 'Drone detection' 22)[d];
      [4:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Dynamic RVIZ' 22)[e];
      [5:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Onboard depth' 22)[f];
      [a][b][c]hstack=inputs=3[top];
      [d][e][f]hstack=inputs=3[bot];
      [top][bot]vstack=inputs=2[v]" \
    -map "[v]" -t "$DUR" "${ENC[@]}" -pix_fmt yuv420p -an -movflags +faststart "$FULL"
  echo "$FULL"
fi

if [ "$MODE" = "highlight" ] || [ "$MODE" = "all" ]; then
  if [ ! -f "$FULL" ]; then
    "$0" full
  fi
  ffmpeg -y -hide_banner -loglevel warning -stats -i "$FULL" \
    -filter_complex "[0:v]setpts=$SPEED_PTS*PTS,fps=30,trim=duration=$TARGET_HIGHLIGHT,setpts=PTS-STARTPTS[v]" \
    -map "[v]" "${ENC[@]}" -pix_fmt yuv420p -an -movflags +faststart "$HILITE"
  echo "$HILITE"
fi
