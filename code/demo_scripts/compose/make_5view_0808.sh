#!/usr/bin/env bash
# 2026-08-08 two-station five-can closed-loop demo, five-view composite.
# Layout: top row 2 views, bottom row 3 views, no title bar.

set -euo pipefail

D=$HOME/jetrover_demo
R="$D/raw/20260808_1538_2station-5can-closedloop"
FONT=/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc
MODE="${1:-full}"

CEIL="$R/WIN_20260808_15_34_35_Pro.mp4";       CEIL_SS=223.406
PHONE="$R/VID_20260808_153600.mp4";            PHONE_SS=138.406
RGB="$R/20260808_153818_demo0808_rgb.mp4";     CAR_SS=0.000
DEPTH="$R/20260808_153818_demo0808_depth.mp4"
RVIZ="$R/20260808_153803_demo0808_rviz.mp4";   RVIZ_SS=15.406

# RGB/depth end at 411.800s in frames.csv; use the common overlap.
DUR=411.80
TARGET_HIGHLIGHT=75
SPEED=5.4907
SPEED_PTS=0.182126

OUTDIR="$D/edited/20260808_1538_2station-5can-closedloop/final"
FULL="$OUTDIR/20260808_1538_2station-5can_closedloop_5view_full.mp4"
HILITE="$OUTDIR/20260808_1538_2station-5can_closedloop_5view_highlight75s.mp4"

if [ -e /dev/nvidiactl ] || [ -e /dev/nvidia0 ]; then
  ENC=(-c:v h264_nvenc -preset hq -rc vbr -cq 23 -b:v 0)
else
  ENC=(-c:v libx264 -preset veryfast -crf 20)
fi

lbl() {
  echo "drawtext=fontfile=$FONT:text='$1':fontsize=$2:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=8:x=14:y=12"
}

mkdir -p "$OUTDIR"

if [ "$MODE" = "full" ] || [ "$MODE" = "all" ]; then
  ffmpeg -y -hide_banner -loglevel warning -stats \
    -ss "$CEIL_SS"  -t "$DUR" -i "$CEIL" \
    -ss "$PHONE_SS" -t "$DUR" -i "$PHONE" \
    -ss "$CAR_SS"   -t "$DUR" -i "$RGB" \
    -ss "$CAR_SS"   -t "$DUR" -i "$DEPTH" \
    -ss "$RVIZ_SS"  -t "$DUR" -i "$RVIZ" \
    -filter_complex "
      [0:v]fps=15,scale=960:540:force_original_aspect_ratio=increase,crop=960:540,setsar=1,$(lbl 'Overhead' 30)[a];
      [1:v]fps=15,scale=960:540:force_original_aspect_ratio=increase,crop=960:540,setsar=1,$(lbl 'Handheld' 30)[b];
      [2:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Onboard RGB' 22)[c];
      [3:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Onboard depth' 22)[d];
      [4:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'rviz' 22)[e];
      [a][b]hstack=inputs=2[top];
      [c][d][e]hstack=inputs=3[bot];
      [top][bot]vstack=inputs=2[v]" \
    -map "[v]" "${ENC[@]}" -pix_fmt yuv420p -an "$FULL"
  echo "$FULL"
fi

if [ "$MODE" = "highlight" ] || [ "$MODE" = "all" ]; then
  if [ ! -f "$FULL" ]; then
    "$0" full
  fi
  ffmpeg -y -hide_banner -loglevel warning -stats -i "$FULL" \
    -filter_complex "[0:v]setpts=$SPEED_PTS*PTS,fps=30,trim=duration=$TARGET_HIGHLIGHT,setpts=PTS-STARTPTS[v]" \
    -map "[v]" "${ENC[@]}" -pix_fmt yuv420p -an "$HILITE"
  echo "$HILITE"
fi
