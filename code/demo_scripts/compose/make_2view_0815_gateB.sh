#!/usr/bin/env bash
# 2026-08-15 gateB demo, two-view composite.
# Depth video in this take is effectively black, so this uses rviz + onboard RGB.

set -euo pipefail

D=$HOME/jetrover_demo
R="$D/raw/20260815_1858_gateB_2station-5can-SUCCESS-5of5"
FONT=/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc
MODE="${1:-all}"

RVIZ="$R/20260815_185822_0815_gateB_rviz.mp4"; RVIZ_SS=4.000
RGB="$R/20260815_185826_0815_gateB_rgb.mp4";   CAR_SS=0.000

DUR=430.733
TARGET_HIGHLIGHT=75
SPEED_PTS=0.174095

OUTDIR="$D/edited/20260815_1858_gateB_2station-5can-SUCCESS-5of5/final"
FULL="$OUTDIR/20260815_1858_gateB_2station-5can_2view_full.mp4"
HILITE="$OUTDIR/20260815_1858_gateB_2station-5can_2view_highlight75s.mp4"

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
    -ss "$RVIZ_SS" -t "$DUR" -i "$RVIZ" \
    -ss "$CAR_SS"  -t "$DUR" -i "$RGB" \
    -filter_complex "
      [0:v]fps=15,scale=960:540:force_original_aspect_ratio=increase,crop=960:540,setsar=1,$(lbl 'rviz' 30)[a];
      [1:v]fps=15,scale=960:540:force_original_aspect_ratio=increase,crop=960:540,setsar=1,$(lbl 'Onboard RGB' 30)[b];
      [a][b]hstack=inputs=2[v]" \
    -map "[v]" "${ENC[@]}" -pix_fmt yuv420p -an -movflags +faststart "$FULL"
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
