#!/usr/bin/env bash
# 2026-08-15 gateB demo, six-view composite.
# Timeline t=0 is bag start (2026-08-15 18:58:30.604 local).

set -euo pipefail

D=$HOME/jetrover_demo
R="$D/raw/20260815_1858_gateB_2station-5can-SUCCESS-5of5"
FONT=/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc
MODE="${1:-full}"

CEIL="$R/WIN_20260815_18_57_27_Pro.mp4";                 CEIL_SS=62.604
PHONE="$R/VID_20260815_185722.mp4";                      PHONE_SS=68.604
RGB="$R/20260815_185826_0815_gateB_rgb.mp4";             CAR_SS=4.189
DRONE="$D/edited/20260815_1858_gateB_2station-5can-SUCCESS-5of5/intermediate/20260815_1858_gateB_drone_detect_boxes.mp4"
RVIZ="$D/edited/20260815_1858_gateB_2station-5can-SUCCESS-5of5/intermediate/20260815_1858_gateB_dynamic_rviz_full.mp4"; RVIZ_SS=2.000
RAW_RVIZ="$R/20260815_185822_0815_gateB_rviz.mp4";       RAW_RVIZ_SS=8.189

DUR=426.5
DRONE_DELAY=34.396
TARGET_HIGHLIGHT=75
SPEED_PTS=0.17585

OUTDIR="$D/edited/20260815_1858_gateB_2station-5can-SUCCESS-5of5/archive"
FULL="$OUTDIR/20260815_1858_gateB_2station-5can_6view_full.mp4"
HILITE="$OUTDIR/20260815_1858_gateB_2station-5can_6view_highlight75s.mp4"

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
    -ss "$RAW_RVIZ_SS" -t "$DUR" -i "$RAW_RVIZ" \
    -filter_complex "
      [0:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Ceiling' 22)[a];
      [1:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Handheld' 22)[b];
      [2:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Onboard RGB' 22)[c];
      [3:v]fps=15,scale=640:360:force_original_aspect_ratio=decrease,pad=640:360:(ow-iw)/2:(oh-ih)/2,setsar=1,setpts=PTS+$DRONE_DELAY/TB[dr];
      color=c=0x101010:s=640x360:r=15:d=$DUR[drbg];
      [drbg][dr]overlay=eof_action=pass:shortest=0,$(lbl 'Drone detection' 22)[d];
      [4:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Dynamic RVIZ' 22)[e];
      [5:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Raw RVIZ path' 22)[f];
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
