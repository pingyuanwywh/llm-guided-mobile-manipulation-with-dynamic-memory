#!/usr/bin/env bash
# 2026-08-16 drone5b six-view composite, v2 style.
set -euo pipefail

D=$HOME/jetrover_demo
R="$D/raw/20260816_1207_drone5b_2station-5can-SUCCESS-5of5"
FONT=/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc
MODE="${1:-full}"

CUT=11.5

CEIL="$R/WIN_20260816_12_06_33_Pro.mp4";                       CEIL_SS=65.015
PHONE="$R/VID_20260816_120639.mp4";                            PHONE_SS=59.015
RGB="$R/20260816_120721_0816_drone5b_rgb.mp4";                 CAR_SS=16.683
DEPTH="$R/20260816_120721_0816_drone5b_depth.mp4"
DRONE="$D/edited/20260816_1207_drone5b_2station-5can-SUCCESS-5of5/intermediate/20260816_1207_drone5b_drone_detect_boxes.mp4"; DRONE_SS=34.015
RVIZ="$D/edited/20260816_1207_drone5b_2station-5can-SUCCESS-5of5/intermediate/20260816_1207_drone5b_dynamic_rviz_full.mp4";   RVIZ_SS=13.500

DUR=480.5
DRONE_TAIL=360
TARGET_HIGHLIGHT=75
SPEED_PTS=0.15609

OUTDIR="$D/edited/20260816_1207_drone5b_2station-5can-SUCCESS-5of5/archive"
FULL="$OUTDIR/20260816_1207_drone5b_2station-5can_6view_v2_full.mp4"
HILITE="$OUTDIR/20260816_1207_drone5b_2station-5can_6view_v2_highlight75s.mp4"

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
    -ss "$DRONE_SS" -i "$DRONE" \
    -ss "$RVIZ_SS"  -t "$DUR" -i "$RVIZ" \
    -ss "$CAR_SS"   -t "$DUR" -i "$DEPTH" \
    -filter_complex "
      [0:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Ceiling' 22)[a];
      [1:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Handheld' 22)[b];
      [2:v]fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,$(lbl 'Onboard RGB' 22)[c];
      [3:v]fps=15,scale=640:360:force_original_aspect_ratio=decrease,pad=640:360:(ow-iw)/2:(oh-ih)/2:color=0x101010,setsar=1,
           tpad=stop_mode=clone:stop_duration=$DRONE_TAIL,
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
