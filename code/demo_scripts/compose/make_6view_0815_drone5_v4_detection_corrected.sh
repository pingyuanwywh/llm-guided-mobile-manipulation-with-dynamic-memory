#!/usr/bin/env bash
# 2026-08-15 drone5 six-view composite with corrected detection identities and timing.
set -euo pipefail

D=$HOME/jetrover_demo
R="$D/raw/20260815_2120_drone5_2station-5can-SUCCESS-5of5"
FONT=/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc
MODE="${1:-full}"

CEIL="$R/WIN_20260815_21_19_52_Pro.mp4";                    CEIL_SS=68.937
PHONE="$R/VID_20260815_212002.mp4";                         PHONE_SS=59.937
RGB="$R/20260815_212045_0815_drone5_rgb.mp4";               CAR_SS=15.833
DEPTH="$R/20260815_212045_0815_drone5_depth.mp4"
DRONE="$D/edited/20260815_2120_drone5_2station-5can-SUCCESS-5of5/intermediate/20260815_2120_drone5_drone_detect_boxes_v2_corrected.mp4"
RVIZ="$D/edited/20260815_2120_drone5_2station-5can-SUCCESS-5of5/intermediate/20260815_2120_drone5_dynamic_rviz_v2_corrected_full.mp4"; RVIZ_SS=14.000

DUR=385.3
DRONE_LEAD=13.063
DRONE_TAIL=250
TARGET_HIGHLIGHT=75
SPEED_PTS=0.19465

OUTDIR="$D/edited/20260815_2120_drone5_2station-5can-SUCCESS-5of5/final"
FULL="$OUTDIR/20260815_2120_drone5_2station-5can_6view_v4_detection_corrected_full.mp4"
HILITE="$OUTDIR/20260815_2120_drone5_2station-5can_6view_v4_detection_corrected_highlight75s.mp4"

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
