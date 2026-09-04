#!/usr/bin/env bash
# Synchronized six-view full edit for the 2026-08-27 daytime drone5 run.
set -euo pipefail

D=$HOME/jetrover_demo
R="$D/raw/20260827_1631_drone5_2station-5can-SUCCESS-5of5"
FONT=/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc
EXT_A="$R/IMG_3319.MOV"; EXT_A_SS=130.771553
EXT_B="$R/VID_20260827_162913.mp4"; EXT_B_SS=114.771553
RGB="$R/20260827_163103_0827b_drone5_rgb.mp4"; ONBOARD_SS=4.346553
DEPTH="$R/20260827_163103_0827b_drone5_depth.mp4"
DRONE="$D/edited/20260827_1631_drone5_2station-5can-SUCCESS-5of5/intermediate/20260827_1631_drone5_detection_boxes.mp4"; DRONE_SS=50.471553
RVIZ="$D/edited/20260827_1631_drone5_2station-5can-SUCCESS-5of5/intermediate/20260827_1631_drone5_dynamic_rviz_full.mp4"; RVIZ_SS=2.000
DUR=683.789
OUTDIR="$D/edited/20260827_1631_drone5_2station-5can-SUCCESS-5of5/final"
OUT="$OUTDIR/20260827_1631_drone5_2station-5can_6view_full.mp4"

if [ -e /dev/nvidiactl ] || [ -e /dev/nvidia0 ]; then ENC=(-c:v h264_nvenc -preset hq -rc vbr -cq 23 -b:v 0); else ENC=(-c:v libx264 -preset veryfast -crf 20); fi
lbl() { echo "drawtext=fontfile=$FONT:text='$1':fontsize=22:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=7:x=12:y=10"; }
cell() { echo "fps=15,scale=640:360:force_original_aspect_ratio=increase,crop=640:360,setsar=1,tpad=stop_mode=clone:stop_duration=700,trim=duration=$DUR,setpts=PTS-STARTPTS"; }
mkdir -p "$OUTDIR"

ffmpeg -y -hide_banner -loglevel warning -stats \
  -ss "$EXT_A_SS" -i "$EXT_A" -ss "$EXT_B_SS" -i "$EXT_B" -ss "$ONBOARD_SS" -i "$RGB" \
  -ss "$DRONE_SS" -i "$DRONE" -ss "$RVIZ_SS" -i "$RVIZ" -ss "$ONBOARD_SS" -i "$DEPTH" \
  -filter_complex "
    [0:v]$(cell),$(lbl 'External A')[a]; [1:v]$(cell),$(lbl 'External B')[b]; [2:v]$(cell),$(lbl 'Onboard RGB')[c];
    [3:v]$(cell),$(lbl 'Drone detection')[d]; [4:v]$(cell),$(lbl 'Dynamic RVIZ')[e]; [5:v]$(cell),$(lbl 'Onboard depth')[f];
    [a][b][c]hstack=inputs=3[top]; [d][e][f]hstack=inputs=3[bot]; [top][bot]vstack=inputs=2[v]" \
  -map "[v]" -t "$DUR" "${ENC[@]}" -pix_fmt yuv420p -an -movflags +faststart "$OUT"
echo "$OUT"
