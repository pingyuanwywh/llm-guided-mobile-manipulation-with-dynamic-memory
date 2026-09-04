#!/bin/bash
# Render the 2026-08-15 drone5 view with brief detection boxes.
set -euo pipefail

RUN="$HOME/jetrover_demo/raw/20260815_2120_drone5_2station-5can-SUCCESS-5of5"
IN="${IN:-$RUN/simple_search_20260815_212115.mp4}"
OUT="${OUT:-$HOME/jetrover_demo/edited/20260815_2120_drone5_2station-5can-SUCCESS-5of5/intermediate/20260815_2120_drone5_drone_detect_boxes_v2_corrected.mp4}"
FONT="/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

mkdir -p "$(dirname "$OUT")"

ffmpeg -hide_banner -y -i "$IN" \
  -vf "\
drawbox=enable='between(t,35.0,35.8)':x=850:y=0:w=110:h=105:color=yellow@0.95:t=5,\
drawtext=enable='between(t,35.0,35.8)':fontfile=$FONT:text='can5 DETECTED':x=630:y=20:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,41.0,41.8)':x=835:y=565:w=125:h=150:color=yellow@0.95:t=5,\
drawtext=enable='between(t,41.0,41.8)':fontfile=$FONT:text='can1 DETECTED':x=615:y=585:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,72.0,72.8)':x=805:y=0:w=105:h=90:color=yellow@0.95:t=5,\
drawtext=enable='between(t,72.0,72.8)':fontfile=$FONT:text='can3 DETECTED':x=585:y=20:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,95.5,96.3)':x=835:y=200:w=125:h=135:color=yellow@0.95:t=5,\
drawtext=enable='between(t,95.5,96.3)':fontfile=$FONT:text='can2 DETECTED':x=600:y=220:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55" \
  -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p -an "$OUT"

echo "$OUT"
