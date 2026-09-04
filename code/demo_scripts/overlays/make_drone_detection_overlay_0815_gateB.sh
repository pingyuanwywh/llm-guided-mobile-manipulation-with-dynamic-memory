#!/bin/bash
# Render the drone view with brief detection boxes at first visual detection.
set -euo pipefail

RUN="$HOME/jetrover_demo/raw/20260815_1858_gateB_2station-5can-SUCCESS-5of5"
IN="${IN:-$RUN/simple_search_20260815_185858.mp4}"
OUT="${OUT:-$HOME/jetrover_demo/edited/20260815_1858_gateB_2station-5can-SUCCESS-5of5/intermediate/20260815_1858_gateB_drone_detect_boxes.mp4}"
FONT="/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

mkdir -p "$(dirname "$OUT")"

ffmpeg -hide_banner -y -i "$IN" \
  -vf "\
drawbox=enable='between(t,29.5,31.3)':x=815:y=0:w=120:h=90:color=yellow@0.95:t=5,\
drawtext=enable='between(t,29.5,31.3)':fontfile=$FONT:text='can5 DETECTED':x=610:y=18:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,96.0,98.2)':x=0:y=565:w=125:h=150:color=yellow@0.95:t=5,\
drawtext=enable='between(t,96.0,98.2)':fontfile=$FONT:text='can3 DETECTED':x=10:y=525:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,96.5,98.7)':x=495:y=570:w=115:h=145:color=yellow@0.95:t=5,\
drawtext=enable='between(t,96.5,98.7)':fontfile=$FONT:text='can1 DETECTED':x=455:y=530:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,105.0,107.2)':x=545:y=510:w=120:h=160:color=yellow@0.95:t=5,\
drawtext=enable='between(t,105.0,107.2)':fontfile=$FONT:text='can2 DETECTED':x=500:y=470:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55" \
  -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p -an "$OUT"

echo "$OUT"
