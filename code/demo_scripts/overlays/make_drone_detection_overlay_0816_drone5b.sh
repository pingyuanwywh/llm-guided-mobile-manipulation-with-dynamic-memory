#!/bin/bash
# Render the 2026-08-16 drone5b view with brief detection boxes.
set -euo pipefail

RUN="$HOME/jetrover_demo/raw/20260816_1207_drone5b_2station-5can-SUCCESS-5of5"
IN="${IN:-$RUN/simple_search_20260816_120704.mp4}"
OUT="${OUT:-$HOME/jetrover_demo/edited/20260816_1207_drone5b_2station-5can-SUCCESS-5of5/intermediate/20260816_1207_drone5b_drone_detect_boxes_v2_corrected.mp4}"
FONT="/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"

mkdir -p "$(dirname "$OUT")"

ffmpeg -hide_banner -y -i "$IN" \
  -vf "\
drawbox=enable='between(t,40.5,41.3)':x=810:y=500:w=120:h=135:color=yellow@0.95:t=5,\
drawtext=enable='between(t,40.5,41.3)':fontfile=$FONT:text='can2 DETECTED':x=580:y=520:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,71.5,72.3)':x=780:y=45:w=130:h=130:color=yellow@0.95:t=5,\
drawtext=enable='between(t,71.5,72.3)':fontfile=$FONT:text='can3 DETECTED':x=585:y=65:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,101.5,102.3)':x=820:y=510:w=130:h=150:color=yellow@0.95:t=5,\
drawtext=enable='between(t,101.5,102.3)':fontfile=$FONT:text='can5 DETECTED':x=610:y=530:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,145.0,145.8)':x=230:y=20:w=140:h=145:color=yellow@0.95:t=5,\
drawtext=enable='between(t,145.0,145.8)':fontfile=$FONT:text='can1 DETECTED':x=380:y=30:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55" \
  -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p -an "$OUT"

echo "$OUT"
