#!/usr/bin/env bash
# Draw brief first-detection boxes for the 2026-08-27 daytime Tello video.
set -euo pipefail

RUN=$HOME/jetrover_demo/raw/20260827_1631_drone5_2station-5can-SUCCESS-5of5
IN="${IN:-$RUN/opencv_tello_record_07.mp4}"
OUT="${OUT:-$HOME/jetrover_demo/edited/20260827_1631_drone5_2station-5can-SUCCESS-5of5/intermediate/20260827_1631_drone5_detection_boxes.mp4}"
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf

mkdir -p "$(dirname "$OUT")"

ffmpeg -y -hide_banner -loglevel warning -stats -i "$IN" \
  -vf "\
drawbox=enable='between(t,92.50,93.40)':x=570:y=0:w=250:h=140:color=yellow@0.95:t=5,\
drawtext=enable='between(t,92.50,93.40)':fontfile=$FONT:text='can4 DETECTED':x=570:y=150:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,111.50,112.40)':x=685:y=0:w=95:h=125:color=yellow@0.95:t=5,\
drawtext=enable='between(t,111.50,112.40)':fontfile=$FONT:text='can3 DETECTED':x=525:y=130:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,129.267,130.167)':x=375:y=0:w=230:h=145:color=yellow@0.95:t=5,\
drawtext=enable='between(t,129.267,130.167)':fontfile=$FONT:text='can2 DETECTED':x=480:y=25:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,143.50,144.40)':x=780:y=335:w=175:h=170:color=yellow@0.95:t=5,\
drawtext=enable='between(t,143.50,144.40)':fontfile=$FONT:text='can1 DETECTED':x=600:y=500:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55" \
  -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p -an -movflags +faststart "$OUT"

printf '%s\n' "$OUT"
