#!/usr/bin/env bash
# Draw brief first-detection boxes for the 2026-08-27 nighttime Tello video.
set -euo pipefail

RUN=$HOME/jetrover_demo/raw/20260827_2030_drone5_2station-5can-SUCCESS-5of5
IN="${IN:-$RUN/opencv_tello_record_09.mp4}"
OUT="${OUT:-$HOME/jetrover_demo/edited/20260827_2030_drone5_2station-5can-SUCCESS-5of5/intermediate/20260827_2030_drone5_detection_boxes.mp4}"
FONT=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf

mkdir -p "$(dirname "$OUT")"

ffmpeg -y -hide_banner -loglevel warning -stats -i "$IN" \
  -vf "\
drawbox=enable='between(t,44.50,45.40)':x=95:y=0:w=145:h=160:color=yellow@0.95:t=5,\
drawtext=enable='between(t,44.50,45.40)':fontfile=$FONT:text='can4 DETECTED':x=235:y=20:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,76.75,77.65)':x=790:y=30:w=115:h=170:color=yellow@0.95:t=5,\
drawtext=enable='between(t,76.75,77.65)':fontfile=$FONT:text='can3 DETECTED':x=560:y=210:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,91.50,92.40)':x=750:y=75:w=155:h=175:color=yellow@0.95:t=5,\
drawtext=enable='between(t,91.50,92.40)':fontfile=$FONT:text='can2 DETECTED':x=450:y=250:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55,\
drawbox=enable='between(t,100.00,100.90)':x=640:y=20:w=220:h=190:color=yellow@0.95:t=5,\
drawtext=enable='between(t,100.00,100.90)':fontfile=$FONT:text='can1 DETECTED':x=485:y=210:fontsize=28:fontcolor=yellow:box=1:boxcolor=black@0.55" \
  -c:v libx264 -preset veryfast -crf 18 -pix_fmt yuv420p -an -movflags +faststart "$OUT"

printf '%s\n' "$OUT"
