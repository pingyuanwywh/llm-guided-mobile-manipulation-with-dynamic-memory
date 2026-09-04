#!/usr/bin/env bash
set -euo pipefail

run_dir="$HOME/jetrover_demo/raw/20260827_2030_drone5_2station-5can-SUCCESS-5of5"
output_dir="$HOME/jetrover_demo/edited/20260827_2030_drone5_2station-5can-SUCCESS-5of5/intermediate/frame_dropped"

# The complete first can1 task is omitted. Source wall-clock boundaries are
# 20:37:26 (failed task begins) and 20:38:41 (successful retry begins).
onboard_cut_start="428.084"
onboard_cut_end="503.084"
external_cut_start="457.084"
external_cut_end="532.084"

watermark="drawtext=fontfile=/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf:text='FRAME-DROPPED':x=w-tw-14:y=14:fontsize=24:fontcolor=white:box=1:boxcolor=black@0.68:boxborderw=7"

mkdir -p "$output_dir"

encode_video_only() {
    local input_path="$1"
    local output_path="$2"
    local cut_start="$3"
    local cut_end="$4"

    echo "Starting $(basename "$output_path")"
    ffmpeg -hide_banner -loglevel warning -stats -y \
        -i "$input_path" \
        -filter_complex \
        "[0:v]trim=start=0:end=${cut_start},setpts=PTS-STARTPTS[v0];\
         [0:v]trim=start=${cut_end},setpts=PTS-STARTPTS[v1];\
         [v0][v1]concat=n=2:v=1:a=0,${watermark}[vout]" \
        -map "[vout]" \
        -c:v libx264 -preset fast -crf 18 -threads 6 \
        -pix_fmt yuv420p -movflags +faststart \
        -metadata comment="FRAME-DROPPED edit; failed can1 task removed" \
        "$output_path"
    echo "Finished $(basename "$output_path")"
}

encode_video_audio() {
    local input_path="$1"
    local output_path="$2"
    local cut_start="$3"
    local cut_end="$4"

    echo "Starting $(basename "$output_path")"
    ffmpeg -hide_banner -loglevel warning -stats -y \
        -i "$input_path" \
        -filter_complex \
        "[0:v]trim=start=0:end=${cut_start},setpts=PTS-STARTPTS[v0];\
         [0:v]trim=start=${cut_end},setpts=PTS-STARTPTS[v1];\
         [v0][v1]concat=n=2:v=1:a=0,${watermark}[vout];\
         [0:a]atrim=start=0:end=${cut_start},asetpts=PTS-STARTPTS[a0];\
         [0:a]atrim=start=${cut_end},asetpts=PTS-STARTPTS[a1];\
         [a0][a1]concat=n=2:v=0:a=1[aout]" \
        -map "[vout]" -map "[aout]" \
        -c:v libx264 -preset fast -crf 18 -threads 6 \
        -c:a aac -b:a 128k \
        -pix_fmt yuv420p -movflags +faststart \
        -metadata comment="FRAME-DROPPED edit; failed can1 task removed" \
        "$output_path"
    echo "Finished $(basename "$output_path")"
}

encode_video_only \
    "$run_dir/20260827_203017_0827d_drone5_rgb.mp4" \
    "$output_dir/20260827_203017_0827d_drone5_rgb_frame_dropped.mp4" \
    "$onboard_cut_start" "$onboard_cut_end" &
pid_rgb=$!

encode_video_only \
    "$run_dir/20260827_203017_0827d_drone5_depth.mp4" \
    "$output_dir/20260827_203017_0827d_drone5_depth_frame_dropped.mp4" \
    "$onboard_cut_start" "$onboard_cut_end" &
pid_depth=$!

encode_video_audio \
    "$run_dir/demo_night(1).mp4" \
    "$output_dir/demo_night(1)_frame_dropped.mp4" \
    "$external_cut_start" "$external_cut_end" &
pid_external1=$!

encode_video_audio \
    "$run_dir/demo_night(2).mp4" \
    "$output_dir/demo_night(2)_frame_dropped.mp4" \
    "$external_cut_start" "$external_cut_end" &
pid_external2=$!

wait "$pid_rgb"
wait "$pid_depth"
wait "$pid_external1"
wait "$pid_external2"

echo "All FRAME-DROPPED videos are ready in $output_dir"
