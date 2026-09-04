#!/usr/bin/env bash
# make_5view_0801.sh -- 08-01 双站五罐那趟的五视角合成 (2026-08-01)
#
# 布局: 上二下三, **无标题条**, 每格左上角一个英文标签。
#   上排 = 人看到的世界:   Overhead 960x540 | Handheld 960x540
#   下排 = 机器人看到的:   Onboard RGB 640x360 | Onboard depth 640x360 | rviz 640x360
#   => 画布 1920x900
#
# 用法: bash make_5view_0801.sh full        完整版(431s)
#       bash make_5view_0801.sh highlight   精华版(跳过 can2, 其余四罐 3.87x -> 75s)
#
# ## 对齐依据(全部实测, 别改)
# 任务开始 = 20:00:13 (run_fixed5 日志的"开始"行), 任务全长 431s。
# 各路起录时刻 -> 相对任务开始要跳过多少:
#   Overhead  19:56:59        -ss 194.00   (⚠️ **VFR**: 标称30 实际 29.05fps, 必须转 CFR)
#   Handheld  19:59:52        -ss  21.00   (恒定 30fps, 不用转)
#   Onboard   20:00:11.146    -ss   1.85   (meta.json 的 wall_start_epoch)
#   rviz      19:49:43        -ss 630.00   (rec_rviz.sh 打印的 wall_start)
# 三方时钟(车/Android/Windows)都是 NTP 同步的, 文件名时间戳可直接用, 实测各路只差 0.4~0.8s。
#
# ## 各罐时间点(相对任务开始, 秒)
#   can4→B    0-65      can5→B   65-148
#   can2→A  148-289   ← ⚠️ **这段有碰撞 + 用户进场扶罐**, 精华版跳过
#   can1→A  289-367     can3→A  367-431
set -u
D=$HOME/jetrover_demo
R="$D/raw/20260801_2000_2station-5can-SUCCESS-5of5"
FONT=/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc
MODE="${1:-full}"

CEIL="$R/WIN_20260801_19_56_59_Pro.mp4";  CEIL_SS=194.00
PHONE="$R/VID_20260801_195952.mp4";       PHONE_SS=21.00
RGB="$R/20260801_200010_demo2st_rgb.mp4"; CAR_SS=1.85
DEPTH="$R/20260801_200010_demo2st_depth.mp4"
RVIZ="$R/20260801_194941_demo2st_rviz.mp4"; RVIZ_SS=630.00
DUR=431

OUTDIR="$D/edited/20260801_2000_2station-5can-SUCCESS-5of5/final"
FULL="$OUTDIR/20260801_2000_2station-5can_5view_full.mp4"
HILITE="$OUTDIR/20260801_2000_2station-5can_5view_highlight75s.mp4"

# 标签: 半透明黑底 + 白字, 左上角
lbl() { echo "drawtext=fontfile=$FONT:text='$1':fontsize=$2:fontcolor=white:box=1:boxcolor=black@0.55:boxborderw=8:x=14:y=12"; }

mkdir -p "$OUTDIR"

if [ "$MODE" = "full" ] || [ ! -f "$FULL" ]; then
  echo "=== 合成完整版 (${DUR}s, 1920x900) ==="
  ffmpeg -y -hide_banner -loglevel warning -stats \
    -ss $CEIL_SS  -t $DUR -i "$CEIL" \
    -ss $PHONE_SS -t $DUR -i "$PHONE" \
    -ss $CAR_SS   -t $DUR -i "$RGB" \
    -ss $CAR_SS   -t $DUR -i "$DEPTH" \
    -ss $RVIZ_SS  -t $DUR -i "$RVIZ" \
    -filter_complex "
      [0:v]fps=15,scale=960:540,setsar=1,$(lbl 'Overhead' 30)[a];
      [1:v]fps=15,scale=960:540,setsar=1,$(lbl 'Handheld' 30)[b];
      [2:v]fps=15,scale=640:360,setsar=1,$(lbl 'Onboard RGB' 22)[c];
      [3:v]fps=15,scale=640:360,setsar=1,$(lbl 'Onboard depth' 22)[d];
      [4:v]fps=15,scale=640:-2,crop=640:360,setsar=1,$(lbl 'rviz' 22)[e];
      [a][b]hstack=inputs=2[top];
      [c][d][e]hstack=inputs=3[bot];
      [top][bot]vstack=inputs=2[v]" \
    -map "[v]" -c:v h264_nvenc -preset hq -rc vbr -cq 23 -b:v 0 -pix_fmt yuv420p -an \
    "$FULL"
  echo "完整版 -> $FULL"
fi

if [ "$MODE" = "highlight" ]; then
  echo "=== 从完整版剪精华 (跳过 can2 的 148-289s, 其余四罐 3.87x) ==="
  # 四段干净素材: 0-65 / 65-148 / 289-367 / 367-431 = 290s -> /3.87 = 75s
  ffmpeg -y -hide_banner -loglevel warning -stats -i "$FULL" \
    -filter_complex "
      [0:v]trim=0:148,setpts=PTS-STARTPTS[s1];
      [0:v]trim=289:431,setpts=PTS-STARTPTS[s2];
      [s1][s2]concat=n=2:v=1[cat];
      [cat]setpts=PTS/3.87,fps=30[v]" \
    -map "[v]" -c:v h264_nvenc -preset hq -rc vbr -cq 23 -b:v 0 -pix_fmt yuv420p -an \
    "$HILITE"
  echo "精华版 -> $HILITE"
fi
