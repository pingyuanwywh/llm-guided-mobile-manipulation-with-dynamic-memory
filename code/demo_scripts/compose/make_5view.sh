#!/bin/bash
# 五视角合成 (2026-07-29 JetRover 五罐自主收集 demo)
#
# ## 对齐依据 —— 别再重新推一遍
# 三方时钟(车/Android/Windows)都是 NTP 同步的, 已验证: 车载 meta.json 的
# wall_start_epoch=1785332659.265 在本机换算正好 = 车上自报的 21:44:19。
# 公共锚点 = **车第一次起步**(nav_goto 发出 can1 目标), 绝对时刻 **1785332743.77**
# (车载视频第 84.5s 处帧差由 0.49 跳到 4.11; 与日志 nav_goto 1785332743.68 差 0.09s)。
# 各路起录时刻(用同一锚点做帧差定位反推):
#   天花板 WIN_...21_41_07  -> 1785332467.37  (21:41:07.37, 锚点在其 276.4s)
#   车载   ..._214419       -> 1785332659.265 (meta.json, 精确)
#   rviz   ..._214425       -> 1785332665.07  (21:44:25.07, 锚点在其 78.7s)
#   手机   VID_..._214457   -> 1785332697.77  (21:44:57.77, 锚点在其 46.0s)
# 旁证: 打板左移在 1785332697.16, 比手机起录早 0.61s => 用户说"没录到过去"完全吻合。
#
# 片子起点取"车起步前 3 秒" = 1785332740.77, 时长 405s(到 ALL DONE 之后约 8s)。
# 手机是最短的一路(452.8s), 43.00+405=448 < 452.8, 卡得最紧的就是它。
#
# ⚠️ 天花板那路是**变帧率**(标称30 实际28.79fps), 必须靠 fps=15 转成固定帧率, 否则越往后越偏。
set -e

D=$HOME/jetrover_demo
EXT="$D/external/20260729_2144_5can-1station_ceiling+phone"   # 08-01 修: 原路径 ~/Downloads/... 已不存在, 脚本本来是坏的
FONT=/usr/share/fonts/opentype/noto/NotoSerifCJK-Bold.ttc

CEIL="$EXT/WIN_20260729_21_41_07_Pro.mp4";   CEIL_SS=273.40
PHONE="$EXT/VID_20260729_214457.mp4";        PHONE_SS=43.00
RAW="$D/raw/20260729_2144_5can-1station-SUCCESS-5of5"
RGB="$RAW/20260729_214419_collect5_rgb.mp4";   CAR_SS=81.51
DEPTH="$RAW/20260729_214419_collect5_depth.mp4"
RVIZ="$RAW/20260729_214425_collect5_rviz.mp4"; RVIZ_SS=75.70
DUR=405

# 用法: bash make_5view.sh [zh|en]   默认 zh
# 五路**原片全是干净的**, 文字只在这一层叠加 => 换语言不用重录、不用重新对齐, 改这里重跑即可。
LANGSEL="${1:-zh}"
if [ "$LANGSEL" = "en" ]; then
  OUT="$D/edited/20260729_2144_5can-1station-SUCCESS-5of5/final/$(date +%Y%m%d_%H%M)_5can-1station_5view_en.mp4"   # 带时间戳, 重跑不会盖旧片
  TITLE="JetRover  ·  Autonomous Can Collection  ·  5/5 in 6 min 39 s  ·  Zero Human Intervention"
  L_CEIL="Ceiling / Overhead"
  L_PHONE="Handheld"
  L_RGB="Onboard RGB  ·  detection overlay"
  L_DEP="Onboard depth  ·  0.3-0.9 m"
  L_RVIZ="rviz  ·  map and planning"
else
  OUT="$D/edited/20260729_2144_5can-1station-SUCCESS-5of5/final/$(date +%Y%m%d_%H%M)_5can-1station_5view_zh.mp4"
  TITLE="JetRover 五罐自主收集 · 5/5 全部成功 · 6 分 39 秒 · 全程零人工干预"
  L_CEIL="天花板全局"
  L_PHONE="手持跟拍"
  L_RGB="车载 RGB · 带检测框"
  L_DEP="车载深度 0.3-0.9m"
  L_RVIZ="rviz · 地图与规划"
fi

mkdir -p "$(dirname "$OUT")"

# 每格左上角的小标签: 半透明黑底白字
lab() { echo "drawtext=fontfile=$FONT:text='$1':fontsize=$2:fontcolor=white:x=18:y=14:box=1:boxcolor=black@0.55:boxborderw=10"; }

ffmpeg -y \
  -ss $CEIL_SS  -i "$CEIL" \
  -ss $PHONE_SS -i "$PHONE" \
  -ss $CAR_SS   -i "$RGB" \
  -ss $CAR_SS   -i "$DEPTH" \
  -ss $RVIZ_SS  -i "$RVIZ" \
  -filter_complex "\
[0:v]fps=15,scale=960:540,setsar=1,$(lab "$L_CEIL" 30)[a];\
[1:v]fps=15,scale=960:540,setsar=1,$(lab "$L_PHONE" 30)[b];\
[2:v]fps=15,scale=640:360,setsar=1,$(lab "$L_RGB" 22)[c];\
[3:v]fps=15,scale=640:360,setsar=1,$(lab "$L_DEP" 22)[d];\
[4:v]fps=15,scale=640:360,setsar=1,$(lab "$L_RVIZ" 22)[e];\
color=c=0x141414:s=1920x180:r=15:d=$DUR,drawtext=fontfile=$FONT:text='$TITLE':fontsize=52:fontcolor=white:x=(w-text_w)/2:y=(h-text_h)/2[t];\
[a][b]hstack=inputs=2[top];\
[c][d][e]hstack=inputs=3[bot];\
[t][top][bot]vstack=inputs=3[out]" \
  -map "[out]" -t $DUR \
  -c:v h264_nvenc -preset hq -rc vbr -cq 23 -b:v 0 -pix_fmt yuv420p \
  "$OUT"

echo "=== 完成 ==="
ls -lh "$OUT"
