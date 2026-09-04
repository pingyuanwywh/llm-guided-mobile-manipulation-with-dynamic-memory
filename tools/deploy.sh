#!/usr/bin/env bash
# 把仓库里的文件铺回**运行时布局** —— 本机 ~/ 和车上 ~/。
#
# 为什么需要这个脚本:
#   仓库的 code/grasp、code/planning 分层是**为了读**, 不是运行时布局。
#   所有脚本互相调用写的都是 ~/foo.py。所以 git clone 完**一个都跑不起来**,
#   必须先把文件铺回家目录。sync_from_home.sh 只有"家目录 -> 仓库"单向, 这个是反向。
#
# 用法:
#   bash tools/deploy.sh                 # 演练: 只打印会写哪些文件, 不落盘  <- 默认
#   bash tools/deploy.sh --apply         # 真写(本机 + 车)
#   bash tools/deploy.sh --host --apply  # 只铺本机
#   bash tools/deploy.sh --car  --apply  # 只铺车上
#   bash tools/deploy.sh --calib --apply # 连现场数据一起铺(危险, 见下)
#
# 车是代码的真值来源。铺车之前会先比对内容, 只要车上有比仓库新的版本就**停下**,
# 并打印把它拉回仓库的命令。确实要用仓库版盖掉车上, 才加 --force-car。
#
# ⚠️ 现场数据默认**不铺**(要 --calib 才铺):
#     llm_nav_places.yaml  航点   —— 铺过去会覆盖车上录好的点
#     imu_bias.yaml        零偏   —— 会温漂, 设计上就要每次现测, 铺陈旧值有害
#     mission_state*.yaml  任务状态 / mission_log.jsonl 账本 / map_* 地图 / paths_* 路径
#   这些是"我的车、我的场地"的值, 换车换场地必须重做, 不是能拷贝的东西。
#
# 覆盖前一律自动备份成 <文件>.bak_deploy_<日期时间>, 不会静默盖掉任何东西。
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="$HOME"
STAMP="$(date +%m%d_%H%M)"
BK=(--backup --suffix=".bak_deploy_${STAMP}")

APPLY=0; DO_HOST=0; DO_CAR=0; DO_CALIB=0; FORCE_CAR=0
for a in "$@"; do case "$a" in
  --apply) APPLY=1;;
  --host)  DO_HOST=1;;
  --car)   DO_CAR=1;;
  --calib) DO_CALIB=1;;
  --force-car) FORCE_CAR=1;;
  -h|--help) sed -n '2,25p' "$0"; exit 0;;
  *) echo "不认识的参数: $a  (--host --car --calib --apply --force-car)"; exit 1;;
esac; done
# 没指定目标就两边都铺
[ $DO_HOST -eq 0 ] && [ $DO_CAR -eq 0 ] && { DO_HOST=1; DO_CAR=1; }

if [ $APPLY -eq 0 ]; then
  RS=(rsync -a --checksum --itemize-changes --dry-run "${BK[@]}")
  echo "=== 演练模式 (没有 --apply, 不会写任何文件) ==="
else
  RS=(rsync -a --checksum --itemize-changes "${BK[@]}")
  echo "=== 真写模式 —— 覆盖的文件会备份成 *.bak_deploy_${STAMP} ==="
fi
echo

note(){ printf '\n\033[1m-- %s\033[0m\n' "$*"; }

# ============================ 本机 ============================
if [ $DO_HOST -eq 1 ]; then
  note "本机: 抓取/导航脚本 -> ~/jetrover_grasp/"
  "${RS[@]}" --exclude '__pycache__/' "$REPO/code/grasp/" "$H/jetrover_grasp/"

  note "本机: 规划 + LLM 决策 -> ~/"
  "${RS[@]}" --exclude '__pycache__/' --exclude 'llm_demos/' "$REPO/code/planning/"*.py "$H/"
  [ -d "$REPO/code/planning/llm_demos" ] && \
    "${RS[@]}" --exclude '__pycache__/' "$REPO/code/planning/llm_demos/" "$H/llm_demos/"

  note "本机: demo 工具 -> ~/"
  "${RS[@]}" "$REPO/code/demo_tools/"*.py "$REPO/code/demo_tools/"*.sh "$H/"
  "${RS[@]}" "$REPO/code/demo_tools/watch.rviz" "$H/"

  note "本机: rviz 布局 -> ~/.rviz/"
  mkdir -p "$H/.rviz"
  "${RS[@]}" "$REPO/code/demo_tools/"jetrover_*.rviz "$H/.rviz/"
  [ -d "$REPO/code/demo_tools/rviz_archive" ] && \
    "${RS[@]}" "$REPO/code/demo_tools/rviz_archive/" "$H/.rviz/"

  note "本机: 多视角合成脚本 -> ~/jetrover_demo/scripts/"
  "${RS[@]}" --exclude '__pycache__/' "$REPO/code/demo_scripts/" "$H/jetrover_demo/scripts/"

  note "本机: 开工起栈脚本 -> ~/"
  "${RS[@]}" "$REPO/code/ops/jetrover_up.sh" "$H/"

  if [ $DO_CALIB -eq 1 ]; then
    note "本机: ⚠️ 现场数据(任务状态/地图/路径/账本) -> ~/"
    "${RS[@]}" "$REPO/run_data/"mission_state*.yaml "$REPO/run_data/"map_* \
               "$REPO/run_data/"paths_*.jsonl "$REPO/run_data/"*.json "$H/" 2>/dev/null || true
  else
    note "本机: 跳过现场数据(任务状态/地图/路径)。要铺加 --calib"
  fi
fi

# ============================ 车上 ============================
if [ $DO_CAR -eq 1 ]; then
  [ -f "$H/.jetrover_env" ] && . "$H/.jetrover_env"
  CAR_IP="${CAR_IP:?未设置 CAR_IP：先 cp jetrover_env.example ~/.jetrover_env 并填车 IP}"
  CAR="${CAR_USER:-uavg}@${CAR_IP}"

  if ! ssh -o BatchMode=yes -o ConnectTimeout=8 "$CAR" true 2>/dev/null; then
    echo "!! ssh 连不上 $CAR —— 车没开机 / IP 变了 / 没配免密。跳过车上部分。"
  else
    # ---- 飞行前检查: 车上有没有比仓库更新的版本? ----
    # ⚠️ 这一段是 2026-09-04 用血换来的: 仓库里 standoff.sh 停在 08-01, 车上是 08-29
    #    (已经改成支持 N 个罐子), 无脑 deploy 会拿旧版盖掉车上的改进。
    #    车是这些文件的真值来源 —— 仓库落后是常态, 因为 sync_from_home.sh 收的是
    #    本机 ~/jetrover_grasp/ 那份暂存副本, 它自己也会落后于车。
    note "车上: 飞行前检查 —— 比对内容, 看哪边更新"
    CAND=$(ls "$REPO/code/grasp/"*.py "$REPO/code/grasp/"*.sh "$REPO/code/grasp/"*.launch \
              "$REPO/code/car/"*.py "$REPO/code/car/"*.sh "$REPO/code/car/"*.launch 2>/dev/null)
    CARINFO=$(ssh -o BatchMode=yes "$CAR" 'cd ~ && for f in *.py *.sh *.launch; do
                [ -f "$f" ] && echo "$f $(md5sum <"$f" | cut -d" " -f1) $(stat -c %Y "$f")"; done' 2>/dev/null)
    NEWER=""; DIFFER=""
    for src in $CAND; do
      b=$(basename "$src")
      line=$(printf '%s\n' "$CARINFO" | awk -v b="$b" '$1==b{print; exit}')
      [ -z "$line" ] && continue                      # 车上没有 => 新增, 不冲突
      cmd5=$(echo "$line" | cut -d' ' -f2); cmt=$(echo "$line" | cut -d' ' -f3)
      rmd5=$(md5sum <"$src" | cut -d' ' -f1);  rmt=$(stat -c %Y "$src")
      [ "$cmd5" = "$rmd5" ] && continue               # 内容一样 => 不用管
      DIFFER="$DIFFER $b"
      [ "$cmt" -gt "$rmt" ] && NEWER="$NEWER $b"
    done
    if [ -n "$NEWER" ]; then
      echo "  ⛔ 车上这些文件比仓库**更新**, 铺过去会把车上的改动盖掉:"
      for b in $NEWER; do echo "       $b"; done
      echo
      echo "  正确做法是先把车上的真值拉回仓库:"
      echo "      rsync -a $(for b in $NEWER; do printf '%s:~/%s ' "$CAR" "$b"; done)\\"
      echo "               $REPO/code/grasp/    # 或 code/car/, 看它原来在哪"
      echo "  确实要用仓库版本覆盖车上, 加 --force-car。"
      [ "${FORCE_CAR:-0}" -eq 1 ] || { echo; echo "已停在这里, 没有动车上任何文件。"; exit 3; }
      echo "  (--force-car 已给, 继续)"
    elif [ -n "$DIFFER" ]; then
      echo "  以下文件两边不同, 但仓库更新, 会覆盖(有备份):$DIFFER"
    else
      echo "  两边一致, 只会补车上缺的文件。"
    fi

    note "车上: 抓取 + 导航脚本 -> ${CAR}:~/"
    # 只铺代码和 launch。.bak_* 是早于 git 的历史版本, 样本数据/图片没必要上车。
    "${RS[@]}" --exclude '*.bak_*' --exclude '__pycache__/' \
      $(ls "$REPO/code/grasp/"*.py "$REPO/code/grasp/"*.sh "$REPO/code/grasp/"*.launch 2>/dev/null) \
      "$CAR:~/"

    note "车上: 只跑在车上的那批 -> ${CAR}:~/"
    "${RS[@]}" $(ls "$REPO/code/car/"*.py "$REPO/code/car/"*.sh "$REPO/code/car/"*.launch 2>/dev/null) \
      "$REPO/code/car/ekf.yaml" "$CAR:~/"

    if [ $DO_CALIB -eq 1 ]; then
      note "车上: ⚠️⚠️ 现场标定(航点/零偏/瓶子记忆) -> ${CAR}:~/"
      echo "   航点会被覆盖成仓库里那份 —— 那是别的场地录的, 十有八九不对。"
      "${RS[@]}" "$REPO/code/car/llm_nav_places.yaml" "$REPO/code/car/imu_bias.yaml" \
                 "$REPO/code/car/bottles.yaml" "$CAR:~/"
    else
      note "车上: 跳过现场标定(航点/零偏)。这是**对的默认** —— 换场地必须重录点、重测零偏"
    fi
  fi
fi

echo
if [ $APPLY -eq 0 ]; then
  echo "以上是演练。确认没问题后加 --apply 真写。"
else
  echo "铺完了。接下来:"
  echo "  1) 没有 ~/.jetrover_env 的话先建: cp $REPO/jetrover_env.example ~/.jetrover_env"
  echo "  2) 车上录点(换了场地必做): bash ~/pop_standoff.sh"
  echo "  3) 开工体检: bash ~/jetrover_up.sh"
fi
