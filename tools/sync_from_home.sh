#!/usr/bin/env bash
# 把散在家目录各处的 JetRover 项目文件收集进仓库。
# 单向：家目录 -> 仓库。你平时照旧在 ~/ 和 ~/jetrover_grasp/ 里改代码和跑脚本，
# 想提交前跑一次这个，然后 git add/commit。
# 用法: bash tools/sync_from_home.sh
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
H="$HOME"
HANDOFF="$H/jetrover_handoff_20260825"

cp_if() { [ -e "$1" ] && cp -a "$1" "$2" || true; }

mkdir -p "$REPO"/code/{grasp,planning,demo_tools,demo_scripts,ops,car} \
         "$REPO"/code/demo_tools/rviz_archive \
         "$REPO"/run_data/logs "$REPO"/figures "$REPO"/deliverables

# ---------- code/grasp : 抓取 + 导航主力 (~/jetrover_grasp) ----------
rsync -a --delete \
  --exclude '__pycache__/' --exclude '*.log' \
  --exclude 'colleague_final_grab/final_track_and_grab.py' \
  "$H/jetrover_grasp/" "$REPO/code/grasp/"

# ---------- code/planning : 任务规划 / LLM 决策 / 离线仿真 (~/) ----------
for f in bottle_run.py car_run.py clearance_scan.py drone_sim.py make_detections.py \
         mission_review.py mission_run.py mission_sim.py plan_next.py \
         scene_search.py two_station_gap.py; do
  cp_if "$H/$f" "$REPO/code/planning/"
done
rsync -a --exclude '__pycache__/' "$H/llm_demos/" "$REPO/code/planning/llm_demos/" 2>/dev/null || true

# ---------- code/demo_tools : 上帝视角重渲染 / 录屏 / 修片 (~/) ----------
for f in demo_markers.py demo_markers_dynamic.py pub_map.py traj_vs_plan.py \
         fix_truncated_mp4.py rec_rviz.sh watch.rviz \
         pop.sh popstack.sh pop_standoff.sh; do
  cp_if "$H/$f" "$REPO/code/demo_tools/"
done
for f in "$H"/render_*.sh "$H"/replay_*.sh "$H"/start_rviz_*.sh; do
  case "$f" in *.bak_*) continue;; esac
  cp_if "$f" "$REPO/code/demo_tools/"
done
# rviz 布局：当前版进 demo_tools，历次拍摄的旧版进 rviz_archive
for f in "$H"/.rviz/*.rviz; do cp_if "$f" "$REPO/code/demo_tools/"; done
for f in "$H"/.rviz/*.bak_*;  do cp_if "$f" "$REPO/code/demo_tools/rviz_archive/"; done

# ---------- code/demo_scripts : 多视角合成 / 检测框 / 出图 ----------
rsync -a --delete --exclude '__pycache__/' \
  "$H/jetrover_demo/scripts/" "$REPO/code/demo_scripts/"

# ---------- code/ops : 开工一键起栈 ----------
cp_if "$H/jetrover_up.sh" "$REPO/code/ops/"

# ---------- run_data : 任务状态 / 地图 / 路径 / 日志 ----------
for f in "$H"/mission_state*.yaml "$H"/map_0808*.pgm "$H"/map_0808*.yaml \
         "$H"/paths_*.jsonl "$H"/mission_log.jsonl* "$H"/waypoints_*.json \
         "$H"/bottles_detected.json "$H"/drone_feed*.jsonl*; do
  cp_if "$f" "$REPO/run_data/"
done
for f in "$H"/mission_*.log "$H"/jetrover_logs/* "$H"/jetrover_grasp/*.log; do
  cp_if "$f" "$REPO/run_data/logs/"
done
cp_if "$HANDOFF/run_data/mission_run_dynamic5.log" "$REPO/run_data/logs/"

# ---------- figures : 论文定性图 + 排查对比图 ----------
rsync -a "$HANDOFF/figures/" "$REPO/figures/" 2>/dev/null || true
mkdir -p "$REPO/figures/analysis"
rsync -a "$H/jetrover_demo/analysis/" "$REPO/figures/analysis/" 2>/dev/null || true

# ---------- deliverables : 作品集 / teaser deck ----------
rsync -a "$HANDOFF/deliverables/" "$REPO/deliverables/" 2>/dev/null || true
mkdir -p "$REPO/deliverables/portfolio_html" "$REPO/deliverables/ppt"
cp_if "$H/jetrover_portfolio/index.html" "$REPO/deliverables/portfolio_html/"
rsync -a "$H/jetrover_ppt/" "$REPO/deliverables/ppt/" 2>/dev/null || true
cp_if "$H/jetrover_demo/README.md" "$REPO/deliverables/jetrover_demo_README.md"

echo "同步完成。仓库大小：$(du -sh "$REPO" | cut -f1)"
echo "接下来： cd $REPO && git status"
