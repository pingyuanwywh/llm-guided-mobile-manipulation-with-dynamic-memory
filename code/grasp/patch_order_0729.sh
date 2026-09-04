#!/bin/bash
# 改 run_collect5.sh 末尾的收集顺序 + 颜色 (2026-07-29 场景重摆后)
#  - 颜色: can3=green, 其余 red (旧版写死 can1=green, 场景一换就白跑一趟)
#  - 顺序: can1 can3 can5 can2 can4
#    理由: 罐子矮 => 雷达扫不到 => costmap 里没有 => 直线会撞飞。
#    几何检查(本机 scratchpad/geom_check.py): 旧顺序最小净空 0.39m, 这个顺序 1.33m。
#    每罐都是 collect 往返 => 总路程与顺序无关 => 换顺序零代价。
set -e
F=/home/uavg/run_collect5.sh
# 备份是第一次跑本脚本时存的原件, 这里先恢复回去再改 —— 不要再 cp 覆盖它。
cp "${F}.bak_0729_beforeorder" "$F"

# ⚠️ 必须写成 '^run_can can[0-9]' 只删**调用**行。
# 第一版写的 '^run_can ' 把函数定义行 `run_can () {` 也一起删了, 留下孤儿 `}` => 语法错误。
sed -i '/^run_can can[0-9]/d' "$F"
sed -i '/ALL DONE/d' "$F"

cat >> "$F" <<'EOF'
run_can can1 red
run_can can3 green
run_can can5 red
run_can can2 red
run_can can4 red
echo "===================== ALL DONE  $(date +%H:%M:%S) ====================="
EOF

bash -n "$F" && echo "=== 语法 OK ==="
echo "=== 末尾 8 行 ==="
tail -8 "$F"
