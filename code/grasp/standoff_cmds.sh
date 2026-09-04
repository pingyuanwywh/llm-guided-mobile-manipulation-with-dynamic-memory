# standoff_cmds.sh -- 录点用的极简命令 (2026-07-29 第四版: 颜色改成可选参数)
# source 进交互 shell 后, 开到罐子跟前直接敲 can1 / can2 / ... / collect。
#
# ## 颜色怎么给
#   can1          -> 用默认色 red
#   can1 green    -> 这个位置放的是绿罐, 现给
# 默认改成 red 是因为场景重摆后大多数是红罐; 哪个位置是绿的, 敲的时候跟一个 green。
# **起因(07-29 现场)**: 第三版把颜色按位置写死(can1=green), 场景一重摆, can1 位置换成了红罐,
# 敲 can1 就拿 green 去找红罐 -> approach 检不到 -> 白跑一趟搜索。
# **位置和颜色本来是两件事, 绑死在别名里是错的。**
#
# 这些函数**只是 standoff.sh 的别名**, 不含任何判断逻辑。
# standoff.sh 本身也只是把三条验证过的命令透明串起来(每条原样打印再执行)。
# 见 feedback_use_existing_commands 记忆 —— 07-28 就是在这里造轮子翻的车。

_S=/home/uavg/standoff.sh

can1()    { bash $_S can1 "${1:-red}"; }
can2()    { bash $_S can2 "${1:-red}"; }
can3()    { bash $_S can3 "${1:-red}"; }
can4()    { bash $_S can4 "${1:-red}"; }
can5()    { bash $_S can5 "${1:-red}"; }
collect()   { bash $_S collect;   }   # 单站时用
collect_a() { bash $_S collect_a; }   # 双站: A 站
collect_b() { bash $_S collect_b; }   # 双站: B 站

# 键盘遥控。录点会先把它杀掉(它持续灌 /cmd_vel 会和逼近抢总线), 所以每录完一个敲一下 tel 继续开。
tel() { source /opt/ros/noetic/setup.bash; python3 ~/teleop.py; }

# 看已录了哪些、还差哪些
points() {
  python3 - <<'PY'
import os, math, yaml
f = os.path.expanduser('~/llm_nav_places.yaml')
if not os.path.exists(f):
    print('还一个点都没录'); raise SystemExit
d = (yaml.safe_load(open(f)) or {}).get('places', {}) or {}
if not d:
    print('还一个点都没录'); raise SystemExit
print('已录 %d 个点:' % len(d))
for k in sorted(d):
    v = d[k]
    print('  %-10s x=%+.3f y=%+.3f yaw=%+.0f deg'
          % (k, v.get('x', 0), v.get('y', 0), math.degrees(v.get('yaw', 0))))
import os as _os
need = ['can1', 'can2', 'can3', 'can4', 'can5']
need += ['collect_a', 'collect_b'] if _os.environ.get('TWO_STATION') else ['collect']
miss = [n for n in need if n not in d]
print('还差: ' + (', '.join(miss) if miss else '无, 六个点齐了'))
PY
}

# 录错了删掉重来
drop() {
  [ -z "$1" ] && { echo '用法: drop can3'; return 1; }
  python3 - "$1" <<'PY'
import os, sys, yaml
f = os.path.expanduser('~/llm_nav_places.yaml')
d = yaml.safe_load(open(f)) or {}
if d.get('places', {}).pop(sys.argv[1], None) is None:
    print('没有这个点: ' + sys.argv[1])
else:
    yaml.safe_dump(d, open(f, 'w'), default_flow_style=False)
    print('已删除 ' + sys.argv[1])
PY
}

h() {
  cat <<'EOF'
  tel                                开键盘遥控 (w前 s后 a左 d右 x停 q退出)
  can1 .. can5                       录点(默认按红罐): 逼近对准 -> 退8cm -> 落盘 -> 回读
  can3 green                         这个位置摆的是绿罐, 就跟一个 green
  collect                            录收集点(单站, 不看颜色)
  collect_a / collect_b              录两个收集站(双站, 不看颜色)
                                     (每条命令会原样打印; 录完敲 tel 继续开)
  points                             看已录哪些、还差哪些
  drop can3                          录错了删掉
  h                                  再看一次这张表
EOF
}
h
