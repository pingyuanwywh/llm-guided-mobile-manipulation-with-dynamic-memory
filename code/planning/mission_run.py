#!/usr/bin/env python3
"""
mission_run.py -- 主循环: 收无人机坐标 -> 叫 LLM 决策 -> 驱动车收一个罐子 -> 回写记忆 -> 重来。

在**本机**跑(Ollama 在这, 车是 Orin 跑不动 8B), 通过 ssh 驱动车 ——
沿用 car_run.py / bottle_run.py 那套验证过的架构。

## 一轮做什么
  1. 从 ~/drone_feed.jsonl 捞 reveal_at 已到的新罐子, 去重后写进记忆
  2. plan_next.decide() 出决策(几何门在里面, LLM 只在通过门的候选里挑)
  3. 把算出来的 standoff 位姿 ssh 写进车的 llm_nav_places.yaml(set_place.py)
  4. ssh 跑 run_one_can.sh, 按退出码回写状态
  5. 回到 1

## 可打断模式(--interruptible)
把 run_one_can 拆成 --phase nav(空手去接, **唯一能安全打断的阶段**) + --phase rest(抓+送)。
nav 期间每 2 秒捞一次 feed, 有新罐子就重新决策; 决策不是 continue 就 cancel_nav.sh 掉头。
⚠️ "车走到哪了"是按**标称速度估的**(TEB 实测 0.144 m/s), 不查 TF ——
   这个数只喂给"还剩多远 vs 改道白走多远"的粗判断, 不参与任何控制。

用法:
  python3 ~/drone_sim.py --scenario blocker_arrives_late      # 先造 feed
  python3 ~/mission_run.py --state ~/mission_state.yaml --dry-run
  python3 ~/mission_run.py --state ~/mission_state.yaml       # 真车
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time

import yaml

import plan_next

FEED = os.path.expanduser("~/drone_feed.jsonl")
def _env(key, default=""):
    """读现场参数：优先环境变量，其次 ~/.jetrover_env（见仓库 jetrover_env.example）。"""
    v = os.environ.get(key)
    if v:
        return v
    p = os.path.expanduser("~/.jetrover_env")
    if os.path.isfile(p):
        for line in open(p):
            line = line.strip()
            if line.startswith("export %s=" % key):
                val = line.split("=", 1)[1].split("#")[0].strip().strip("\"'")
                pre = "${%s:-" % key          # 认 export CAR_IP="${CAR_IP:-10.x.x.x}" 这种写法
                if val.startswith(pre) and val.endswith("}"):
                    val = val[len(pre):-1]
                return val
    if default:
        return default
    raise SystemExit("缺少 %s：请配置 ~/.jetrover_env（参考仓库 jetrover_env.example）" % key)

CAR = _env("CAR_IP")
USER = _env("CAR_USER", "uavg")
# rosbag: 录真实轨迹, 供以后离线重渲染上帝视角 / 量 TEB 实际轨迹与全局路径的偏差。
# ⚠️ 不录 /map(1.05MB x 2Hz = 2.1MB/s, 会吃掉整块盘), 地图另用 map_saver 存快照。
# 停录必须 SIGINT —— SIGKILL 会让 bag 没写完索引, 要 rosbag reindex 才能用。
BAG_TOPICS = ("/tf /tf_static /scan /cmd_vel /move_base/NavfnROS/plan "
              "/move_base/TebLocalPlannerROS/local_plan "
              "/move_base/TebLocalPlannerROS/global_plan "
              "/move_base/current_goal /move_base/global_costmap/footprint")
# ⚠️ 用 tmux 起, 别用 `setsid nohup ... &` —— 08-08 实测后者的 ssh 命令**不返回**
# (后台进程没和 ssh 通道脱干净), 会把 mission_run 卡死在开跑前。整套栈本来就是 tmux 起的。
BAG_START = ("mkdir -p ~/bags && tmux kill-session -t bag 2>/dev/null; "
             "tmux new-session -d -s bag "
             "'source /opt/ros/noetic/setup.bash && rosbag record -O ~/bags/%s.bag "
             + BAG_TOPICS + "'; sleep 3; pgrep -af 'rosbag record -O' | head -1")
# SIGINT 才会让 rosbag 写完索引; SIGKILL 得事后 `rosbag reindex` 才能用。
# ⚠️ SIGINT 之后必须**等 .active 后缀消失**再关 tmux —— 08-08 实测: 固定 sleep 5 会在
# rosbag 写索引的中途把 session 杀掉, 留下一个 .bag.active(要 `rosbag reindex` 才能用)。
BAG_STOP = ("pkill -INT -f 'rosbag record -O'; "
            "for i in $(seq 1 25); do sleep 1; "
            "ls ~/bags/*.active >/dev/null 2>&1 || break; done; "
            "tmux kill-session -t bag 2>/dev/null; "
            "ls ~/bags/*.active >/dev/null 2>&1 && echo '!! bag 仍是 .active, 需 rosbag reindex' "
            "|| echo 'bag 已正常收尾'; ls -la ~/bags/ | tail -2")
MERGE_R = 0.25          # 同一个罐子被重复上报的合并半径
DEBOUNCE = 2.0          # 连报多个时先攒一下, 别每个都叫一次 LLM
NOMINAL_SPEED = 0.144   # m/s, TEB 实测均值(07-28)
DRY_COMPRESS = 12.0     # dry-run 里把腿的时长压缩多少倍(--dry-compress 可调)


# ---------------- 记忆读写 ----------------

def load(path):
    return yaml.safe_load(open(path)) or {}


def save(path, state):
    with open(path, "w") as f:
        yaml.safe_dump(state, f, allow_unicode=True, default_flow_style=False, sort_keys=False)


def apply_transform(state, x, y, frame):
    """无人机坐标 -> map 系。今天喂的是模拟坐标(frame=map), 变换是恒等。
    真无人机接进来时在 state 里配 transform: {dx, dy, dtheta_deg}(由 ≥2 个共同点解出)。"""
    if frame == "map":
        return x, y
    t = state.get("transform") or {}
    th = math.radians(t.get("dtheta_deg", 0.0))
    return (t.get("dx", 0.0) + x * math.cos(th) - y * math.sin(th),
            t.get("dy", 0.0) + x * math.sin(th) + y * math.cos(th))


def ingest(state, feed_path, seen):
    """捞 reveal_at 已到、还没吃过的行。返回新增的罐子名列表。"""
    if not os.path.exists(feed_path):
        return []
    now = time.time()
    state.setdefault("cans", {})
    added = []
    for i, line in enumerate(open(feed_path)):
        line = line.strip()
        if not line or i in seen:
            continue
        rec = json.loads(line)
        if rec.get("reveal_at", 0) > now:
            continue
        seen.add(i)
        x, y = apply_transform(state, rec["x"], rec["y"], rec.get("frame", "map"))

        # 去重: 同一个罐子飞两遍会报两次
        dup = None
        for n, c in state["cans"].items():
            if math.hypot(c["x"] - x, c["y"] - y) < MERGE_R:
                dup = n
                break
        if dup:
            print("   (无人机重报 %s, 距已知 %s 不到 %.2fm, 忽略)" % (rec["id"], dup, MERGE_R))
            continue

        name = rec["id"]
        state["cans"][name] = {
            "x": round(x, 4), "y": round(y, 4), "color": rec.get("color", "red"),
            "source": "drone", "status": "pending", "attempts": 0,
            "seen_at": time.strftime("%H:%M:%S"),
        }
        added.append(name)
    return added


def real_clearance(legs, cans):
    """把所有候选腿一次性丢给车上的 plan_clearance.py, 拿 move_base 的**真实全局路径**算净空。

    ⚠️ 08-01 加的。原来拿直线近似车的路径, 实测一条"净空 0.48m"的腿把罐子撞倒了 ——
       TEB 全向走曲线还会横移绕障, 偏离量和净空同一个量级。
    一次 ssh 跑完全部腿(每条 make_plan 是毫秒级), 不是一条一条问。
    服务不在 / 出错 => 抛异常, plan_next 自己退回直线模型(保守度不同, 但不会崩)。
    """
    payload = json.dumps({"legs": legs, "cans": cans}, ensure_ascii=False)
    p = subprocess.run(["ssh", "-o", "BatchMode=yes", "%s@%s" % (USER, CAR),
                        "source /opt/ros/noetic/setup.bash; "
                        "source ~/ros1_ws/devel/setup.bash; python3 ~/plan_clearance.py"],
                       input=payload, capture_output=True, text=True, timeout=120)
    if p.returncode != 0:
        raise RuntimeError("plan_clearance 退出码 %d: %s" % (p.returncode, p.stderr.strip()[:200]))
    out = json.loads(p.stdout.strip().splitlines()[-1])
    if not out.get("ok"):
        raise RuntimeError(out.get("error", "未知错误"))
    return out["legs"]


# ---------------- 驱动车 ----------------

def ssh(cmd, dry, timeout=600):
    if dry:
        print("   [dry] ssh %s@%s %s" % (USER, CAR, cmd))
        return 0, ""
    p = subprocess.run(["ssh", "-o", "BatchMode=yes", "%s@%s" % (USER, CAR), cmd],
                       capture_output=True, text=True, timeout=timeout)
    if p.stdout:
        print(p.stdout.rstrip())
    if p.returncode != 0 and p.stderr:
        print(p.stderr.rstrip(), file=sys.stderr)
    return p.returncode, p.stdout


def push_place(entry, dry):
    """把算出来的 standoff 写进车上的 llm_nav_places.yaml —— 和手工录点同一个文件同一个格式。"""
    x, y, yawdeg = entry["standoff"]
    return ssh("python3 ~/set_place.py %s %.4f %.4f %.2f" % (entry["name"], x, y, yawdeg), dry)[0]


def push_station(name, xy, dry):
    """收集站也得在 yaml 里(nav_goto --place 要用)。yaw 用"从场地中心看向站"的反方向没意义,
    这里用 0 —— 站是矮盒, --face 转到什么朝向都不影响 place(place 是朝臂基座正前方伸手,
    车到站的朝向由录点决定)。⚠️ 所以**收集站还是得手工录**, 这里只是 dry-run 时占位。"""
    return ssh("python3 ~/set_place.py %s %.4f %.4f 0" % (name, xy[0], xy[1]), dry)[0]


# 退出码 -> (状态, note)
RC = {
    0:  ("collected", None),
    2:  ("failed", "导航到不了它(可能路被挡或目标点在障碍里)"),
    3:  ("failed", "没抓住(空夹)。罐身通体是色的, 所以多半是配料表/条形码那面正对相机, 色块外接框中心偏离了罐子轴心 —— 重试时接近方向会自动换 60 度"),
    4:  ("stuck",   "夹着罐子但回站导航失败, 车手里还有东西, 需要人工介入"),
    5:  ("failed", "place 服务失败"),
}


def run_leg(entry, station, dry, interruptible, state, feed, seen, statepath):
    """跑一条腿。interruptible 时先 --phase nav(可打断)再 --phase rest。
    返回 (rc, 是否被打断)。"""
    name, color = entry["name"], entry["color"]
    if not interruptible:
        return ssh("bash ~/run_one_can.sh %s %s %s" % (name, color, station), dry)[0], False

    # --- 可打断的那段: 空手去接罐子 ---
    origin = plan_next.robot_pos(state, plan_next.stations_of(state))
    state["active"] = {"can": name, "phase": "nav", "from": list(origin)}
    save(statepath, state)

    cmd = "bash ~/run_one_can.sh %s %s %s --phase nav" % (name, color, station)
    if dry:
        print("   [dry] ssh (后台) %s" % cmd)
        proc = subprocess.Popen([sys.executable, "-c",
                                 "import time;time.sleep(%f)" % _dry_leg_seconds(origin, entry)])
    else:
        proc = subprocess.Popen(["ssh", "-o", "BatchMode=yes", "%s@%s" % (USER, CAR), cmd])

    t0 = time.time()
    while proc.poll() is None:
        time.sleep(2.0)
        if ingest(state, feed, seen):
            time.sleep(DEBOUNCE)
            ingest(state, feed, seen)
            # 按标称速度估车走到哪了 —— 只喂给粗判断, 不参与控制
            travelled = min(NOMINAL_SPEED * (time.time() - t0),
                            math.hypot(origin[0] - entry["standoff"][0],
                                       origin[1] - entry["standoff"][1]))
            head = math.atan2(entry["standoff"][1] - origin[1], entry["standoff"][0] - origin[0])
            state["robot"] = {"x": origin[0] + travelled * math.cos(head),
                              "y": origin[1] + travelled * math.sin(head), "holding": None}
            save(statepath, state)
            print("   >> 途中收到新目标, 重新决策...")
            d, _, _ = plan_next.decide(state)
            print("   >> %s : %s" % (d["next"], d["reason"]))
            if d["next"] != "continue":
                ssh("bash ~/cancel_nav.sh", dry)
                proc.terminate()
                proc.wait(timeout=30)
                state["active"] = None
                save(statepath, state)
                return None, True

    rc = proc.returncode
    if rc not in (0, 10):
        state["active"] = None
        return rc, False

    # --- 不可打断的那段: 抓 + 送 ---
    state["active"] = {"can": name, "phase": "grab"}
    save(statepath, state)
    rc = ssh("bash ~/run_one_can.sh %s %s %s --phase rest" % (name, color, station), dry)[0]
    state["active"] = None
    return rc, False


def _dry_leg_seconds(origin, entry):
    d = math.hypot(origin[0] - entry["standoff"][0], origin[1] - entry["standoff"][1])
    return max(2.0, d / NOMINAL_SPEED / DRY_COMPRESS)


# ---------------- 主循环 ----------------

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", default=os.path.expanduser("~/mission_state.yaml"))
    ap.add_argument("--feed", default=FEED)
    ap.add_argument("--dry-run", action="store_true", help="不连车, 模拟执行结果")
    ap.add_argument("--real-clearance", action="store_true",
                    help="dry-run 时也用 move_base 真实路径算净空(预览要和真跑门控一致就加它)")
    ap.add_argument("--interruptible", action="store_true", help="允许空手导航途中改道")
    ap.add_argument("--fail", default="", help="dry-run 用: 让这些罐子第一次抓失败, 逗号分隔")
    ap.add_argument("--wait-empty", type=float, default=60.0,
                    help="队列空了再等多少秒没有新目标就收工")
    ap.add_argument("--max-legs", type=int, default=20)
    ap.add_argument("--record", default="",
                    help="车载 RGB+深度 录制的 tag。开录和停录都由本脚本负责(try/finally), "
                         "不靠人记得 —— 08-01 两次栽在忘开/忘停上。")
    ap.add_argument("--order", default="",
                    help='把 LLM 从决策里摘掉, 按写死的顺序跑, 但 standoff 仍然由几何算。'
                         '格式 "can4:collect_b,can5:collect_b,..." 。'
                         '⚠️ 08-01: "写死顺序"和"算 standoff"是两件事, 别再一起退回手工录点 ——'
                         'can2 就是因为沿用录点方向(在罐子另一侧)被车辗倒的。')
    ap.add_argument("--dry-compress", type=float, default=12.0,
                    help="dry-run 里腿的时长压缩倍数; 调小才看得到途中打断")
    a = ap.parse_args()

    global DRY_COMPRESS
    DRY_COMPRESS = a.dry_compress
    if not a.dry_run or a.real_clearance:
        plan_next.REAL_CLEARANCE = real_clearance      # 净空问 move_base 要真实路径
    state = load(a.state)
    statepath = a.state
    seen = set()
    fail_once = set(x for x in a.fail.split(",") if x)
    legs = 0
    empty_since = None

    if a.record and not a.dry_run:
        print("+ bash ~/rec.sh start %s" % a.record)
        ssh("bash ~/rec.sh start %s" % a.record, False)
        # rosbag: 留下真实轨迹, 以后可离线重渲染上帝视角(rviz 画的是实时话题, 不录就补不回来)。
        # ⚠️ 排除 /map —— 单条 1.05MB x 2Hz = 2.1MB/s, 十几分钟就 1.5G; 地图另用 map_saver 存快照。
        print("+ rosbag record %s" % a.record)
        ssh(BAG_START % a.record, False)
    print("=== 任务开始  模式=%s  站=%s  %s ==="
          % (state.get("mode", "single"), list(plan_next.stations_of(state)),
             "DRY-RUN" if a.dry_run else "真车 %s@%s" % (USER, CAR)))
    print("    告警线 %.2fm / 必撞线 %.3fm   净空来源: %s"
          % (plan_next.WARN, plan_next.HARD,
             "move_base 真实路径" if plan_next.REAL_CLEARANCE else "直线近似"))

    while legs < a.max_legs:
        added = ingest(state, a.feed, seen)
        if added:
            time.sleep(DEBOUNCE if not a.dry_run else 0)
            added += ingest(state, a.feed, seen)
            print("\n📡 无人机报来: %s" % ", ".join(added))
            empty_since = None
        save(statepath, state)

        if a.order:
            # 写死顺序: 跳过 LLM, 但几何门照常跑(standoff 还是算出来的)
            clear, blocked = plan_next.plan_candidates(state)
            nxt = None
            for item in a.order.split(","):
                cn, _, sn = item.strip().partition(":")
                c = state.get("cans", {}).get(cn)
                if c and c.get("status") in ("pending", "failed"):
                    nxt = (cn, sn or None)
                    break
            if nxt is None:
                decision = {"next": "done", "reason": "写死的顺序跑完了"}
            elif not any(x["name"] == nxt[0] for x in clear):
                b = next((x for x in blocked if x["name"] == nxt[0]), None)
                print("\n🛑 写死顺序里的下一个 %s **过不了几何门**%s"
                      % (nxt[0], ("(净空 %.2f, 被 %s 挡)" % (b["clearance"], b["blocked_by"])) if b else ""))
                print("   不硬跑 —— 08-01 就是硬跑把 can2 辗倒的。停下让人看。")
                decision = {"next": "done", "reason": "%s 过不了几何门" % nxt[0]}
            else:
                decision = {"next": nxt[0], "reason": "写死的顺序"}
                if nxt[1]:
                    decision["station"] = nxt[1]
        else:
            decision, clear, blocked = plan_next.decide(state)
        for b in blocked:
            print("   ⛔ %s 暂时去不了(净空 %.2f, 被 %s 挡)"
                  % (b["name"], b["clearance"], b["blocked_by"]))

        if decision["next"] == "done":
            live = [n for n, c in (state.get("cans") or {}).items()
                    if c.get("status") in ("pending", "failed")]
            if not live:
                if empty_since is None:
                    empty_since = time.time()
                    print("\n⏳ 手头没活了, 再等 %.0fs 看无人机还报不报..." % a.wait_empty)
                if time.time() - empty_since > a.wait_empty:
                    print("\n=== 收工: 等了 %.0fs 没有新目标 ===" % a.wait_empty)
                    break
                time.sleep(3)
                continue
            print("\n=== 收工: %s ===" % decision["reason"])
            break

        if decision["next"] == "continue":
            # 顶层循环里 active 应该是 None, 走到这说明状态没清干净。别硬跑, 清掉重来。
            print("   (顶层收到 continue, 清掉 active 重新决策)")
            state["active"] = None
            save(statepath, state)
            continue

        pick = next(c for c in clear if c["name"] == decision["next"])
        station = decision.get("station") or pick["station"]
        print("\n🧠 [%d] → %s (%s) 送 %s" % (legs + 1, pick["name"], pick["color"], station))
        print("   理由: %s" % decision["reason"])
        if pick.get("label_warning"):
            print("   ⚠️  %s" % pick["label_warning"])

        push_place(pick, a.dry_run)

        if a.dry_run:
            rc = 3 if pick["name"] in fail_once else 0
            fail_once.discard(pick["name"])
            interrupted = False
            if a.interruptible:
                rc2, interrupted = run_leg(pick, station, True, True, state,
                                           a.feed, seen, statepath)
                if not interrupted:
                    rc = rc2 if rc2 not in (0, 10) else rc
        else:
            rc, interrupted = run_leg(pick, station, False, a.interruptible, state,
                                      a.feed, seen, statepath)

        if interrupted:
            print("   ↩️  已打断, 重新决策")
            continue

        legs += 1
        status, note = RC.get(rc, ("failed", "未知退出码 %s" % rc))
        c = state["cans"][pick["name"]]
        c["status"] = status
        c["attempts"] = c.get("attempts", 0) + 1
        if note:
            c["note"] = note
        if status == "collected":
            c["station"] = station
            print("   ✅ 收进 %s" % station)
        else:
            print("   ❌ %s: %s" % (status, note))
        # 车现在停在站上
        st = plan_next.stations_of(state)
        if station in st:
            state["robot"] = {"x": st[station][0], "y": st[station][1],
                              "holding": pick["name"] if status == "stuck" else None,
                              "at": station}
        state["active"] = None
        save(statepath, state)

        if status == "stuck":
            print("\n🛑 车夹着罐子停住了, 需要人工介入。停。")
            break

    if a.record and not a.dry_run:
        print("--- 停止车载录制 ---")
        ssh("bash ~/rec.sh stop", False)
        ssh(BAG_STOP, False)
    cans = state.get("cans") or {}
    ok = [n for n, c in cans.items() if c.get("status") == "collected"]
    print("\n=== 结束: 收进 %d/%d  (%s) ===" % (len(ok), len(cans), ", ".join(sorted(ok))))
    print("决策日志: %s" % plan_next.LOG)


if __name__ == "__main__":
    try:
        main()
    except BaseException:
        # ⚠️ 出错/Ctrl-C 也必须把录制收尾, 否则 H.264 写不完 moov 就打不开(08-01 实测)
        import sys as _s
        if "--record" in _s.argv and "--dry-run" not in _s.argv:
            print("!! 异常退出, 补一次停录")
            try:
                ssh("bash ~/rec.sh stop", False)
            except Exception:
                pass
            try:
                ssh(BAG_STOP, False)
            except Exception:
                pass
        raise
