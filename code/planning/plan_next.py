#!/usr/bin/env python3
"""
plan_next.py -- 动态局部重规划: 读任务记忆 -> 构造候选 -> LLM 挑下一个动作。

## 分层安全边界(别打破)
    确定性层(本文件上半)          LLM 层(下半)              执行层(车上, 已验证不动)
    合成 standoff 位姿      ->    只在**通过几何门的**  ->  run_one_can.sh
    几何净空检查                  候选里挑一个
⇒ LLM 挑错顶多路径不优, **撞不了罐子** —— 会撞的选项压根没进 enum。
   依据: enum 只保证格式和词汇合法**不保证语义**(07-01 实测); temperature=0 **不保证确定性**(07-08 实测)。

## 三种模式(state 里的 mode 字段)
  single  单站(07-29 那套现状)。总路程 = 2*sum|站-罐|, **与顺序无关** ⇒ 没有优化空间。
  free    双站自由选。放完可去任一站, 选站决定下一条腿从哪起步 ⇒ 顺序与选站耦合。
  sorted  双站按颜色分类(站由颜色定死, 只有顺序自由)。

## 罐子只给 x/y 就够 —— yaw 不是罐子的属性
录点录的 yaw 是"我打算从哪边开过去", 不是罐子的朝向。
几何关系已被 07-29 两次 5/5 标定死: 罐子 = standoff + 0.45m * [cos yaw, sin yaw], 反过来用就是合成位姿。
**首选接近方向 = 从"这个罐子要送去的那个站"指向罐子** —— 这样每个罐子的接近方向是固定的,
物理上才摆得出来(标签朝那个站就行)。sorted 模式下这条最干净: 红罐标签朝 A, 绿罐标签朝 B。

用法:
    python3 ~/plan_next.py --state ~/mission_state.yaml
    python3 ~/plan_next.py --state ... --no-llm     # 只看确定性那半
    python3 ~/plan_next.py --state ... --json       # 只吐决策 JSON, 给 mission_run.py 读
"""
import argparse
import datetime
import json
import math
import os
import sys
import urllib.error
import urllib.request

import yaml

# ---- 标定常数(全部来自实测) ----
CAN_AHEAD = 0.45   # standoff 前方多远是罐子 = 逼近环收敛到臂基座 x=0.35 后再 --nudge -0.08
ROBOT_R = 0.16     # 车体半径
CAN_R = 0.033      # 罐子半径 (Ø66mm)
HARD = ROBOT_R + CAN_R   # 0.193 必撞线
WARN = 0.50              # 告警线, 低于此不进 enum。
# ⚠️ 08-01 从 0.30 抬到 0.50: 一条**直线净空 0.48m** 的腿, 车实际把罐子撞倒了。
# 直线是 TEB 实际路径的粗糙近似 —— TEB 全向、走曲线、还会主动横移绕障,
# 07-28 测绕障时量到过横向最大偏离 0.48m, 和净空同一个量级。
# 抬阈值只是顶上去的一半, 另一半是 REAL_CLEARANCE(问 move_base 要真实路径)。

SELF_MIN = 0.30          # 去程路径离**目标罐自己**最近不得小于此。
# ⚠️ 08-01 血的教训: standoff 若落在罐子的另一侧, 去程会从罐子身上正面压过去(实测 0.039m)。
# 干净的接近全程离目标罐 >=0.45m(standoff 就在 0.45m), 所以 0.30 既拦得住压过去、又不误伤。
YAW_PREF_TOL = math.radians(45)   # 偏离首选方向超过这个就提醒用户转标签
YAW_SEARCH = math.radians(90)
YAW_STEP = math.radians(15)

OLLAMA_URL = "http://localhost:11434/api/chat"
MODEL = "qwen3:8b"
LOG = os.path.expanduser("~/mission_log.jsonl")

# 真实净空钩子。mission_run 跑真车时把它设成"ssh 到车上跑 plan_clearance.py"的函数;
# 离线 dry-run / mission_sim 保持 None => 退回直线模型(够用, 只是保守度不同)。
# 签名: fn(legs, cans) -> {leg_id: {"min": float|None, "blocker": str|None}}
#   legs = [{"id":..., "start":[x,y,yaw], "goal":[x,y,yaw]}]
REAL_CLEARANCE = None


# ================= 确定性层 =================

def seg_pt_dist(a, b, p):
    """点 p 到线段 ab 的最近距离。(与 geom_check.py 同一份逻辑)"""
    ax, ay = a
    bx, by = b
    px, py = p
    dx, dy = bx - ax, by - ay
    L2 = dx * dx + dy * dy
    if L2 == 0:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / L2))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def stations_of(state):
    """返回 {名字: (x, y, accepts列表)}。兼容旧 schema 里的单个 collect。"""
    st = state.get("stations")
    if st:
        return {n: (s["x"], s["y"], s.get("accepts")) for n, s in st.items()}
    c = state["collect"]
    return {"collect": (c["x"], c["y"], None)}


def station_for(name, color, state, stations):
    """这个罐子该送去哪个站。sorted 由颜色定死; single 只有一个; free 返回 None(交给 LLM)。"""
    mode = state.get("mode", "single")
    if len(stations) == 1:
        return next(iter(stations))
    if mode == "sorted":
        for n, (_, _, accepts) in stations.items():
            if accepts and color in accepts:
                return n
        return None
    return None      # free: LLM 挑


def standoff_for(can_xy, yaw):
    return (can_xy[0] - CAN_AHEAD * math.cos(yaw),
            can_xy[1] - CAN_AHEAD * math.sin(yaw))


def clearance(origin, stand, dest, others):
    """去程(origin->standoff)和回程(standoff->dest)两条腿, 离其余各罐子的最近距离。"""
    worst, blocker = 99.0, None
    for other_name, other_xy in others:
        for leg in ((origin, stand), (stand, dest)):
            d = seg_pt_dist(leg[0], leg[1], other_xy)
            if d < worst:
                worst, blocker = d, other_name
    return worst, blocker


def approach_ref(state, stations):
    """接近方向的参考点。默认 None = 从车当前位置直着开过去(不用绕)。

    ⚠️⚠️ 08-01 两次修正, 以这版为准:
    ① 最早用"送达站"当参考 —— 那把接近方向和选站绑死了, 自由选站模式下站一变方向就变。
    ② 然后改成"标签朝着的固定参考点" —— **前提是错的**: 用户指出罐子是超市可乐/雪碧罐,
       **罐身通体红/绿**(除了顶和底), 根本没有"标签"这一面。真正影响检测的是
       **配料表/条形码那一片破坏了纯色** => 色块外接框中心偏离罐子轴心几厘米 => 爪子蹭罐壁推倒它。
       那是**每个罐子自己的一个小扇区**, 不是全场统一的朝向。
    ⇒ 所以接近方向**基本自由, 该纯按几何选**。默认从车当前位置直着进去(转得最少)。
       真要固定方向(比如场地一侧有墙进不去)才在 state 里给 approach_from。
    """
    ref = state.get("approach_from")
    if isinstance(ref, (list, tuple)) and len(ref) == 2:
        return tuple(ref)
    if ref in stations:
        return stations[ref][:2]
    return None      # 默认: 从车当前位置进


def pick_yaw(can_xy, origin, dest, others, ref, attempts=0):
    """选接近方向: 默认从车当前位置直着进去, 被挡就绕罐子搜净空最大的。

    🔑 attempts>0 时**首选方向先转 60 度**: 上一次空夹很可能是配料表那面正对相机,
       原地重试同一个角度结果多半一样(07-25/07-29 都吃过)。换个方向进去才有意义。
    """
    base = ref if ref else origin
    pref = math.atan2(can_xy[1] - base[1], can_xy[0] - base[0])
    if attempts:
        pref += math.radians(60) * attempts
    stand = standoff_for(can_xy, pref)
    worst, blocker = clearance(origin, stand, dest, others)
    if worst >= WARN:
        return pref, stand, worst, blocker, 0.0

    best = (worst, pref, stand, blocker)
    k = YAW_STEP
    while k <= YAW_SEARCH:
        for cand in (pref + k, pref - k):
            s2 = standoff_for(can_xy, cand)
            w2, b2 = clearance(origin, s2, dest, others)
            if w2 > best[0]:
                best = (w2, cand, s2, b2)
        k += YAW_STEP
    worst, yaw, stand, blocker = best
    dev = abs((yaw - pref + math.pi) % (2 * math.pi) - math.pi)
    return yaw, stand, worst, blocker, dev


def robot_pos(state, stations):
    r = state.get("robot") or {}
    if "x" in r:
        return (r["x"], r["y"])
    at = r.get("at")
    if at in stations:
        return stations[at][:2]
    return next(iter(stations.values()))[:2]


def plan_candidates(state):
    """对每个未收走的罐子: 定送达站 -> 选接近方向 -> 算 standoff -> 判净空。
    返回 (clear, blocked)。只有 clear 里的才会进 enum。"""
    stations = stations_of(state)
    origin = robot_pos(state, stations)
    ref = approach_ref(state, stations)
    cans = state.get("cans", {}) or {}
    live = {n: c for n, c in cans.items() if c.get("status") in ("pending", "failed")}

    pending = []
    for name, c in sorted(live.items()):
        can_xy = (c["x"], c["y"])
        color = c.get("color", "red")
        others = [(n, (o["x"], o["y"])) for n, o in live.items() if n != name]

        # 送达站: 定死的就用它; free 模式**两个站都要算出来给 LLM 看**。
        # ⚠️ 08-01 修: 原来两个站都试但只把"净空更好"的那个放进候选 ——
        #    而没有其他罐子时两边净空都是哨兵 99.0 = 平手, 严格 > 保留先遍历的那个
        #    ⇒ **站实际上是按字典顺序选的, 距离从没参与过决策**。
        #    实测代价: can5 离 B 只有 1.58m 却送去了 3.95m 外的 A。
        fixed = station_for(name, color, state, stations)
        trials = [fixed] if fixed else list(stations)
        opts = {}
        for sname in trials:
            dest = stations[sname][:2]
            yaw, stand, worst, blocker, dev = pick_yaw(
                can_xy, origin, dest, others, ref, c.get("attempts", 0))
            trip = (math.hypot(origin[0] - stand[0], origin[1] - stand[1])
                    + math.hypot(stand[0] - dest[0], stand[1] - dest[1]))
            opts[sname] = dict(yaw=yaw, stand=stand, clearance=worst,
                               blocker=blocker, dev=dev, trip=trip)

        # 过几何门的站; 一个都不过这个罐子才算 blocked
        ok = {k: v for k, v in opts.items() if v["clearance"] >= WARN}
        pool = ok or opts
        # 默认站 = 过了门里**路程最短**的(平手绝不能让字典顺序决定)
        sname = min(pool, key=lambda k: (round(pool[k]["trip"], 2), k))
        v = pool[sname]
        yaw, worst, stand, blocker, dev, trip = (
            v["yaw"], v["clearance"], v["stand"], v["blocker"], v["dev"], v["trip"])

        entry = {
            "name": name, "color": color,
            "can": [round(can_xy[0], 3), round(can_xy[1], 3)],
            "station": sname,
            "standoff": [round(stand[0], 3), round(stand[1], 3), round(math.degrees(yaw), 1)],
            "clearance": round(worst, 2),
            "trip": round(trip, 2),           # 这一趟(去+回站)要走多远, 从车当前位置算
            "attempts": c.get("attempts", 0),
            "note": c.get("note"),
        }
        if len(opts) > 1:
            entry["各站的账"] = {
                k: {"trip": round(v["trip"], 2), "净空": round(v["clearance"], 2),
                    "可走": v["clearance"] >= WARN}
                for k, v in opts.items()}
        if dev > YAW_PREF_TOL:
            entry["detour_note"] = ("为了避开别的罐子, 接近方向绕了 %.0f 度(车要多转一点)"
                                    % math.degrees(dev))
        if c.get("attempts"):
            entry["retry_angle"] = ("这是第 %d 次重试, 接近方向已经换到 %.0f 度(上次那个角度没抓上, "
                                    "多半是配料表那面正对相机)" % (c["attempts"] + 1, math.degrees(yaw)))
        entry["_opts"] = opts
        entry["_origin"] = origin
        entry["_stations"] = {k: stations[k][:2] for k in opts}
        entry["_worst"] = worst
        entry["_blocker"] = blocker
        pending.append(entry)

    return _finalize(pending, live)


def _finalize(pending, live):
    """直线模型已经给每个罐子选好了角度和默认站; 若挂了 REAL_CLEARANCE 就用
    move_base 的真实全局路径复核一遍(直线只是粗糙近似, 08-01 实测会漏)。"""
    real = None
    if REAL_CLEARANCE and pending:
        legs = []
        for e in pending:
            for sname, v in e["_opts"].items():
                sx, sy = e["_origin"]
                stx, sty = v["stand"]
                dx, dy = e["_stations"][sname]
                yr = math.radians(round(math.degrees(v["yaw"]), 1))
                # ⚠️ exclude 必须带 —— 目标罐自己离 standoff 恒为 0.45m,
                #    不排除的话每条腿的净空都是 0.45, 配 WARN=0.50 会把所有罐子判死。
                # target(不是 exclude): 目标罐照样量, 用 SELF_MIN 单独判 —— 见 plan_clearance.py 注释
                legs.append({"id": "%s|%s|out" % (e["name"], sname), "target": e["name"],
                             "start": [sx, sy, 0.0], "goal": [stx, sty, yr]})
                legs.append({"id": "%s|%s|back" % (e["name"], sname), "target": e["name"],
                             "start": [stx, sty, yr], "goal": [dx, dy, 0.0]})
        cans = {n: [c["x"], c["y"]] for n, c in live.items()}
        try:
            real = REAL_CLEARANCE(legs, cans)
        except Exception as ex:                       # 查不到就退回直线, 别把整条链搞崩
            print("!! 真实路径净空查不到(%s), 退回直线模型" % ex, file=sys.stderr)
            real = None

    clear, blocked = [], []
    for e in pending:
        if real:
            best = None
            for sname, v in e["_opts"].items():
                ds = []
                for suf in ("out", "back"):
                    r = real.get("%s|%s|%s" % (e["name"], sname, suf)) or {}
                    if r.get("min") is not None:
                        ds.append(r["min"])
                    elif r.get("pts") == 0:
                        ds.append(-1.0)               # 规划器给不出路 => 当作不可走
                    # 去程从目标罐身上压过去 => 直接判死(换算成一个必然低于 WARN 的值)
                    tm = r.get("target_min")
                    if suf == "out" and tm is not None and tm < SELF_MIN:
                        ds.append(tm)
                w = min(ds) if ds else v["clearance"]
                # 目标罐自己不算障碍(本来就要开过去), 但**其余罐**用真实路径量
                blk = None
                for suf in ("out", "back"):
                    r = real.get("%s|%s|%s" % (e["name"], sname, suf)) or {}
                    if r.get("min") is not None and abs(r["min"] - w) < 1e-6:
                        blk = r.get("blocker")
                cand = (w, sname, v, blk)
                if best is None or (cand[0] >= WARN) > (best[0] >= WARN) or (
                        (cand[0] >= WARN) == (best[0] >= WARN)
                        and (round(v["trip"], 2), sname) < (round(best[2]["trip"], 2), best[1])):
                    best = cand
            w, sname, v, blk = best
            e["station"] = sname
            e["standoff"] = [round(v["stand"][0], 3), round(v["stand"][1], 3),
                             round(math.degrees(v["yaw"]), 1)]
            e["trip"] = round(v["trip"], 2)
            e["clearance"] = round(w, 2)
            e["净空来源"] = "move_base 真实路径"
            if len(e["_opts"]) > 1:
                e["各站的账"] = {}
                for k, vv in e["_opts"].items():
                    ds = [real.get("%s|%s|%s" % (e["name"], k, suf), {}).get("min")
                          for suf in ("out", "back")]
                    ds = [d for d in ds if d is not None]
                    e["各站的账"][k] = {"trip": round(vv["trip"], 2),
                                        "净空": round(min(ds), 2) if ds else None,
                                        "可走": bool(ds) and min(ds) >= WARN}
            worst, blocker = w, blk
        else:
            worst, blocker = e["_worst"], e["_blocker"]
            e["净空来源"] = "直线近似(未接 move_base)"

        for k in ("_opts", "_origin", "_stations", "_worst", "_blocker"):
            e.pop(k, None)
        if worst >= WARN:
            clear.append(e)
        else:
            e["blocked_by"] = blocker
            blocked.append(e)
    return clear, blocked


# ================= LLM 层 =================

SYSTEM_HEAD = """你是一台移动机械臂小车的任务规划器。场地里有若干罐子, 车要一个一个开过去抓起来, 送到收集站放下, 再去下一个。
无人机在场地上空飞, 会**陆续**把新发现的罐子坐标报进来, 所以你看到的清单随时会变长。

你每次只决定**下一步做什么**, 不做完整计划。

通用判据(按重要性):
1. blocked 里的罐子这一轮**不能选** —— 去它的路会从别的罐子旁边擦过去撞飞(罐子太矮, 车的激光雷达扫不到, 避障系统里根本没有它们)。但注意: **先把挡路的那个收走, 它下一轮就能选了**。
2. 失败过的罐子(attempts>0)要结合 note 判断值不值得马上重试。常见原因是罐子标签没朝着车, 马上原地重试结果多半一样, 不如先收别的、回头再来。
3. trip 是"从车当前位置去收它再送到站"的总路程, 小的优先。
"""

RULES_SINGLE = """
## 这一局只有一个收集站
每收一个罐子都是"从站出发、抓完回同一个站"的往返 ⇒ **总路程与收集顺序无关**, 换顺序零收益。
所以 **"新来的罐子更近"不是改道理由**。

中途改道的判断法: 比较"车离当前目标还有多远"和"改道会白走多远"。
**"还剩"比"白走"小 ⇒ 快到了 ⇒ 继续。**
真正值得改道的只有两类: (a) 当前目标出问题了(新罐子把它的路挡住进了 blocked, 或它自己失败过);
(b) 车基本还没出发。除此之外一律 continue。
"""

RULES_MULTI = """
## 这一局有两个收集站 —— 顺序**是有讲究的**
放完罐子之后车就停在那个站, 下一条腿从那里起步 ⇒ **顺序和选站耦合, 换顺序真的能省路**。
经验: **同色/同区域的罐子连着收**比来回横穿场地省。追"最近的那个"容易被拽着在两站之间来回跑。

所以和单站不同: **新来的罐子如果明显更顺路, 是可以考虑改道的**。
但仍要算账: 改道会把当前腿已经走的那段白付掉(字段里给了)。"还剩"很小就说明快到了, 那就先收完。
"""

RULES_FREE = """
## 选站是你的决定
每个罐子放到哪个站由你定(station 字段)。选站会决定下一条腿从哪起步, 所以**要往后看一步**:
放到离剩下罐子近的那个站, 下一趟才省。
"""

RULES_SORTED = """
## 分类任务: 站由颜色定死
红罐必须送 A 站、绿罐必须送 B 站, 这不是你能选的, 候选里的 station 字段已经定好了。
你能决定的只有**顺序**。注意颜色一交替车就要横穿场地一次。
"""

SCHEMA = {
    "type": "object",
    "properties": {"reason": {"type": "string"}, "next": {"type": "string"}},
    "required": ["reason", "next"],
}


def build_prompt(state, stations):
    mode = state.get("mode", "single")
    s = SYSTEM_HEAD
    s += RULES_SINGLE if len(stations) == 1 else RULES_MULTI
    if len(stations) > 1:
        s += RULES_FREE if mode == "free" else RULES_SORTED
    s += "\n只输出 JSON。reason 用中文, 一句话说清为什么, 要引用具体数字。"
    return s


def ask_llm(state, clear, blocked, stations):
    active = (state.get("active") or {}).get("can")
    allow_continue = bool(active) and (state.get("active") or {}).get("phase") == "nav"
    # ⚠️ allow_continue 时必须把 active 本身从可选里拿掉 —— 否则 continue 和 canN
    #    是同一动作的两个 token, 模型会挑字面更眼熟的, 决策退化成"这是当前任务所以继续"。
    selectable = [c for c in clear if not (allow_continue and c["name"] == active)]
    options = [c["name"] for c in selectable] + ["done"]
    if allow_continue:
        options.insert(0, "continue")

    schema = json.loads(json.dumps(SCHEMA))
    schema["properties"]["next"]["enum"] = options
    mode = state.get("mode", "single")
    # ⚠️ free 模式下 station **不在这一段问** —— 见 pick_station() 的注释。
    # 一个 JSON 里同时问"选哪个罐子"和"送哪个站", 08-01 实测模型会串。

    ctx = {
        "模式": {"single": "单站", "free": "双站自由选站", "sorted": "双站按颜色分类"}[mode],
        "收集站": {n: {"位置": [round(v[0], 2), round(v[1], 2)],
                       "收": v[2] or "全部"} for n, v in stations.items()},
        "车现在在": [round(x, 2) for x in robot_pos(state, stations)],
        "可选的罐子": selectable,
        "暂时不能去的罐子": [{k: v for k, v in b.items()
                              if k in ("name", "clearance", "blocked_by", "trip")}
                             for b in blocked],
        "已收走": [n for n, c in (state.get("cans") or {}).items()
                   if c.get("status") == "collected"],
        "可选动作": options,
    }

    if allow_continue:
        cur = next((c for c in clear if c["name"] == active), None)
        origin = robot_pos(state, stations)
        leg = {"目标": active}
        if cur:
            start = (state.get("active") or {}).get("from")
            left = math.hypot(origin[0] - cur["standoff"][0], origin[1] - cur["standoff"][1])
            leg["车离它还有"] = round(left, 2)
            if start:
                leg["改道会白走"] = round(math.hypot(origin[0] - start[0], origin[1] - start[1]), 2)
            if cur.get("attempts"):
                leg["注意"] = "这个目标失败过 %d 次: %s" % (cur["attempts"], cur.get("note"))
        elif any(b["name"] == active for b in blocked):
            leg["注意"] = "⚠️ 当前目标刚刚被新罐子挡住了, 已经去不了"
        ctx["当前腿"] = leg
        ctx["说明"] = ("车空手在去 %s 的路上, 现在打断是安全的"
                       "(只有'空手去接罐子'这一段能安全改道, 一旦抓上就必须走完)。" % active)

    body = {
        "model": MODEL,
        "messages": [{"role": "system", "content": build_prompt(state, stations)},
                     {"role": "user", "content": json.dumps(ctx, ensure_ascii=False, indent=1)}],
        "format": schema, "stream": False, "think": False,
        "options": {"temperature": 0},
    }
    req = urllib.request.Request(OLLAMA_URL, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        out = json.loads(r.read().decode())
    decision = json.loads(out["message"]["content"])

    # station 兜底: sorted/single 下由确定性层定, 不信 LLM 也不需要它给
    if "station" not in decision:
        pick = next((c for c in selectable if c["name"] == decision.get("next")), None)
        if pick:
            decision["station"] = pick["station"]
    return decision, options, ctx


ABORT_SYSTEM = """你在帮一台正在执行任务的小车做一个**是非判断**: 车正空手开向某个罐子, 半路无人机报来了新目标。
问题只有一个: **要不要放弃当前这个目标、掉头去别的?**

规则(照着套, 别引入别的考虑):
- **"新来的罐子更近"不是理由。**{extra}
- 值得放弃, 只有这几种情况:
    (a) 当前目标**已经去不了了** —— 新罐子挡住了它的路(下面会明说);
    (b) 车基本还没出发(离它还有的距离 ≈ 整条腿), 那放弃本来就不亏;
    (c) 当前目标自己失败过, 而现在有没试过的。
- 其余一律 continue。**"车离它还有"越小越该继续 —— 都快到了。**

只输出 JSON。reason 用中文一句话, 引用具体数字。"""

ABORT_EXTRA_MULTI = ("\n  (这一局有两个收集站, 顺序确实能省路 —— 但那是**下一轮**选下一个罐子时的事, "
                     "不构成半路放弃当前目标的理由。)")

ABORT_SCHEMA = {
    "type": "object",
    "properties": {"reason": {"type": "string"},
                   "abort": {"type": "string", "enum": ["continue", "abort"]}},
    "required": ["reason", "abort"],
}


def should_abort(state, clear, blocked, stations):
    """中途打断的第一段: 只问"要不要放弃当前目标", 不问"改去哪"。

    ⚠️ 为什么要拆成两段(08-01 实测踩出来的):
    一次性问"下一步做什么"时, 通用判据里的"trip 小的优先"和改道规则里的"更近不是理由"
    **是冲突的**, qwen3:8b 会挑简单那条, 于是拿"新罐子更近"当理由掉头 —— 正是禁止的行为。
    拆开之后这一段的上下文里**根本没有 trip**, 冲突消失。
    """
    active = (state.get("active") or {}).get("can")
    cur = next((c for c in clear if c["name"] == active), None)
    gone = next((b for b in blocked if b["name"] == active), None)
    origin = robot_pos(state, stations)
    start = (state.get("active") or {}).get("from")

    ctx = {"当前目标": active}
    if gone:
        ctx["⚠️ 当前目标刚被新罐子挡住"] = "被 %s 挡, 净空只有 %.2f 米, 车过去会撞飞它" % (
            gone["blocked_by"], gone["clearance"])
    if cur:
        ctx["车离它还有"] = round(math.hypot(origin[0] - cur["standoff"][0],
                                             origin[1] - cur["standoff"][1]), 2)
        if cur.get("attempts"):
            ctx["当前目标失败过"] = "%d 次: %s" % (cur["attempts"], cur.get("note"))
    if start:
        ctx["放弃就白走了"] = round(math.hypot(origin[0] - start[0], origin[1] - start[1]), 2)
    ctx["刚报来的新目标"] = [c["name"] for c in clear if c["name"] != active]

    schema = json.loads(json.dumps(ABORT_SCHEMA))
    extra = ABORT_EXTRA_MULTI if len(stations) > 1 else ""
    body = {
        "model": MODEL,
        "messages": [{"role": "system", "content": ABORT_SYSTEM.format(extra=extra)},
                     {"role": "user", "content": json.dumps(ctx, ensure_ascii=False, indent=1)}],
        "format": schema, "stream": False, "think": False,
        "options": {"temperature": 0},
    }
    req = urllib.request.Request(OLLAMA_URL, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        out = json.loads(r.read().decode())
    d = json.loads(out["message"]["content"])
    log({"stage": "abort?", "ctx": ctx, "decision": d})
    return d


STATION_SYSTEM = """你在帮一台小车做一个**二选一**: 它刚抓起一个罐子, 要送到收集站放下。
问题只有一个: **送哪个站?**

规则:
- **送完之后车就停在那个站, 下一趟从那里出发** —— 所以别只看这一趟, 也看看剩下的罐子在哪边。
- trip = 从车现在的位置去抓它、再送到这个站的总路程。**没有别的考虑就选 trip 小的。**
- 只有当"另一个站明显更靠近剩下没收的罐子"时, 才值得为此多走一点。
- 标了「不可走」的站**不能选**(路上会撞到别的罐子)。

只输出 JSON。reason 用中文一句话, 必须引用两个站的 trip 数字。"""

STATION_SCHEMA = {
    "type": "object",
    "properties": {"reason": {"type": "string"}, "station": {"type": "string"}},
    "required": ["reason", "station"],
}


def pick_station(state, entry, clear, stations):
    """第二段: 只问"这罐送哪个站"。

    ⚠️ 为什么要单独问(08-01 实测): 原来把"选哪个罐子"和"选哪个站"塞进同一个 JSON,
    模型拿到了两个站的 trip(4.60 vs 5.50)却选了贵的, 还编了句"collect_b 更近"(假的),
    引用的数字也不在候选里 —— 一次问两件事它就串了。
    顶层的 trip 是"默认站的 trip", 它一改站这个数就失效, 更加剧混乱。
    拆开之后这一段的上下文里**只有两个站和两个数**, 没有别的东西可串。
    """
    opts = entry.get("各站的账")
    if not opts or len(opts) < 2:
        return entry["station"], None
    ok = [k for k, v in opts.items() if v.get("可走")]
    if len(ok) < 2:
        return (ok[0] if ok else entry["station"]), None

    rest = [{"罐": c["name"], "位置": c["can"]} for c in clear if c["name"] != entry["name"]]
    ctx = {"要送的罐子": entry["name"], "车现在在": [round(x, 2) for x in robot_pos(state, stations)],
           "两个站": {k: {"trip": v["trip"], "可走": v["可走"],
                         "位置": [round(z, 2) for z in stations[k][:2]]} for k, v in opts.items()},
           "剩下还没收的罐子": rest}
    schema = json.loads(json.dumps(STATION_SCHEMA))
    schema["properties"]["station"]["enum"] = ok
    body = {"model": MODEL,
            "messages": [{"role": "system", "content": STATION_SYSTEM},
                         {"role": "user", "content": json.dumps(ctx, ensure_ascii=False, indent=1)}],
            "format": schema, "stream": False, "think": False,
            "options": {"temperature": 0}}
    req = urllib.request.Request(OLLAMA_URL, data=json.dumps(body).encode(),
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=120) as r:
        d = json.loads(json.loads(r.read().decode())["message"]["content"])
    cheapest = min(ok, key=lambda k: opts[k]["trip"])
    log({"stage": "station?", "can": entry["name"], "opts": opts,
         "decision": d, "确定性层最省的": cheapest,
         "分歧": d["station"] != cheapest})
    return d["station"], d.get("reason")


def log(entry):
    entry["t"] = datetime.datetime.now().isoformat(timespec="seconds")
    with open(LOG, "a") as f:
        f.write(json.dumps(entry, ensure_ascii=False) + "\n")


def decide(state, quiet=False):
    """给 mission_run.py 用的入口。返回 (decision, clear, blocked)。"""
    stations = stations_of(state)
    clear, blocked = plan_candidates(state)
    active = (state.get("active") or {}).get("can")
    allow_continue = bool(active) and (state.get("active") or {}).get("phase") == "nav"

    # 中途打断分两段问: ① 要不要放弃当前目标(是非题, 上下文里没有 trip, 没有冲突判据)
    #                    ② 放弃了才问改去哪(走正常那条路)
    if allow_continue:
        d = should_abort(state, clear, blocked, stations)
        if d["abort"] == "continue":
            dec = {"next": "continue", "reason": d["reason"]}
            log({"stage": "pick", "decision": dec, "skipped": "继续当前目标"})
            return dec, clear, blocked
        state = dict(state, active=None)          # 放弃了, 按空闲重新挑
        clear, blocked = plan_candidates(state)
        allow_continue = False

    if not clear and not allow_continue:
        decision, options = {"reason": "没有可去的罐子了", "next": "done"}, ["done"]
        ctx = {}
    else:
        decision, options, ctx = ask_llm(state, clear, blocked, stations)

    if decision["next"] not in options:      # 兜底: schema 理论上挡得住, 但别信
        decision = {"reason": "LLM 越界输出 %r, 拒绝执行" % decision.get("next"), "next": "done"}
    elif decision["next"] not in ("done", "continue"):
        pick = next((c for c in clear if c["name"] == decision["next"]), None)
        if pick:
            sn, why = pick_station(state, pick, clear, stations)
            decision["station"] = sn
            if why:
                decision["station_reason"] = why
    log({"options": options, "decision": decision, "mode": state.get("mode", "single"),
         "clear": [c["name"] for c in clear], "blocked": [b["name"] for b in blocked],
         "active": active})
    return decision, clear, blocked


# ================= CLI =================

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--state", default=os.path.expanduser("~/mission_state.yaml"))
    ap.add_argument("--no-llm", action="store_true")
    ap.add_argument("--json", action="store_true")
    a = ap.parse_args()

    state = yaml.safe_load(open(a.state)) or {}
    stations = stations_of(state)
    clear, blocked = plan_candidates(state)

    if not a.json:
        print("=== 确定性层: 候选构造 + 几何门 (必撞线 %.2f / 告警线 %.2f, 模式 %s) ==="
              % (HARD, WARN, state.get("mode", "single")))
        for c in clear:
            gap = ("净空 %.2f" % c["clearance"]) if c["clearance"] < 90 else "路上没别的罐子"
            print("  ✅ %-6s %-5s 罐(%+.2f,%+.2f) → %-10s standoff(%+.2f,%+.2f,%+.0f°) "
                  "%s  这趟 %.2fm  试过 %d 次"
                  % (c["name"], c["color"], c["can"][0], c["can"][1], c["station"],
                     c["standoff"][0], c["standoff"][1], c["standoff"][2],
                     gap, c["trip"], c["attempts"]))
            if c.get("note"):
                print("       note: %s" % c["note"])
            if c.get("detour_note"):
                print("       ↪️  %s" % c["detour_note"])
            if c.get("retry_angle"):
                print("       🔄 %s" % c["retry_angle"])
        for b in blocked:
            print("  ⛔ %-6s 净空只有 %.2f, 被 %s 挡住 —— 收走 %s 之后就能去"
                  % (b["name"], b["clearance"], b["blocked_by"], b["blocked_by"]))
        if not clear and not blocked:
            print("  (没有待收的罐子)")

    if a.no_llm:
        return
    try:
        decision, _, _ = decide(state)
    except (urllib.error.URLError, OSError) as e:
        print("!! 叫不动 Ollama: %s" % e, file=sys.stderr)
        sys.exit(3)

    if a.json:
        print(json.dumps(decision, ensure_ascii=False))
        return
    print("\n=== LLM 层 ===")
    print("  → next   : %s" % decision["next"])
    if decision.get("station"):
        print("  → station: %s" % decision["station"])
    print("  → reason : %s" % decision["reason"])


if __name__ == "__main__":
    main()
