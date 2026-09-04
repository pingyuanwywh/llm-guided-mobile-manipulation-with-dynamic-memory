#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
5 瓶子复杂任务主线:模拟无人机探测清单 -> LLM 消化+筛选+排序规划 -> SSH 驱动实车逐点导航 -> 落盘 bottles.yaml。

与 car_run.py 的区别:
  - 地点不再是固定 A/B,而是"无人机探测"到的一批瓶子(带类型+坐标),从 detections JSON 动态读。
  - LLM 拿到整份瓶子清单,按任务语义(如"收集所有可乐")筛选+排序,输出 [goto b1, goto b3, ...] 计划。
  - plan 的 enum 由探测到的瓶子 id 动态生成,格式仍 100% 合法。

本机运行:需能访问 localhost:11434 的 Ollama,并能免密 ssh 到车。
探测清单默认 ~/bottles_detected.json(录完点后由 make_detections 生成);瓶子 id 必须与车上 llm_nav_places.yaml 的航点名一致。

用法:
    python3 bottle_run.py "收集所有可乐"           # 干跑:只翻译+打印计划,不动车
    python3 bottle_run.py --go "收集所有可乐"       # 真执行:逐条 goto 发给实车
    python3 bottle_run.py --go "把所有瓶子都巡一遍" # 全巡
"""
import os
import argparse, json, os, subprocess, sys, time, urllib.request

# ---------------- 配置 ----------------
OLLAMA_URL   = "http://localhost:11434/api/chat"
MODEL        = "qwen3:8b"
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

CAR          = "%s@%s" % (_env("CAR_USER", "uavg"), _env("CAR_IP"))
DETECTIONS   = os.path.expanduser("~/bottles_detected.json")   # 模拟无人机探测结果
CAR_SOURCE   = "source /opt/ros/noetic/setup.bash && source ~/ros1_ws/devel/setup.bash"

_opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def load_detections(path):
    """读模拟无人机探测清单:{"bottles":[{"id","type","x","y"}...]}。"""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    bottles = data.get("bottles", data)
    if not bottles:
        raise SystemExit(f"[中止] 探测清单为空:{path}")
    return bottles


def translate(nl, bottles):
    """翻译器①:人话 + 瓶子清单 -> {reason, plan:[goto <id>...]}。plan 每项被 enum 锁死成合法命令。"""
    ids = [b["id"] for b in bottles]
    allowed = [f"goto {i}" for i in ids] + ["stop"]
    schema = {
        "type": "object",
        "properties": {
            "reason": {"type": "string"},
            "plan": {"type": "array",
                     "items": {"type": "string", "enum": allowed}},
        },
        "required": ["reason", "plan"],
    }
    # 把探测清单摊给 LLM 看:id、类型、坐标
    inv = "\n".join(f"  - {b['id']}: 类型={b['type']}, 位置=({b['x']:.2f}, {b['y']:.2f})"
                    for b in bottles)
    sys_prompt = (
        "你是一辆室内移动机械臂小车的任务规划器。无人机刚探测到以下瓶子(清单固定,不要臆造不存在的瓶子):\n"
        + inv + "\n\n"
        "把用户的任务翻译成一串导航命令,每条只能是 " + " / ".join(allowed) + " 之一。\n"
        "规则:\n"
        "1. 先在 reason 里想清楚要去哪些瓶子、为什么(可按类型筛选)。\n"
        "2. 只对'需要去'的瓶子生成 goto,不需要的不要放进 plan。\n"
        "3. 尽量按就近顺序排列,减少来回跑(位置是 map 坐标,单位米)。\n"
        "4. 用户要求停下、或任务结束时可用 stop 收尾。\n"
        "只输出 plan 里真正要执行的命令,按先后排列。"
    )
    body = {"model": MODEL, "stream": False, "format": schema,
            "options": {"temperature": 0},
            "messages": [{"role": "system", "content": sys_prompt},
                         {"role": "user", "content": nl}]}
    req = urllib.request.Request(OLLAMA_URL, data=json.dumps(body).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    with _opener.open(req, timeout=180) as r:
        content = json.loads(r.read())["message"]["content"]
    return json.loads(content)


def run_on_car(cmd_str):
    """把一条 plan 命令通过 SSH 发到车上执行,返回车回的 JSON dict。"""
    if cmd_str == "stop":
        remote = f"{CAR_SOURCE} && python3 ~/llm_nav_commander.py stop"
    elif cmd_str.startswith("goto "):
        place = cmd_str.split(None, 1)[1].strip()
        remote = f"{CAR_SOURCE} && python3 ~/llm_nav_commander.py goto --place {place} --wait"
    else:
        return {"ok": False, "error": "unknown_command", "cmd": cmd_str}

    proc = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", CAR, remote],
        capture_output=True, text=True)
    json_lines = [l for l in proc.stdout.splitlines() if l.strip().startswith("{")]
    if json_lines:
        try:
            return json.loads(json_lines[-1])
        except json.JSONDecodeError:
            pass
    return {"ok": False, "error": "no_json_from_car", "returncode": proc.returncode,
            "stdout": proc.stdout[-500:], "stderr": proc.stderr[-500:]}


def save_bottles_memory(bottles, visited, task):
    """把这次任务消化后的瓶子清单 + 实际去过的点落盘到车上 ~/bottles.yaml(记忆)。"""
    lines = ["# 由 bottle_run.py 落盘:无人机探测到的瓶子清单 + 本次任务去过的点",
             f"task: {json.dumps(task, ensure_ascii=False)}",
             "bottles:"]
    for b in bottles:
        was = "true" if b["id"] in visited else "false"
        lines.append(f"  - id: {b['id']}")
        lines.append(f"    type: {json.dumps(b['type'], ensure_ascii=False)}")
        lines.append(f"    x: {b['x']:.4f}")
        lines.append(f"    y: {b['y']:.4f}")
        lines.append(f"    visited: {was}")
    payload = "\n".join(lines) + "\n"
    proc = subprocess.run(
        ["ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=8", CAR,
         "cat > ~/bottles.yaml"],
        input=payload, capture_output=True, text=True)
    return proc.returncode == 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instruction", help="给车的自然语言任务,如 '收集所有可乐'")
    ap.add_argument("--go", action="store_true", help="真执行(驱动实车);不加则只干跑打印计划")
    ap.add_argument("--detections", default=DETECTIONS, help="模拟无人机探测清单 JSON 路径")
    args = ap.parse_args()

    bottles = load_detections(args.detections)
    print(f"[探测] 无人机清单({len(bottles)} 瓶):")
    for b in bottles:
        print(f"        {b['id']}: {b['type']} @ ({b['x']:.2f},{b['y']:.2f})")

    print(f"\n[翻译] 任务: {args.instruction}")
    result = translate(args.instruction, bottles)
    plan = result.get("plan", [])
    print(f"[翻译] 理由: {result.get('reason', '')}")
    print(f"[翻译] 计划: {plan}")

    if not plan:
        print("[中止] 计划为空,不执行。")
        return 1
    if not args.go:
        print("\n[干跑] 未加 --go,不驱动实车。确认计划无误后加 --go 再跑一次即可真执行。")
        return 0

    print("\n⚠️  即将驱动实车!2 秒内 Ctrl-C 可中止 ...")
    try:
        time.sleep(2)
    except KeyboardInterrupt:
        print("\n[中止] 用户取消。")
        return 1

    visited = []
    rc = 0
    try:
        for i, cmd in enumerate(plan, 1):
            print(f"\n[{i}/{len(plan)}] 执行: {cmd}")
            res = run_on_car(cmd)
            print(f"        车返回: {res}")
            if not res.get("ok", False):
                print("[中止] 该命令失败,发送 stop 并停止后续。")
                run_on_car("stop"); rc = 2; break
            state = res.get("state_text")
            if state is not None and state != "SUCCEEDED":
                print(f"[中止] 导航结果 {state}(非 SUCCEEDED),发送 stop 并停止后续。")
                run_on_car("stop"); rc = 2; break
            if cmd.startswith("goto ") and state == "SUCCEEDED":
                visited.append(cmd.split(None, 1)[1].strip())
        else:
            print("\n✅ 全部命令执行完成。")
    except KeyboardInterrupt:
        print("\n[中止] 收到 Ctrl-C,发送 stop 给车 ...")
        run_on_car("stop"); rc = 1

    # 落盘记忆:不管成败,把消化后的清单 + 去过的点写到车上 bottles.yaml
    ok = save_bottles_memory(bottles, visited, args.instruction)
    print(f"\n[记忆] bottles.yaml 落盘{'成功' if ok else '失败'};本次去过: {visited or '无'}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
