#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
主线 MVP:自然语言 -> LLM 规划(翻译器①) -> SSH 驱动车上的 llm_nav_commander(翻译器②)。
本机运行:需能访问 localhost:11434 的 Ollama,并能免密 ssh 到车。

用法:
    python3 car_run.py "去A再去B"          # 干跑:只翻译+打印计划,不动车
    python3 car_run.py --go "去A再去B"      # 真执行:逐条把 goto 发给实车
"""
import os
import argparse, json, subprocess, sys, time, urllib.request

# ---------------- 配置 ----------------
OLLAMA_URL = "http://localhost:11434/api/chat"
MODEL      = "qwen3:8b"
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

CAR        = "%s@%s" % (_env("CAR_USER", "uavg"), _env("CAR_IP"))
ALLOWED_PLACES = ["A", "B"]          # 必须与车上 ~/llm_nav_places.yaml 一致
CAR_SOURCE = "source /opt/ros/noetic/setup.bash && source ~/ros1_ws/devel/setup.bash"

_opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))


def translate(nl):
    """翻译器①:人话 -> {reason, plan:[...]}。plan 里每项被 enum 锁死成合法命令。"""
    allowed = [f"goto {p}" for p in ALLOWED_PLACES] + ["stop"]
    schema = {
        "type": "object",
        "properties": {
            "reason": {"type": "string"},
            "plan": {"type": "array",
                     "items": {"type": "string", "enum": allowed}},
        },
        "required": ["reason", "plan"],
    }
    sys_prompt = (
        "你是一辆室内小车的任务规划器。小车能去的地点只有:" + ", ".join(ALLOWED_PLACES) + "。\n"
        "把用户的话翻译成一串命令,每条只能是 " + " / ".join(allowed) + " 之一,按意图先后排列。\n"
        "用户要求停下时用 stop。"
    )
    body = {"model": MODEL, "stream": False, "format": schema,
            "options": {"temperature": 0},
            "messages": [{"role": "system", "content": sys_prompt},
                         {"role": "user", "content": nl}]}
    req = urllib.request.Request(OLLAMA_URL, data=json.dumps(body).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    with _opener.open(req, timeout=120) as r:
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
    # 车上脚本最后打印一行 JSON;前面可能有 ROS 日志,取最后一个 '{' 开头的行
    json_lines = [l for l in proc.stdout.splitlines() if l.strip().startswith("{")]
    if json_lines:
        try:
            return json.loads(json_lines[-1])
        except json.JSONDecodeError:
            pass
    return {"ok": False, "error": "no_json_from_car", "returncode": proc.returncode,
            "stdout": proc.stdout[-500:], "stderr": proc.stderr[-500:]}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("instruction", help="给车的自然语言指令,如 '去A再去B'")
    ap.add_argument("--go", action="store_true", help="真执行(驱动实车);不加则只干跑打印计划")
    args = ap.parse_args()

    print(f"[翻译] 指令: {args.instruction}")
    result = translate(args.instruction)
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

    try:
        for i, cmd in enumerate(plan, 1):
            print(f"\n[{i}/{len(plan)}] 执行: {cmd}")
            res = run_on_car(cmd)
            print(f"        车返回: {res}")
            if not res.get("ok", False):
                print("[中止] 该命令失败,发送 stop 并停止后续。")
                run_on_car("stop")
                return 2
            state = res.get("state_text")          # goto --wait 完成后带
            if state is not None and state != "SUCCEEDED":
                print(f"[中止] 导航结果 {state}(非 SUCCEEDED),发送 stop 并停止后续。")
                run_on_car("stop")
                return 2
        print("\n✅ 全部命令执行完成。")
        return 0
    except KeyboardInterrupt:
        print("\n[中止] 收到 Ctrl-C,发送 stop 给车 ...")
        run_on_car("stop")
        return 1


if __name__ == "__main__":
    sys.exit(main())
