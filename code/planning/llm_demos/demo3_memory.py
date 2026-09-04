import json, urllib.request

URL = "http://localhost:11434/api/chat"
MODEL = "qwen3:8b"
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

def plan_once(user_msg):
    """调一次规划器,返回解析好的 dict(格式由 SCHEMA 护栏保证)。"""
    body = {"model": MODEL, "stream": False, "format": SCHEMA,
            "options": {"temperature": 0},
            "messages": [{"role": "system", "content": SYS},
                         {"role": "user",   "content": user_msg}]}
    req = urllib.request.Request(URL, data=json.dumps(body).encode("utf-8"),
                                 headers={"Content-Type": "application/json"})
    with opener.open(req, timeout=600) as r:
        content = json.loads(r.read())["message"]["content"]   # 这层是模型吐的 JSON 字符串
    return json.loads(content)                                 # 再解析成 dict

SCHEMA = {
    "type": "object",
    "properties": {
        "reason": {"type": "string"},
        # 结构化"记忆":每个瓶子的类型和状态
        "bottles": {"type": "array", "items": {
            "type": "object",
            "properties": {
                "id":     {"type": "string"},
                "type":   {"type": "string", "enum": ["cola", "water"]},
                "status": {"type": "string", "enum": ["unpicked", "carried", "done"]},
            },
            "required": ["id", "type", "status"]}},
        # 参数化命令:动词锁死(enum),目标自由
        "plan": {"type": "array", "items": {
            "type": "object",
            "properties": {
                "action": {"type": "string", "enum": ["navigate", "pick", "put"]},
                "target": {"type": "string"},
            },
            "required": ["action", "target"]}},
    },
    "required": ["reason", "bottles", "plan"],
}

SYS = (
    "你是地面小车(带机械臂)的任务规划器。小车一次只能拿一个瓶子。\n"
    "任务:清理场地所有瓶子并分类——可乐瓶(cola)送到 recycle_cola,矿泉水瓶(water)送到 recycle_water。\n"
    "你会收到:无人机探测报告、上一轮世界记忆、刚发生的事件。\n"
    "请输出更新后的 bottles(世界记忆)和 plan(接下来的子任务序列)。\n"
    "已经 done 的瓶子绝不能再安排任何任务。plan 的 action 只能是 navigate/pick/put。"
)

def show(step, world):
    print(f"\n{'='*58}\n{step}\n{'='*58}")
    print("世界记忆 bottles:")
    for b in world["bottles"]:
        print(f"   {b['id']:>9}  type={b['type']:<5}  status={b['status']}")
    print(f"plan(共 {len(world['plan'])} 步):")
    for p in world["plan"]:
        print(f"   {p['action']} -> {p['target']}")

# ---------- 第0步:无人机首次探测,初始化记忆 ----------
step0_msg = (
    "无人机探测报告:\n"
    "- bottle_1: 可乐瓶\n- bottle_2: 矿泉水瓶\n- bottle_3: 可乐瓶\n"
    "- bottle_4: 矿泉水瓶\n- bottle_5: 可乐瓶\n"
    "上一轮世界记忆: 无(这是第一次)\n"
    "刚发生的事件: 无\n"
    "请初始化世界记忆并规划。"
)
world = plan_once(step0_msg)
show("第0步:无人机探测到5个瓶子 -> 初始化记忆 + 规划", world)

# ---------- 第1步:小车执行了一次,代码把"真实结果"喂回去 ----------
# 关键:是【代码】把上一轮 world 序列化后塞回 prompt,模型自己不记得
step1_msg = (
    "无人机探测报告: 同上,无新增。\n"
    f"上一轮世界记忆: {json.dumps(world['bottles'], ensure_ascii=False)}\n"
    "刚发生的事件: 小车已成功抓取 bottle_1 并放入 recycle_cola(执行成功)。\n"
    "请更新世界记忆并重新规划。"
)
world = plan_once(step1_msg)
show("第1步:bottle_1 已送达 -> 记忆应更新为 done、plan 应去掉它", world)
