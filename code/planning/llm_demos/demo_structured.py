import json, urllib.request

URL = "http://localhost:11434/api/chat"
MODEL = "qwen3:8b"

# 避免环境里的 http 代理把 localhost 也拦了
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

def call(messages, fmt=None):
    body = {"model": MODEL, "messages": messages, "stream": False}
    if fmt is not None:
        body["format"] = fmt           # 方法3 的关键:传一个 JSON schema 当"护栏"
    req = urllib.request.Request(
        URL, data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"})
    with opener.open(req, timeout=300) as r:
        return json.loads(r.read())

# 两次调用用完全一样的任务描述,唯一区别就是加不加 format
SYS  = "你是小车任务规划器。可用命令只有三个:goto A、goto B、stop。"
USER = '把这句话翻译成命令计划,用户说:"去A再去B"。'

sep = "=" * 60

# ---------- 方法1:纯 prompt 求它输出 JSON,不加任何约束 ----------
print(sep); print("方法1:纯 prompt(求它给 JSON,不加护栏)"); print(sep)
m1_user = USER + ' 请只输出一个 JSON,形如 {"reason":"...","plan":["goto A","goto B"]}'
r1 = call([{"role": "system", "content": SYS},
           {"role": "user",   "content": m1_user}])
msg1 = r1["message"]
if msg1.get("thinking"):
    print("[模型的思考,单独字段里]:\n", msg1["thinking"][:300], "...\n")
raw1 = msg1["content"]
print("--- 模型吐出来的 content 原文 ---")
print(repr(raw1))            # 用 repr 你能看清有没有 ```、换行、多余的话
print("\n--- 直接 json.loads(content) 会怎样 ---")
try:
    obj = json.loads(raw1)
    print("  解析成功:", obj)
except Exception as e:
    print(f"  ❌ 崩了 -> {type(e).__name__}: {e}")

# ---------- 方法3:传 format(JSON schema)当护栏 ----------
print("\n" + sep); print("方法3:structured outputs(传 format 护栏)"); print(sep)
schema = {
    "type": "object",
    "properties": {
        "reason": {"type": "string"},
        # 把 plan 里每一项也锁死成只能是这三个命令之一(enum)
        "plan": {"type": "array",
                 "items": {"type": "string",
                           "enum": ["goto A", "goto B", "stop"]}},
    },
    "required": ["reason", "plan"],
}
r3 = call([{"role": "system", "content": SYS},
           {"role": "user",   "content": USER}], fmt=schema)
raw3 = r3["message"]["content"]
print("--- 模型吐出来的 content 原文 ---")
print(repr(raw3))
print("\n--- 直接 json.loads(content) 会怎样 ---")
obj3 = json.loads(raw3)
print("  解析成功:", obj3)
print("  代码可直接用 -> plan =", obj3["plan"])
