import json, urllib.request

URL = "http://localhost:11434/api/chat"
MODEL = "qwen3:8b"
opener = urllib.request.build_opener(urllib.request.ProxyHandler({}))

def call(messages, fmt=None):
    body = {"model": MODEL, "messages": messages, "stream": False}
    if fmt is not None:
        body["format"] = fmt
    req = urllib.request.Request(
        URL, data=json.dumps(body).encode("utf-8"),
        headers={"Content-Type": "application/json"})
    with opener.open(req, timeout=300) as r:
        return json.loads(r.read())

sep = "=" * 60

# 新手随手写的 prompt:自然口气,没死命令"只输出JSON"
SYS  = "你是小车助手。可用命令:goto A、goto B、stop。"
USER = "我想让小车先去A再去B,帮我规划一下路线,顺便说说为什么这么走。"

# ---------- 方法1:同样这句话,不加 format ----------
print(sep); print("方法1:自然 prompt,不加护栏"); print(sep)
r1 = call([{"role": "system", "content": SYS},
           {"role": "user",   "content": USER}])
raw1 = r1["message"]["content"]
print("--- content 原文 ---")
print(raw1)
print("\n--- 直接 json.loads(content) 会怎样 ---")
try:
    obj = json.loads(raw1)
    print("  解析成功:", obj)
except Exception as e:
    print(f"  ❌ 崩了 -> {type(e).__name__}: {e}")

# ---------- 方法3:一模一样的自然 prompt,只多加 format ----------
print("\n" + sep); print("方法3:同一句自然 prompt + format 护栏"); print(sep)
schema = {
    "type": "object",
    "properties": {
        "reason": {"type": "string"},
        "plan": {"type": "array",
                 "items": {"type": "string",
                           "enum": ["goto A", "goto B", "stop"]}},
    },
    "required": ["reason", "plan"],
}
r3 = call([{"role": "system", "content": SYS},
           {"role": "user",   "content": USER}], fmt=schema)
raw3 = r3["message"]["content"]
print("--- content 原文 ---")
print(raw3)
print("\n--- 直接 json.loads(content) 会怎样 ---")
obj3 = json.loads(raw3)
print("  解析成功,代码直接能用 -> plan =", obj3["plan"])
