#!/usr/bin/env python3
"""M-1 legacy SDK 侧驱动器（宿主 legacy venv 运行：openai 1.58.1 + upstream）。

调用路径 = legacy_stub/run_legacy.py PatchedLLM.query_viewpoint_api 的等价
复刻（该类内联于脚本不可 import，逻辑逐字对应）：
  - prompt：upstream/src/llm/prompt_builder.route_planning_prompt_builder
    （直接 import 上游函数，非代理包副本——两侧同源由
    tests/test_prompts_alignment.py AST 比对保证，这里用上游原件做终审）；
  - 消息：逐视角 "{viewpoint} image: " + data:image/jpeg;base64（上游原始
    JPEG 字节），末尾 prompt 文本块，单条 user 消息；
  - 响应 NFKC 归一；有限重试 3 次线性退避（D10）。
模型/base_url/key 与 legacy 运行同源（.env）。
"""
import argparse
import base64
import json
import os
from pathlib import Path
import sys
import time
import unicodedata

REPO = Path(__file__).resolve().parents[6]
UPSTREAM = REPO / "src" / "tool" / "navigation" / "planner" / "citynav" / "upstream"
ENV_PATH = REPO / "src" / "tool" / "navigation" / "planner" / "citynav" / "legacy_stub" / ".env"

VIEW_ORDER = ["left", "slightly_left", "front", "slightly_right", "right"]


def load_env(path):
    for line in path.read_text(encoding="utf-8").splitlines():
        line = line.strip()
        if line and not line.startswith("#") and "=" in line:
            k, v = line.split("=", 1)
            os.environ.setdefault(k.strip(), v.strip())


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--manifest", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--context", type=Path, default=None,
                    help="JSON {instruction,landmarks,traversed,next}")
    ap.add_argument("--instruction", default=None)
    ap.add_argument("--landmarks", default=None, help="JSON 数组")
    ap.add_argument("--traversed", default="[]", help="JSON 数组")
    ap.add_argument("--next", dest="next_lm", default=None)
    ap.add_argument("--only", type=int, nargs="*", default=None)
    args = ap.parse_args()
    if args.context:
        ctx = json.loads(args.context.read_text(encoding="utf-8"))
        args.instruction = ctx["instruction"]
        args.landmarks = json.dumps(ctx["landmarks"])
        args.traversed = json.dumps(ctx.get("traversed", []))
        args.next_lm = ctx["next"]
    if not args.instruction or not args.landmarks or not args.next_lm:
        raise SystemExit("need --context or --instruction/--landmarks/--next")

    load_env(ENV_PATH)
    model = os.environ.get("CITYNAV_LLM_MODEL", "glm-4v-flash")
    base = os.environ.get("OPENAI_BASE_URL", "https://open.bigmodel.cn/api/paas/v4")
    key = os.environ.get("OPENAI_API_KEY", "")
    if not key:
        raise SystemExit("OPENAI_API_KEY missing (.env)")

    sys.path.insert(0, str(UPSTREAM))
    from src.llm.prompt_builder import route_planning_prompt_builder
    from openai import OpenAI
    client = OpenAI(api_key=key, base_url=base, max_retries=0)

    manifest = json.loads(args.manifest.read_text())
    frames_dir = args.manifest.parent
    landmarks = json.loads(args.landmarks)
    traversed = json.loads(args.traversed)

    def query_viewpoint_api(prompt, image_paths):
        content_block = []
        for viewpoint in VIEW_ORDER:
            content_block.append({"type": "text", "text": f"{viewpoint} image: "})
            with open(image_paths[viewpoint], "rb") as f:
                b64 = base64.b64encode(f.read()).decode("ascii")
            content_block.append({"type": "image_url",
                                  "image_url": {"url": f"data:image/jpeg;base64,{b64}"}})
        content_block.append({"type": "text", "text": f"{prompt}"})
        completion = client.chat.completions.create(
            messages=[{"role": "user", "content": content_block}], model=model)
        return unicodedata.normalize("NFKC", completion.choices[0].message.content)

    results = []
    if args.out.exists():
        results = json.loads(args.out.read_text())
    done = {r["set"] for r in results}
    for entry in manifest["sets"]:
        if args.only is not None and entry["id"] not in args.only:
            continue
        if entry["id"] in done:
            continue
        image_paths = {label: str(frames_dir / entry["files"][label])
                       for label in VIEW_ORDER}
        prompt = route_planning_prompt_builder(args.instruction, landmarks,
                                               traversed, args.next_lm)
        last = None
        text = None
        for attempt in range(3):
            try:
                text = query_viewpoint_api(prompt, image_paths)
                break
            except Exception as exc:  # D10 有限重试
                last = exc
                print(f"set {entry['id']:02d} attempt {attempt + 1}/3 failed: {exc}",
                      flush=True)
                time.sleep(2 * (attempt + 1))
        rec = {"set": entry["id"], "pose": entry["pose"],
               "ok": text is not None, "text": text,
               "error": repr(last) if text is None else None}
        results.append(rec)
        print(f"set {entry['id']:02d}: {'ok' if text else last}", flush=True)
        args.out.write_text(json.dumps(results, indent=2, ensure_ascii=False))
    print(f"sdk side done: {sum(1 for r in results if r['ok'])}/{len(results)} ok")


if __name__ == "__main__":
    main()
