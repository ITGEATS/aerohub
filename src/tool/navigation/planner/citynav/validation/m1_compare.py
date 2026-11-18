#!/usr/bin/env python3
"""M-1 对照评分：代理 vs legacy SDK 同帧响应一致率。

解析双侧文本（parse_viewpoint_response_v2 = 上游原件，经 upstream 路径 import；
代理与 planner 的副本一致性由 test_prompts_alignment AST 测试保证）。
判据（迁移方案阶段 3 M-1 锁定口径"视角选择/object 词表一致"）：
  strict  = is_found 相同 且 逐视角 (view→object) 映射规范化后完全一致；
  relaxed = is_found 相同 且 object 词表（忽略视角归属）一致——仅记录，
  门槛按 strict ≥ 90%。
规范化：小写、去首尾空白、collapse 连续空白。
"""
import argparse
import json
from pathlib import Path
import sys

REPO = Path(__file__).resolve().parents[6]
UPSTREAM = REPO / "src" / "tool" / "navigation" / "planner" / "citynav" / "upstream"
sys.path.insert(0, str(UPSTREAM))
from src.llm.prompt_builder import parse_viewpoint_response_v2  # noqa: E402

VIEWS = ("left", "slightly_left", "front", "slightly_right", "right")


def norm(s):
    return " ".join(str(s).lower().split())


def structure(parsed):
    is_found = bool(parsed.get("is_found", False))
    views = {v: norm(parsed[v]) for v in VIEWS if v in parsed}
    return is_found, views


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--proxy", type=Path, required=True)
    ap.add_argument("--sdk", type=Path, required=True)
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--threshold", type=float, default=0.90)
    args = ap.parse_args()

    proxy = {r["set"]: r for r in json.loads(args.proxy.read_text())}
    sdk = {r["set"]: r for r in json.loads(args.sdk.read_text())}
    sets = sorted(set(proxy) & set(sdk))

    rows = []
    strict = relaxed = usable = 0

    def unpack(rec):
        """兼容两种记录形态：代理侧 {"result": {ok,text,error_code}} 与
        SDK 侧 {"ok","text","error"}（也用于 SDK-vs-SDK 自对照）。"""
        if "result" in rec:
            r = rec.get("result") or {}
            return bool(r.get("ok")), r.get("text"), r.get("error_code")
        return bool(rec.get("ok")), rec.get("text"), rec.get("error")

    for sid in sets:
        p, s = proxy[sid], sdk[sid]
        p_ok, ptext, p_err = unpack(p)
        s_ok, stext, s_err = unpack(s)
        row = {"set": sid}
        if not p_ok or not ptext or not s_ok or not stext:
            row.update(status="NOT-COMPARABLE", proxy_error=p_err, sdk_error=s_err)
            rows.append(row)
            continue
        usable += 1
        try:
            ps = structure(parse_viewpoint_response_v2(ptext))
            ss = structure(parse_viewpoint_response_v2(stext))
        except ValueError as exc:
            row.update(status="PARSE-FAIL", error=str(exc))
            rows.append(row)
            continue
        is_strict = ps[0] == ss[0] and ps[1] == ss[1]
        is_relaxed = ps[0] == ss[0] and sorted(ps[1].values()) == sorted(ss[1].values())
        strict += int(is_strict)
        relaxed += int(is_relaxed)
        row.update(status="MATCH" if is_strict else "DIFF",
                   strict=is_strict, relaxed=is_relaxed,
                   proxy={"is_found": ps[0], "views": ps[1]},
                   sdk={"is_found": ss[0], "views": ss[1]})
        rows.append(row)

    total = len(sets)
    report = {
        "criterion": "M-1 strict structured agreement (is_found + per-view object) "
                     ">= 90% over >= 20 same-frame sets",
        "sets_compared": total,
        "sets_usable": usable,
        "strict_match": strict,
        "strict_rate": round(strict / total, 4) if total else None,
        "relaxed_match": relaxed,
        "threshold": args.threshold,
        "verdict": ("PASS" if total >= 20 and usable == total and strict / total >= args.threshold
                    else "FAIL" if total else "NOT-EXECUTED"),
        "rows": rows,
    }
    if total and usable != total:
        report["verdict"] = "FAIL"  # 缺可比组不视为通过（AGENTS 证据完整性）
    args.out.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n")
    print(json.dumps({k: v for k, v in report.items() if k != "rows"}, indent=2))
    raise SystemExit(0 if report["verdict"] == "PASS" else 1)


if __name__ == "__main__":
    main()
