"""prompt/clean_response_text 与上游锁定提交的逐字一致性测试（阶段 3 验收 M-1 前置）。

防止复制模板随时间漂移：直接从子模块源码提取比对，不依赖网络。
"""
import ast
import re
from pathlib import Path

from uav_spf_vlm_proxy.prompt import PROMPT_TEMPLATE, clean_response_text

UPSTREAM = Path(__file__).resolve().parents[2] / "upstream"
ACTION_PROJECTOR = UPSTREAM / "src" / "spf" / "airsim" / "action_projector.py"
VLM_CLIENT = UPSTREAM / "src" / "spf" / "clients" / "vlm_client.py"


def _upstream_prompt_template() -> str:
    src = ACTION_PROJECTOR.read_text(encoding="utf-8")
    # _get_single_action 内的第一个 f-string（上游唯一使用点）
    m = re.search(r'prompt = f"""(.*?)"""', src, re.S)
    assert m, "upstream prompt f-string not found"
    return m.group(1)


def _dump_body(fn_node) -> str:
    body = list(fn_node.body)
    # 忽略 docstring 差异，比对可执行逻辑
    if body and isinstance(body[0], ast.Expr) and isinstance(body[0].value, ast.Constant) \
            and isinstance(body[0].value.value, str):
        body = body[1:]
    return ast.dump(ast.Module(body=body, type_ignores=[]))


def _upstream_clean_body() -> str:
    tree = ast.parse(VLM_CLIENT.read_text(encoding="utf-8"))
    for cls in [n for n in tree.body if isinstance(n, ast.ClassDef)]:
        for fn in [n for n in cls.body if isinstance(n, ast.FunctionDef)
                   and n.name == "clean_response_text"]:
            return _dump_body(fn)
    raise AssertionError("upstream clean_response_text not found")


def _local_clean_body() -> str:
    import uav_spf_vlm_proxy.prompt as mod
    tree = ast.parse(open(mod.__file__, encoding="utf-8").read())
    for f in [n for n in tree.body if isinstance(n, ast.FunctionDef)
              and n.name == "clean_response_text"]:
        return _dump_body(f)
    raise AssertionError("local clean_response_text not found")


def test_prompt_template_verbatim():
    assert PROMPT_TEMPLATE == _upstream_prompt_template()


def test_clean_response_text_ast_equal():
    assert _local_clean_body() == _upstream_clean_body()


def test_clean_response_text_behavior():
    assert clean_response_text('```json\n[{"point":[1,2]}]\n```') == '[{"point":[1,2]}]'
    assert clean_response_text('```\nabc\n```') == "abc"
    assert clean_response_text('plain') == "plain"
