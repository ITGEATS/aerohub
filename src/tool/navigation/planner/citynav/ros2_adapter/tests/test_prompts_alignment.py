"""跨包复制源段防漂移：hspm.parse_viewpoint_response_v2 与 upstream 锁定提交
（upstream/src/llm/prompt_builder.py L30-54）做 AST 源段比对。

测试期读上游子模块路径；运行时 planner 包不依赖 upstream 文件（两包分别部署
于 onboard-computer 容器与工作站容器，不互相 import，理由见 hspm.py 模块头）。
"""
import ast
from pathlib import Path

import pytest

from uav_citynav_planner import hspm

UPSTREAM = (Path(__file__).resolve().parents[2]
            / "upstream" / "src" / "llm" / "prompt_builder.py")


def _func_dump(source: str, name: str) -> str:
    """函数 AST 比对，允许复制方在首行添加归属注释 docstring（upstream 原函数
    无 docstring）；除该 docstring 外逐语句严格一致，防止逻辑漂移。"""
    tree = ast.parse(source)
    for node in tree.body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            body = list(node.body)
            if (body and isinstance(body[0], ast.Expr)
                    and isinstance(body[0].value, ast.Constant)
                    and isinstance(body[0].value.value, str)):
                body = body[1:]
            return ast.dump(ast.Module(body=body, type_ignores=[]))
    raise AssertionError(f"function {name} not found")


def test_parse_viewpoint_response_v2_matches_upstream_ast():
    if not UPSTREAM.is_file():
        pytest.skip("upstream 子模块未检出（锁定提交 4b6e926c），跳过源段比对")
    ours = _func_dump(Path(hspm.__file__).read_text(encoding="utf-8"),
                      "parse_viewpoint_response_v2")
    upstream = _func_dump(UPSTREAM.read_text(encoding="utf-8"),
                          "parse_viewpoint_response_v2")
    assert ours == upstream
