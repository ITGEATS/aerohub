"""Tool 侧 prompts 与 upstream 锁定提交的逐字一致性测试（Tool 化改造迁移）。

原居 llm_proxy/tests/test_prompts.py；prompt 现由 Tool 持有（prompts.py），
防漂移职责不变：直接从子模块源码提取比对，不依赖网络与 rclpy。
parse_viewpoint_response_v2 的逐字比对在 test_prompts_alignment.py（hspm 副本）。
"""
import ast
from pathlib import Path

import pytest

from uav_citynav_planner import prompts

UPSTREAM = Path(__file__).resolve().parents[2] / "upstream"
PB = (UPSTREAM / "src" / "llm" / "prompt_builder.py").read_text(encoding="utf-8")
LE = (UPSTREAM / "utils" / "landmark_extraction.py").read_text(encoding="utf-8")
LOCAL = Path(prompts.__file__).read_text(encoding="utf-8")


def _fn_src(text, name):
    tree = ast.parse(text)
    for n in tree.body:
        if isinstance(n, ast.FunctionDef) and n.name == name:
            seg = ast.get_source_segment(text, n)
            assert seg is not None
            return seg
    raise AssertionError(f"{name} not found")


@pytest.mark.parametrize("name", [
    "route_planning_prompt_builder",
    "landmark_caption_prompt_builder",
])
def test_functions_verbatim(name):
    assert _fn_src(LOCAL, name) == _fn_src(PB, name)


def _upstream_landmark_messages():
    """限定在 landmark_extraction_prompting 函数体内找 create 调用
    （文件里 generate_image_description 也有 create，消息形状不同）。"""
    tree = ast.parse(LE)
    fn = [n for n in tree.body if isinstance(n, ast.FunctionDef)
          and n.name == "landmark_extraction_prompting"]
    assert fn, "landmark_extraction_prompting not found"
    for node in ast.walk(fn[0]):
        if (isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
                and node.func.attr == "create"):
            for kw in node.keywords:
                if kw.arg == "messages" and isinstance(kw.value, ast.List):
                    out = {}
                    for d in kw.value.elts:
                        pairs = {k.value: v for k, v in zip(d.keys, d.values)}
                        out[pairs["role"].value] = pairs["content"]
                    return out["system"], out["user"]
    raise AssertionError("upstream landmark extraction messages not found")


def test_landmark_extraction_system_prompt_verbatim():
    sys_node, _ = _upstream_landmark_messages()
    assert prompts.LANDMARK_EXTRACTION_SYSTEM_PROMPT == ast.literal_eval(sys_node)


def test_landmark_extraction_user_prompt_verbatim():
    _, user_node = _upstream_landmark_messages()
    # 上游 f-string 与本地 builder 在同一 instruction 下逐字一致
    rendered = eval(compile(ast.Expression(user_node), "<up>", "eval"),
                    {}, {"instruction": "fly past the orange ball"})
    assert prompts.landmark_extraction_user_prompt(
        "fly past the orange ball") == rendered


def test_route_prompt_contains_inputs_and_examples():
    p = prompts.route_planning_prompt_builder(
        "INSTR", ["L1", "L2"], ["L1"], "L2")
    assert "Navigation instruction: INSTR" in p
    assert "reach the target: L1, L2" in p
    assert "have traversed are: L1" in p
    assert "next navigation subgoal: L2" in p
    # 示例 JSON 花括号（f-string 双写转义后）与尾随空白保留
    assert '"is_found": true,' in p and '"slightly left": subgoal' in p
    assert "you need to plan your next waypoint. \n" in p
    assert "There are two situations: \n" in p


def test_caption_prompt_branches():
    empty = prompts.landmark_caption_prompt_builder([])
    assert "appears in the images" in empty
    filled = prompts.landmark_caption_prompt_builder(["ball", "building"])
    assert "appears in the image from the list below:" in filled
    assert "['ball', 'building']." in filled
