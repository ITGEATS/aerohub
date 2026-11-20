"""Host tests for prompt assembly + the AST lock on the template literal.

Tokenization equivalence against the openvla-7b tokenizer was already
proven 8/8 EXACT by log/aerovla/stage1/make_prompts.py against the locked
board table; transformers is not required on the host, so the tokenizer is
exercised here through a mock and the path/env contract is checked without
loading anything.
"""

import ast
import pathlib

import pytest

from uav_aerovla_planner import prompts

# Byte-identical to make_prompts.py L23 / aerovla_wrapper_ui.py L146-150.
EXPECTED_TEMPLATE = "<image>\nFly {hint}and find the target. {obj_desc}\nAction: "


def _module_constants(source, names):
    tree = ast.parse(source)
    found = {}
    for node in tree.body:
        if isinstance(node, ast.Assign):
            for target in node.targets:
                if isinstance(target, ast.Name) and target.id in names:
                    found[target.id] = node.value
    return found


def test_prompt_template_ast_lock():
    """The template must remain ONE plain string Constant equal to the
    locked literal; f-strings, concatenation or edited bytes all fail here."""
    source = pathlib.Path(prompts.__file__).read_text(encoding="utf-8")
    values = _module_constants(source, {"PROMPT_TEMPLATE", "BOS_TOKEN_ID"})
    assert "PROMPT_TEMPLATE" in values, "PROMPT_TEMPLATE assignment missing"
    node = values["PROMPT_TEMPLATE"]
    assert isinstance(node, ast.Constant) and isinstance(node.value, str), \
        "PROMPT_TEMPLATE must be a single string literal (AST lock)"
    assert node.value == EXPECTED_TEMPLATE
    assert isinstance(values["BOS_TOKEN_ID"], ast.Constant)
    assert values["BOS_TOKEN_ID"].value == 1


def test_build_prompt_text_concatenation():
    desc = "The orange ball is situated on the ground."
    # Non-empty hints carry their OWN trailing space (policy literals); the
    # template has no space before "and".
    text = prompts.build_prompt_text("straight ahead ", desc)
    assert text == ("<image>\nFly straight ahead and find the target. "
                    + desc + "\nAction: ")
    # Empty hint collapses to "Fly and find..." exactly as upstream.
    assert prompts.build_prompt_text("", desc) == (
        "<image>\nFly and find the target. " + desc + "\nAction: ")
    assert text.endswith("Action: ")
    assert text.startswith("<image>\n")


class _FakeTokenizer:
    """Mimics tok(text) -> {"input_ids": [...]} with BOS=1 prepended, the
    openvla-7b behavior verified by make_prompts.py."""

    def __init__(self):
        self.seen = []

    def __call__(self, text):
        self.seen.append(text)
        return {"input_ids": [1] + [2 + i for i in range(len(text))]}


def test_tokenize_preserves_bos_and_text_tokens():
    fake = _FakeTokenizer()
    ids = prompts.tokenize("<image>\nFly and find the target. x\nAction: ",
                           tokenizer=fake)
    assert fake.seen == ["<image>\nFly and find the target. x\nAction: "]
    assert ids[0] == prompts.BOS_TOKEN_ID  # BOS=1 first, no special handling
    assert all(isinstance(v, int) for v in ids)


def test_prompt_token_ids_uses_template():
    fake = _FakeTokenizer()
    ids = prompts.prompt_token_ids("to your left ", "The umbrella table.",
                                   tokenizer=fake)
    assert fake.seen == [prompts.build_prompt_text("to your left ",
                                                   "The umbrella table.")]
    assert ids[0] == 1


def test_tokenizer_path_resolution(monkeypatch):
    monkeypatch.delenv(prompts.TOKENIZER_PATH_ENV, raising=False)
    assert prompts.resolve_tokenizer_path() == prompts.DEFAULT_TOKENIZER_PATH
    monkeypatch.setenv(prompts.TOKENIZER_PATH_ENV, "/tmp/host-tokenizer")
    assert prompts.resolve_tokenizer_path() == "/tmp/host-tokenizer"
    assert prompts.resolve_tokenizer_path("/explicit/path") == "/explicit/path"


def test_policy_and_prompts_are_ros_free():
    """Constraint: ROS imports live only in planner_node.py. Checked on the
    TOP-LEVEL import list only, so prompts.load_tokenizer's deliberate lazy
    ``from transformers import ...`` inside the function body stays allowed
    (host import must not require transformers)."""
    base = pathlib.Path(prompts.__file__).parent
    for name in ("policy.py", "prompts.py"):
        source = (base / name).read_text(encoding="utf-8")
        tree = ast.parse(source)
        imported = set()
        for node in tree.body:
            if isinstance(node, ast.Import):
                imported.update(a.name.split(".")[0] for a in node.names)
            elif isinstance(node, ast.ImportFrom) and node.module:
                imported.add(node.module.split(".")[0])
        assert not imported & {"rclpy", "sensor_msgs", "geometry_msgs",
                               "std_msgs", "uav_planner_interfaces",
                               "uav_inference_interfaces", "transformers"}, name
