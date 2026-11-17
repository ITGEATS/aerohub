"""parse_viewpoint_response_v2 行为测试（上游 L30-54 逐字复制的语义固化）。"""
import pytest

from uav_citynav_llm_proxy.prompts import parse_viewpoint_response_v2


def test_plain_json_found():
    r = parse_viewpoint_response_v2(
        '{"is_found": true, "slightly left": "orange ball"}')
    assert r == {"is_found": True, "slightly_left": "orange ball"}


def test_markdown_fenced_and_python_bool():
    # 模型常输出 ```json 围栏与 Python 风格 True/False，上游 strip/replace 处理
    r = parse_viewpoint_response_v2(
        '```json\n{"is_found": True, "front": "red building", '
        '"left": "car", "slightly right": "tree", "right": "road"}\n```')
    assert r["is_found"] is True
    assert r["front"] == "red building"
    assert r["left"] == "car"
    assert r["slightly_right"] == "tree"
    assert r["right"] == "road"


def test_missing_is_found_defaults_false(capsys):
    r = parse_viewpoint_response_v2('{"front": "ball"}')
    assert r["is_found"] is False
    assert "Warning" in capsys.readouterr().out


def test_label_reformat_variants():
    # 上游分支是大小写敏感的子串匹配（prompt 示例即小写视角名）
    r = parse_viewpoint_response_v2(
        '{"is_found": false, "slightly left view": "a", "the front": "b", '
        '"right side": "c"}')
    assert r["slightly_left"] == "a"
    assert r["front"] == "b"
    assert r["right"] == "c"


def test_bad_json_raises_value_error():
    # node 侧把 ValueError(JSONDecodeError) 映射为 error_code=parse
    with pytest.raises(ValueError):
        parse_viewpoint_response_v2("not a json at all")
