"""http_client 纯函数与解析失败路径单测（无网络）。"""
import pytest

from uav_spf_vlm_proxy.http_client import (
    VlmHttpError, build_gemini_payload, build_openai_payload,
    extract_gemini_content, extract_openai_content)
from uav_spf_vlm_proxy.prompt import VlmParseError, parse_point_response


def test_openai_payload_matches_upstream_shape():
    p = build_openai_payload("PROMPT", "QklDRQ==", "glm-4v-flash", 0.4, 1024)
    assert p["model"] == "glm-4v-flash"
    assert p["temperature"] == 0.4 and p["max_tokens"] == 1024
    content = p["messages"][0]["content"]
    assert content[0] == {"type": "text", "text": "PROMPT"}
    assert content[1]["image_url"]["url"] == "data:image/jpeg;base64,QklDRQ=="


def test_extract_openai_content():
    assert extract_openai_content(
        {"choices": [{"message": {"content": "hello"}}]}) == "hello"
    assert extract_openai_content(
        {"choices": [{"message": {"content": None}}]}) == ""
    with pytest.raises(VlmHttpError) as e:
        extract_openai_content({"choices": []})
    assert e.value.code == "bad_payload"


def test_extract_gemini_content():
    assert extract_gemini_content({"candidates": [{"content": {"parts": [
        {"text": "a"}, {"text": "b"}]}}]}) == "ab"
    with pytest.raises(VlmHttpError):
        extract_gemini_content({})


def test_gemini_payload_generation_config():
    p = build_gemini_payload("P", "Qg==", 0.4, 0.95, 40, 8192)
    assert p["generationConfig"] == {"temperature": 0.4, "topP": 0.95,
                                     "topK": 40, "maxOutputTokens": 8192}
    assert p["contents"][1]["parts"][0]["inline_data"]["mime_type"] == "image/jpeg"


def test_parse_point_response_ok():
    r = parse_point_response('[{"point": [450, 800], "depth": 5, "label": "ball"}]')
    assert r == {"point": [450, 800], "depth": 5, "label": "ball"}


def test_parse_point_response_depth_default():
    r = parse_point_response('[{"point": [1, 2], "label": "x"}]')
    assert r["depth"] == 4  # 上游 point_info.get("depth", 4)


def test_parse_point_response_markdown():
    r = parse_point_response('```json\n[{"point":[1,2],"depth":3,"label":"l"}]\n```')
    assert r["depth"] == 3


def test_parse_error_codes():
    for text, code in [("", "empty_response"), ("   ", "empty_response"),
                       ("not json", "bad_json"), ("{'point':[1,2]}", "bad_json"),
                       ("[]", "no_points")]:
        with pytest.raises(VlmParseError) as e:
            parse_point_response(text)
        assert e.value.code == code
