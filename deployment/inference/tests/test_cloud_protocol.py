"""http_client 纯函数与解析失败路径单测（无网络）。"""
import pytest

from aerohub_inference.backends.cloud import (
    CloudError as VlmHttpError, build_gemini_payload, build_openai_payload, extract_text)

def extract_openai_content(body):
    return extract_text("openai", body)

def extract_gemini_content(body):
    return extract_text("gemini", body)


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
    assert e.value.code == "BAD_PAYLOAD"


def test_extract_gemini_content():
    assert extract_gemini_content({"candidates": [{"content": {"parts": [
        {"text": "a"}, {"text": "b"}]}}]}) == "ab"
    with pytest.raises(VlmHttpError):
        extract_gemini_content({})


def test_gemini_payload_generation_config():
    p = build_gemini_payload("P", "Qg==", 0.4, 8192)
    assert p["generationConfig"] == {"temperature": 0.4, "topP": 0.95,
                                     "topK": 40, "maxOutputTokens": 8192}
    assert p["contents"][0]["parts"][1]["inline_data"]["mime_type"] == "image/jpeg"
