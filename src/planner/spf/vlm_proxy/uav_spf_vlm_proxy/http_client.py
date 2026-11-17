"""纯 stdlib VLM HTTP 客户端（偏离 D2：SDK 依赖 pydantic-core，PC/板端统一 stdlib）。

请求语义与上游 VLMClient 对齐：
  - openai provider：POST {base_url}/chat/completions，messages 结构同上游
    _get_openai_response（text + data:image/jpeg;base64 image_url），
    temperature=0.4，extra_headers HTTP-Referer/X-Title 同上游。
    max_tokens 上游 8192；智谱上限 1024（上游 dirty diff 同款适配），可配置。
  - gemini provider：REST generateContent，contents=[prompt, inline_data]，
    generationConfig 同上游 generation_config（temperature/top_p/top_k/max_output_tokens）。
"""
import json
import urllib.error
import urllib.request

UPSTREAM_HEADERS = {
    "HTTP-Referer": "https://spf-web.pages.dev",
    "X-Title": "See, Point, Fly",
}


class VlmHttpError(Exception):
    def __init__(self, code: str, detail: str = ""):
        super().__init__(f"{code}: {detail}")
        self.code = code  # http_error / timeout / transport_error / bad_payload
        self.detail = detail


def build_openai_payload(prompt: str, jpeg_b64: str, model: str,
                         temperature: float, max_tokens: int) -> dict:
    return {
        "model": model,
        "messages": [{
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {"type": "image_url",
                 "image_url": {"url": f"data:image/jpeg;base64,{jpeg_b64}"}},
            ],
        }],
        "temperature": temperature,
        "max_tokens": max_tokens,
    }


def build_gemini_payload(prompt: str, jpeg_b64: str, temperature: float,
                         top_p: float, top_k: int, max_output_tokens: int) -> dict:
    return {
        "contents": [
            {"role": "user", "parts": [{"text": prompt}]},
            {"role": "user", "parts": [{"inline_data":
                {"mime_type": "image/jpeg", "data": jpeg_b64}}]},
        ],
        "generationConfig": {
            "temperature": temperature,
            "topP": top_p,
            "topK": top_k,
            "maxOutputTokens": max_output_tokens,
        },
    }


def extract_openai_content(resp_json: dict) -> str:
    try:
        return resp_json["choices"][0]["message"]["content"] or ""
    except (KeyError, IndexError, TypeError) as exc:
        raise VlmHttpError("bad_payload", f"openai schema: {exc}")


def extract_gemini_content(resp_json: dict) -> str:
    try:
        parts = resp_json["candidates"][0]["content"]["parts"]
        return "".join(p.get("text", "") for p in parts)
    except (KeyError, IndexError, TypeError) as exc:
        raise VlmHttpError("bad_payload", f"gemini schema: {exc}")


def post_json(url: str, payload: dict, headers: dict, timeout_s: float) -> dict:
    data = json.dumps(payload).encode("utf-8")
    req = urllib.request.Request(url, data=data, method="POST")
    req.add_header("Content-Type", "application/json")
    for k, v in headers.items():
        req.add_header(k, v)
    try:
        with urllib.request.urlopen(req, timeout=timeout_s) as resp:
            return json.loads(resp.read().decode("utf-8"))
    except urllib.error.HTTPError as exc:
        body = ""
        try:
            body = exc.read().decode("utf-8", errors="replace")[:400]
        except Exception:
            pass
        raise VlmHttpError("http_error", f"{exc.code} {body}")
    except urllib.error.URLError as exc:
        raise VlmHttpError("timeout" if isinstance(getattr(exc, "reason", None), TimeoutError)
                           or "timed out" in str(exc.reason).lower() else "transport_error",
                           str(exc.reason))
    except json.JSONDecodeError as exc:
        raise VlmHttpError("bad_payload", f"non-json response: {exc}")


def request_point_text(provider: str, api_key: str, base_url: str, model: str,
                       prompt: str, jpeg_b64: str, temperature: float,
                       max_tokens: int, timeout_s: float) -> str:
    """一次标注请求，返回模型原始文本；失败抛 VlmHttpError。"""
    if provider == "openai":
        url = base_url.rstrip("/") + "/chat/completions"
        payload = build_openai_payload(prompt, jpeg_b64, model, temperature, max_tokens)
        headers = {**UPSTREAM_HEADERS, "Authorization": f"Bearer {api_key}"}
        resp = post_json(url, payload, headers, timeout_s)
        return extract_openai_content(resp)
    elif provider == "gemini":
        url = (f"https://generativelanguage.googleapis.com/v1beta/models/"
               f"{model}:generateContent?key={api_key}")
        payload = build_gemini_payload(prompt, jpeg_b64, temperature, 0.95, 40, max_tokens)
        resp = post_json(url, payload, {}, timeout_s)
        return extract_gemini_content(resp)
    raise VlmHttpError("bad_payload", f"unknown provider {provider}")
