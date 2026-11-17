"""SPF VLM prompt 与响应解析。

prompt 模板与 clean_response_text 逐字复制自上游锁定提交
5621bcf43e9826d60df014541dd0498e743a92bd：
  - src/spf/airsim/action_projector.py L111-132（f-string 模板）
  - src/spf/clients/vlm_client.py L138-149（clean_response_text）
复制原因：复现保真（迁移方案阶段 3 与许可证裁决③，仅限组内非商业研究）。
任何修改都会破坏与上游决策的等价性；tests/test_prompt_parity.py 对上游源文件
做逐字比对，防止漂移。
"""
import json

PROMPT_TEMPLATE = """You are a drone navigation expert analyzing a drone camera view.

Task: {instruction}

First, identify ALL objects in the image that match the description "{instruction}".
Then, select the MOST RELEVANT target object and place a single point DIRECTLY ON that object.

Return in this exact JSON format:
[{{"point": [y, x], "depth": depth_value, "label": "action description"}}]

Coordinate system:
- x: 0-1000 scale (500=center, >500=right, <500=left)
- y: 0-1000 scale (lower values=higher in image/sky)
- depth: 1-10 scale where:
    * 1: Object is very close/large in frame
    * 10: Object is far away/small in frame

IMPORTANT:
- Place the point PRECISELY on the center of the target object
- Choose the largest/closest matching object if multiple exist
- Assess the depth based on how much of the frame the object occupies
- Your accuracy in point placement is critical for navigation success"""


def build_prompt(instruction: str) -> str:
    return PROMPT_TEMPLATE.format(instruction=instruction)


def clean_response_text(response_text: str) -> str:
    # 逐字复制自上游 VLMClient.clean_response_text（staticmethod 行为一致）
    if "```json" in response_text:
        json_start = response_text.find("```json") + 7
        json_end = response_text.find("```", json_start)
        response_text = response_text[json_start:json_end].strip()
    elif "```" in response_text:
        json_start = response_text.find("```") + 3
        json_end = response_text.find("```", json_start)
        response_text = response_text[json_start:json_end].strip()
    return response_text


class VlmParseError(Exception):
    """error_code 与上游失败路径一一对应：
    empty_response —— 上游 "Expecting value: line 1 column 1"（空文本）
    bad_json       —— 上游 JSONDecodeError（非标准 JSON）
    no_points      —— 上游 "No points returned from VLM"（空列表）
    """

    def __init__(self, code: str, detail: str = ""):
        super().__init__(f"{code}: {detail}")
        self.code = code
        self.detail = detail


def parse_point_response(response_text: str) -> dict:
    """按上游 _get_single_action 的解析顺序提取首个标注点。

    返回 {"point": [y, x], "depth": float, "label": str}；
    depth 缺省 4（上游 point_info.get("depth", 4) 语义）。
    反投影不在代理内做——板端 planner 以 CameraInfo 真针孔完成（偏离 D3）。
    """
    if not response_text or not response_text.strip():
        raise VlmParseError("empty_response", "model returned no content")
    cleaned = clean_response_text(response_text)
    try:
        points_data = json.loads(cleaned)
    except json.JSONDecodeError as exc:
        raise VlmParseError("bad_json", str(exc))
    if not points_data:
        raise VlmParseError("no_points", "empty points list")
    point_info = points_data[0]
    return {
        "point": point_info["point"],
        "depth": point_info.get("depth", 4),
        "label": point_info.get("label", ""),
    }
