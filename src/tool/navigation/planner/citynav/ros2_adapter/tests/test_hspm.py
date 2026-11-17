"""hspm 单测：viewpoint 解析逐字语义、landmark JSON 解析、探索管线编排
（is_found 可见即推进、grounding 全失败→原地、alpha 收缩 waypoint）、
卡死判据逐字（sum(hist[-4:-1])）、LLM payload 对齐权威契约
（无 prompt/图像字段，view_stamps ns 整数）。"""
import numpy as np
import pytest

from uav_citynav_planner.hspm import (OBS_VIEWPOINTS, extract_landmarks, is_stuck,
                                      parse_landmark_response,
                                      parse_viewpoint_response_v2,
                                      plan_next_waypoint)
from uav_citynav_planner.perception import rotation_body_from_optical


def _batch(depth_value=2.0):
    K = np.array([[4.0, 0, 4.0], [0, 4.0, 4.0], [0, 0, 1.0]])
    batch = []
    for i, view in enumerate(OBS_VIEWPOINTS):
        rgb = np.zeros((8, 8, 3), np.uint8)
        depth = np.zeros((8, 8), np.float32)
        depth[4, 4] = depth_value
        batch.append({"view": view, "rgb": rgb, "depth": depth, "K": K,
                      "R": rotation_body_from_optical(0.0),
                      "pose": np.array([0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 1.0]),
                      "stamp": 100.0 + i, "stamp_ns": int((100.0 + i) * 10**9)})
    return batch


def test_parse_viewpoint_response_v2_verbatim():
    resp = "```json\n{\"is_found\": True, \"slightly left\": \"building\"}\n```"
    out = parse_viewpoint_response_v2(resp)
    assert out["is_found"] is True
    assert out["slightly_left"] == "building"
    # 缺 is_found → False（上游警告语义）
    out2 = parse_viewpoint_response_v2('{"front": "tree"}')
    assert out2["is_found"] is False and out2["front"] == "tree"


def test_parse_landmark_response():
    resp = ('blah\n```json\n{"Landmark sequence": ["red car", "tower"], '
            '"Thought": "x"}\n```')
    assert parse_landmark_response(resp) == ["red car", "tower"]
    assert parse_landmark_response("no json here") is None
    assert parse_landmark_response('```json\n{"Landmark sequence": []}\n```') is None


def test_extract_landmarks_payload_and_text_field():
    seen = {}

    def llm(op, timeout_s, **payload):
        seen["op"] = op
        seen.update(payload)
        return {"ok": True, "op": op,
                "text": '```json\n{"Landmark sequence": ["a","b"],'
                        '"Thought":"t"}\n```',
                "error_code": "ok", "latency_s": 1.2}
    res = extract_landmarks(llm, "go past a then b", 30.0)
    assert res["ok"] and res["landmarks"] == ["a", "b"]
    # 权威契约：landmark 查询只发 instruction（提示词在代理侧构造）
    assert seen["op"] == "landmark" and seen["instruction"] == "go past a then b"
    assert "prompt" not in seen and "system_prompt" not in seen and "images" not in seen

    def llm_fail(op, timeout_s, **payload):
        return {"ok": False, "error_code": "busy"}
    assert extract_landmarks(llm_fail, "x", 30.0)["error_code"] == "busy"


def test_plan_next_waypoint_is_found_and_waypoint_math():
    # 前视 ψ=0、pose 原点、mask 像素中心 d=2 → ENU 点 (2,0,0)；
    # alpha=0.6 → (1.2,0,0)；z=0 < min_alt 1.2 → 钳制 1.2
    seen = {}

    def llm(op, timeout_s, **payload):
        seen["op"] = op
        seen.update(payload)
        return {"ok": True, "op": op,
                "text": '{"is_found": true, "front": "building"}',
                "error_code": "ok", "latency_s": 9.5}

    def grounding(rgb, text):
        mask = np.zeros((8, 8), np.uint8)
        mask[4, 4] = 255
        return mask, True

    res = plan_next_waypoint(llm, grounding, _batch(), "go to building",
                             ["building"], 0, np.array([0.0, 0.0, 0.0]),
                             alpha=0.6, min_altitude_m=1.2)
    assert res["ok"] and res["is_found"] and res["seg_success"]
    wp = res["waypoint"]
    assert wp[0] == pytest.approx(1.2) and wp[1] == pytest.approx(0.0)
    assert wp[2] == pytest.approx(1.2)
    assert wp[3] == pytest.approx(0.0)  # yaw=atan2(0,1.2)
    # 权威契约 payload：结构化字段 + view_stamps(ns)，无 prompt/图像
    assert seen["op"] == "viewpoint"
    assert seen["instruction"] == "go to building"
    assert seen["landmarks"] == ["building"] and seen["traversed"] == []
    assert seen["next"] == "building"
    assert set(seen["view_stamps"]) == set(OBS_VIEWPOINTS)
    assert all(isinstance(v, int) for v in seen["view_stamps"].values())
    assert seen["max_age_s"] == pytest.approx(90.0)
    for forbidden in ("prompt", "images", "image_b64"):
        assert forbidden not in seen


def test_plan_next_waypoint_seg_fail_keeps_is_found_advance():
    # 可见即推进：is_found=True 但 grounding 全失败 → waypoint None，
    # is_found 仍为 True（上游 L257-258 与分割成败无关，逐字保留）
    def llm(op, timeout_s, **payload):
        return {"ok": True, "text": '{"is_found": true, "front": "x"}',
                "error_code": "ok", "latency_s": 8.0}

    def grounding(rgb, text):
        return None, False

    res = plan_next_waypoint(llm, grounding, _batch(), "i", ["l"], 0,
                             np.array([0.0, 0.0, 1.5]))
    assert res["ok"] and res["is_found"] and res["waypoint"] is None
    assert not res["seg_success"]


def test_plan_next_waypoint_llm_error_and_bad_json():
    def llm_err(op, timeout_s, **payload):
        return {"ok": False, "error_code": "stale"}
    res = plan_next_waypoint(llm_err, lambda *a: (None, False), _batch(),
                             "i", ["l"], 0, np.array([0.0, 0.0, 1.5]))
    assert not res["ok"] and res["error_code"] == "stale"

    def llm_bad(op, timeout_s, **payload):
        return {"ok": True, "text": "not json at all"}
    res = plan_next_waypoint(llm_bad, lambda *a: (None, False), _batch(),
                             "i", ["l"], 0, np.array([0.0, 0.0, 1.5]))
    assert not res["ok"] and res["error_code"] == "parse"


def test_is_stuck_verbatim_criterion():
    # 上游 L534：len>=4 且 sum(hist[-4:-1])==0（最近 4 步中前 3 个）
    assert is_stuck([0, 0, 0, 5])
    assert not is_stuck([0, 0, 5, 0])   # 前 3 个含 5 → 未卡死
    assert not is_stuck([0, 0, 0])      # 不足 4 步
    assert not is_stuck([1, 1, 1, 1])
