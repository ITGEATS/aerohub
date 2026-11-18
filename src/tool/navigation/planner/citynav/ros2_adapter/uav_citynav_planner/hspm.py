"""HSPM 层级语义规划（纯逻辑编排，客户端以回调注入，无 ROS 依赖）。

语义来源：
  - landmark 抽取（op=landmark）：payload 仅 {instruction}；system/user 提示词
    由 uav_citynav_llm_proxy 内部构造（prompts.landmark_extraction_messages），
    planner 不发送任何 prompt 文本（权威契约，proxy node.py L13-17）。响应解析
    逐字复制 upstream/utils/landmark_extraction.py L56-65（```json 正则 +
    "Landmark sequence"）。
  - viewpoint 路线查询（op=viewpoint）：payload
    {instruction, landmarks, traversed, next, view_stamps, max_age_s}；
    route_planning_prompt_builder 与图像消息由代理按 view_stamps 从其相机缓存
    取帧后内部构造（决策：图像数据流止步算力容器/PC GPU 之外的代理侧，公网
    访问仅代理出板，README §3）。本包保留 parse_viewpoint_response_v2 一份
    逐字复制用于字段提取——代理已校验响应可解析（error_code=parse），但
    viewpoint/object 字段消费在 planner。
  - 主链：SimRun.py L227-321 explore_pipeline_by_sam——五视角名
    left/slightly_left/front/slightly_right/right、逐视角 grounding、
    成功即反投影、>30 点统计滤波、只取首个非空类、alpha=0.6 收缩、
    is_found 推进（可见即推进，与分割成败无关，L257-258/L525-526 逐字保留）、
    卡死判据（最近 4 步位移和为 0，L534）由 node.py 消费本模块 steps。

跨部署目标不互相 import 的理由：uav_citynav_llm_proxy 运行在工作站容器、
本包运行在 onboard-computer 容器，两包独立构建安装（colcon overlay 不重叠），
共享 prompt/解析源段以"各自逐字复制 + 测试期 AST 比对 upstream 锁定提交
4b6e926c"防漂移（tests/test_prompts_alignment.py），不做运行时跨包依赖。
"""
import json
import math
import re
from typing import Callable, Dict, List, Optional

import numpy as np

from .perception import backproject_view, semantic_waypoint, statistical_filter, yaw_to_point

OBS_VIEWPOINTS = ["left", "slightly_left", "front", "slightly_right", "right"]


def parse_viewpoint_response_v2(resp):
    """upstream/src/llm/prompt_builder.py L30-54 逐字复制（含 print 警告；
    tests/test_prompts_alignment.py 对 upstream 源做 AST 比对防漂移）。"""
    cleaned_resp = resp.strip('`json\n').strip('`')
    cleaned_resp = cleaned_resp.replace("True", "true").replace("False", "false")

    json_resp = json.loads(cleaned_resp)
    reformat_resp = {}
    if "is_found" not in json_resp:
        print("Warning: LLM output missing 'is_found'")
        reformat_resp["is_found"] = False
    else:
        reformat_resp["is_found"] = json_resp["is_found"]

    for k in json_resp:
        if "slightly" in k and "left" in k:
            reformat_resp["slightly_left"] = json_resp[k]
        elif "slightly" in k and "right" in k:
            reformat_resp["slightly_right"] = json_resp[k]
        elif "left" in k:
            reformat_resp["left"] = json_resp[k]
        elif "right" in k:
            reformat_resp["right"] = json_resp[k]
        elif "front" in k:
            reformat_resp["front"] = json_resp[k]

    return reformat_resp


def parse_landmark_response(resp: str) -> Optional[List[str]]:
    """landmark_extraction.py L56-65 逐字移植（```json 正则 + 字段名）。"""
    match = re.search(r'```json\s*(\{.*\})', resp, re.DOTALL)
    if not match:
        return None
    try:
        res_json = json.loads(match.group(1))
    except ValueError:
        return None
    seq = res_json.get("Landmark sequence")
    if not isinstance(seq, list) or not seq:
        return None
    return [str(s) for s in seq]


def extract_landmarks(llm_request: Callable[..., Dict], instruction: str,
                      timeout_s: float) -> Dict:
    """op=landmark：landmark 序列 LLM 抽取（决策 7；提示词在代理侧）。

    llm_request(op=..., timeout_s=..., **payload) → result dict
    （{"ok","text","error_code","latency_s","op","request_id"}，权威契约见包
    README）。
    """
    result = llm_request(op="landmark", timeout_s=timeout_s,
                         instruction=instruction)
    if not result.get("ok"):
        return {"ok": False, "error_code": result.get("error_code", "llm_fail")}
    landmarks = parse_landmark_response(result.get("text", ""))
    if landmarks is None:
        return {"ok": False, "error_code": "parse"}
    return {"ok": True, "landmarks": landmarks}


def plan_next_waypoint(llm_request: Callable[..., Dict],
                       grounding_predict: Callable[[np.ndarray, str], tuple],
                       batch: List[Dict], instruction: str,
                       landmarks: List[str], next_landmark_idx: int,
                       cur_pose: np.ndarray, alpha: float = 0.6,
                       min_altitude_m: Optional[float] = 1.2,
                       llm_timeout_s: float = 30.0,
                       max_age_s: float = 90.0,
                       logger=None) -> Dict:
    """SimRun.py L227-321 explore_pipeline_by_sam 的 AS2 等价编排。

    batch 项：{"view", "rgb"(uint8 HxWx3), "depth"(float32 HxW 米 axial),
    "K"(3x3), "R"(3x3 base←optical), "pose"([x,y,z,qx,qy,qz,qw] 与图像 stamp
    匹配), "stamp"(s), "stamp_ns"(int)}。cur_pose：[x,y,z] odom ENU。
    viewpoint 查询只发结构化字段与 view_stamps（各视角图像 stamp ns），图像与
    prompt 由代理构造。返回 {"ok","waypoint"([x,y,z,yaw]|None),"is_found",
    "seg_success","selected","error_code"}。
    """
    views = {b["view"]: b for b in batch}

    traversed_landmarks = landmarks[:next_landmark_idx]
    view_stamps = {v: int(views[v]["stamp_ns"]) for v in OBS_VIEWPOINTS if v in views}
    result = llm_request(op="viewpoint", timeout_s=llm_timeout_s,
                         instruction=instruction,
                         landmarks=list(landmarks),
                         traversed=list(traversed_landmarks),
                         next=landmarks[next_landmark_idx],
                         view_stamps=view_stamps,
                         max_age_s=float(max_age_s))
    if not result.get("ok"):
        return {"ok": False, "error_code": result.get("error_code", "llm_fail")}
    try:
        route_predicted_dict = parse_viewpoint_response_v2(result.get("text", ""))
    except ValueError:
        return {"ok": False, "error_code": "parse"}

    next_subgoal_found = bool(route_predicted_dict.get("is_found", False))

    semantic_pc: List[np.ndarray] = []
    seg_succ_all = False
    selected = []
    for vp, obj in route_predicted_dict.items():
        if vp == "is_found":
            continue
        view = views.get(vp)
        if view is None:
            if logger:
                logger.warning("unknown viewpoint %s from LLM" % vp)
            continue
        mask, seg_succ = grounding_predict(view["rgb"], str(obj))
        seg_succ_all = seg_succ_all or seg_succ
        if seg_succ:
            if logger:
                logger.info("Selected viewpoint, object: %s, %s" % (vp, obj))
            selected.append({"view": vp, "object": obj})
            part_pc = backproject_view(view["depth"], view["K"], mask,
                                        view["R"], view["pose"])
            semantic_part_pc = part_pc
            if len(semantic_part_pc) > 30:
                semantic_part_pc, _ = statistical_filter(semantic_part_pc)
            if len(semantic_part_pc) > 0:
                semantic_pc.append(semantic_part_pc)

    cur_pos = np.asarray(cur_pose[:3], dtype=np.float64)
    route_coords, ok = semantic_waypoint(semantic_pc, cur_pos, alpha=alpha,
                                         min_altitude_m=min_altitude_m)
    if not ok:
        waypoint = None
        steps_estimate = 0
    else:
        yaw = yaw_to_point(cur_pos, route_coords)
        waypoint = [float(route_coords[0]), float(route_coords[1]),
                    float(route_coords[2]), float(yaw)]
        # SimRun.py L316-317 step 预算估计（逐字：|yaw°|//15 + dz//2 + dxy//5）
        rel = route_coords - cur_pos
        steps_estimate = int(np.abs(np.rad2deg(yaw)) // 15 + np.abs(rel[2]) // 2
                             + math.hypot(rel[0], rel[1]) // 5)
    return {"ok": True, "waypoint": waypoint, "is_found": next_subgoal_found,
            "seg_success": seg_succ_all, "selected": selected,
            "steps_estimate": steps_estimate}


def is_stuck(hist_step_size: List[int]) -> bool:
    """SimRun.py L534 逐字：len>=4 且 sum(hist[-4:-1]) == 0。"""
    return len(hist_step_size) >= 4 and sum(hist_step_size[-4:-1]) == 0
