"""SPF 反投影与深度映射（板端纯 stdlib，迁移方案阶段 4）。

语义来源（上游 5621bcf，逐行对照）：
  - 深度映射：src/spf/airsim/action_projector.py L154-161（obstacle 档
    depth = vlm_depth/10*2，缺省 4）与 L197-213（adaptive 三分支）。
  - 反投影：本实现按 CameraInfo 真针孔 (fx, fy, cx, cy) 计算（偏离 D3）；
    上游用硬编码 FOV=108 单因子。当 fx=(W/2)/tan54、fy=(H/2)/tan54 时两者
    逐点等价（tests/test_projector.py golden 验证），差异仅在真实 FOV≠108 时出现。
坐标系：输出机体系位移 (dx 右, dy 前, dz 上) 米，与上游 ActionPoint 一致。
"""
import math
from dataclasses import dataclass
from typing import Optional


@dataclass
class ActionPoint:
    dx: float  # right, m
    dy: float  # forward, m
    dz: float  # up, m
    velocity_multiplier: float = 1.0
    vlm_depth: Optional[float] = None
    adaptive_depth: Optional[float] = None


def calculate_adaptive_depth(vlm_depth: float) -> float:
    """逐语义移植上游 _calculate_adaptive_depth（airsim/action_projector.py L197-213）。"""
    if vlm_depth <= 2:
        return 0.0
    if vlm_depth <= 5:
        return (vlm_depth / 10.0) * 2
    return 1.0 + (vlm_depth - 5) / 5.0


def project(point_yx, depth_value, fx: float, fy: float, cx: float, cy: float,
            width: int, height: int, adaptive_mode: bool) -> Optional[ActionPoint]:
    """VLM 标注 → 机体系位移。

    point_yx: 上游 JSON 的 [y, x]，0-1000 归一化。
    depth_value: 上游 JSON 的 depth（缺失时调用方传 4，同上游缺省）。
    返回 None 表示不产生动作（adaptive 深度 0 / 非法内参）。
    """
    if fx <= 0 or fy <= 0 or width <= 0 or height <= 0:
        return None
    y_norm, x_norm = float(point_yx[0]), float(point_yx[1])
    vlm_depth = 4.0 if depth_value is None else float(depth_value)

    # 上游 L150-152：归一化 → 像素（整型截断语义保留）
    pixel_x = int((x_norm / 1000.0) * width)
    pixel_y = int((y_norm / 1000.0) * height)

    if adaptive_mode:
        adaptive_depth = calculate_adaptive_depth(vlm_depth)
        if adaptive_depth == 0:
            return None  # 上游：过近，不产生移动命令
        depth_for_projection = adaptive_depth
        velocity_multiplier = adaptive_depth
    else:
        adaptive_depth = None
        depth_for_projection = vlm_depth / 10.0 * 2.0
        velocity_multiplier = 1.0

    # 真针孔反投影（光心像素坐标，y 向下）；dz 上为正 = -(v-cy)
    dx = (pixel_x - cx) * depth_for_projection / fx
    dz = (cy - pixel_y) * depth_for_projection / fy
    return ActionPoint(
        dx=dx, dy=depth_for_projection, dz=dz,
        velocity_multiplier=velocity_multiplier,
        vlm_depth=vlm_depth, adaptive_depth=adaptive_depth,
    )
