"""projector 语义单测：golden 基准来自 legacy 阶段真实决策
（log/spf/legacy/run-002-smoke-pass/decision_20261112_195523.json，上游 5621bcf 输出）。
"""
import math

import pytest

from uav_spf_planner.projector import ActionPoint, calculate_adaptive_depth, project

W, H = 1920, 1080
# 上游 108° 单因子假设的等价针孔内参：fx=(W/2)/tan54, fy=(H/2)/tan54
FX_U = (W / 2) / math.tan(math.radians(54))
FY_U = (H / 2) / math.tan(math.radians(54))
CX, CY = W / 2.0, H / 2.0

# run-002 真实决策：point [y,x]=[800,450]（归一化），vlm_depth=5
GOLDEN_POINT = [800, 450]
GOLDEN_DX = -0.13763819204711733
GOLDEN_DZ = -0.825829152282704


def test_golden_matches_upstream_when_intrinsics_assumed():
    a = project(GOLDEN_POINT, 5, FX_U, FY_U, CX, CY, W, H, adaptive_mode=False)
    assert a is not None
    assert abs(a.dx - GOLDEN_DX) < 1e-9
    assert abs(a.dz - GOLDEN_DZ) < 1e-9
    assert a.dy == pytest.approx(1.0)  # 5/10*2
    assert a.velocity_multiplier == 1.0


def test_d3_fov_mismatch_quantified():
    """真实 AirSim 90° hfov 方形像素内参 vs 上游 108° 假设的方向差（D3 偏差表样例）。"""
    fx90 = (W / 2) / math.tan(math.radians(45))  # =960
    a_real = project(GOLDEN_POINT, 5, fx90, fx90, CX, CY, W, H, adaptive_mode=False)
    a_up = project(GOLDEN_POINT, 5, FX_U, FY_U, CX, CY, W, H, adaptive_mode=False)
    ang_real = math.atan2(a_real.dx, a_real.dy)
    ang_up = math.atan2(a_up.dx, a_up.dy)
    assert abs(ang_real - ang_up) > 1e-3  # 中心外点方向必然分歧，阶段 2 出全表
    assert a_real.dz > a_up.dz  # 90° 垂直视场更小，同像素行的上仰角更缓


def test_depth_default_4_when_missing():
    a = project([500, 500], None, FX_U, FY_U, CX, CY, W, H, adaptive_mode=False)
    assert a.dy == pytest.approx(4 / 10 * 2)


def test_adaptive_branches():
    assert calculate_adaptive_depth(1) == 0.0
    assert calculate_adaptive_depth(2) == 0.0
    assert calculate_adaptive_depth(3) == pytest.approx(0.6)
    assert calculate_adaptive_depth(5) == pytest.approx(1.0)
    assert calculate_adaptive_depth(6) == pytest.approx(1.2)
    assert calculate_adaptive_depth(10) == pytest.approx(2.0)


def test_adaptive_zero_depth_no_action():
    assert project([800, 450], 2, FX_U, FY_U, CX, CY, W, H, adaptive_mode=True) is None


def test_adaptive_multiplier_propagates():
    a = project([800, 450], 6, FX_U, FY_U, CX, CY, W, H, adaptive_mode=True)
    assert a.velocity_multiplier == pytest.approx(1.2)
    assert a.dy == pytest.approx(1.2)
    assert a.adaptive_depth == pytest.approx(1.2)


def test_invalid_intrinsics_rejected():
    assert project([500, 500], 5, 0.0, 1.0, 0.0, 0.0, W, H, adaptive_mode=False) is None
