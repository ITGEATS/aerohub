"""trajectory 映射单测：golden 基准来自 run-002 真实决策与上游命令参数
（golden_result.json: target_angle -7.84°、无旋转、vz_ned 1.636、duration 2.0）。

每个修订以 t=0 交接点开头（follow_path timeline.hpp 要求首点 t==0 且点数>=2），
其后是 rotate（若 |θ|>10°）与 forward 航点。
"""
import math

import pytest

from uav_spf_planner.projector import ActionPoint
from uav_spf_planner.trajectory import build_revision

GOLDEN_ACTION = ActionPoint(dx=-0.13763819204711733, dy=1.0, dz=-0.825829152282704)


def _handoff(r):
    assert r["points"][0]["t"] == 0.0
    return r["points"][0]


def test_golden_no_rotation_step():
    r = build_revision(GOLDEN_ACTION, 0.0, 0.0, 0.0, 0.0, 2.0, 30.0, 2.0)
    assert r is not None and not r["rotated"]
    assert r["theta_deg"] == pytest.approx(-7.84, abs=0.01)
    assert len(r["points"]) == 2  # handoff + forward
    h = _handoff(r)
    assert (h["x"], h["y"], h["z"], h["yaw"]) == (0.0, 0.0, 0.0, 0.0)
    assert (h["vx"], h["vy"], h["vz"]) == (0.0, 0.0, 0.0)
    p = r["points"][1]
    d_xy = math.hypot(GOLDEN_ACTION.dx, GOLDEN_ACTION.dy)
    assert p["t"] == pytest.approx(max(d_xy / 2.0, 2.0))  # =2.0，上游 min_command_duration
    travel = 2.0 * p["t"]
    # |θ|≤10° 不旋转：沿当前 yaw(=0) 直行（上游语义，多循环逐步收敛目标方位）
    assert p["x"] == pytest.approx(travel, abs=1e-9)
    assert p["y"] == pytest.approx(0.0, abs=1e-9)
    assert p["yaw"] == pytest.approx(0.0)
    # 上游 vz_ned=+1.636（下降）→ ENU vz=-1.636
    assert p["vz"] == pytest.approx(-1.636, abs=0.01)
    assert p["z"] == pytest.approx(-1.636 * p["t"], abs=0.02)


def test_rotation_then_forward():
    a = ActionPoint(dx=1.0, dy=1.0, dz=0.0)
    r = build_revision(a, 5.0, 5.0, 2.0, 0.0, 2.0, 30.0, 2.0)
    assert r["rotated"]
    assert len(r["points"]) == 3  # handoff + rotate + forward
    h, rot, fwd = r["points"]
    assert h["t"] == 0.0 and h["yaw"] == 0.0
    assert rot["t"] == pytest.approx(45.0 / 30.0)  # θ=45°, yaw_rate 30°/s
    assert (rot["x"], rot["y"], rot["z"]) == (5.0, 5.0, 2.0)  # 原地
    assert rot["yaw"] == pytest.approx(math.radians(45))
    assert fwd["yaw"] == pytest.approx(math.radians(45))
    d_xy = math.hypot(1.0, 1.0)
    T = max(d_xy / 2.0, 2.0)
    assert fwd["t"] == pytest.approx(1.5 + T)
    assert fwd["x"] == pytest.approx(5.0 + 2.0 * T * math.cos(math.radians(45)), abs=1e-9)


def test_noop_small_distance():
    assert build_revision(ActionPoint(0.001, 0.001, 0.0), 0, 0, 0, 0, 2.0, 30.0, 2.0) is None


def test_adaptive_multiplier_scales_velocity():
    a = ActionPoint(dx=0.0, dy=1.0, dz=0.0, velocity_multiplier=0.6)
    r = build_revision(a, 0, 0, 0, 0, 2.0, 30.0, 2.0)
    p = r["points"][1]
    assert p["vx"] == pytest.approx(1.2)  # base_velocity * 0.6
    assert p["t"] == pytest.approx(2.0)   # max(1/1.2, 2)


def test_pure_vertical_move():
    a = ActionPoint(dx=0.0, dy=0.0, dz=1.0)
    r = build_revision(a, 0, 0, 0, 0, 2.0, 30.0, 2.0)
    p = r["points"][1]
    assert p["t"] == pytest.approx(2.0)
    assert p["vz"] == 0.0  # 上游 d_xy≤0.01 时 vz=0（防除零，L120-124 语义）
    assert p["x"] == pytest.approx(4.0)  # 仍前飞 v*T（上游 vx=velocity_scale 恒定）
