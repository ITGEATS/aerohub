"""trajectory 单测：calculate_movement_steps 分解顺序（15° 偏航→5 m 水平→
2 m 垂直，顺序=时间顺序）与步数，build_timed_path 分段时长
（TURN 15°→0.5 s@30°/s、FORWARD 5 m→5 s@1.0 m/s、垂直 vz=v·dz/dxy 比例、
15° 子步合并单段时长不变）。"""
import math

import pytest

from uav_citynav_planner.trajectory import build_timed_path, calculate_movement_steps

D15 = math.radians(15.0)


def test_decomposition_order_and_counts():
    # A=(0,0,1.5,yaw0) → B=(10,5,3.5,yaw=30°)
    # 偏航：rot_angle=30° → 子步 [0,15,30] 共 3 项（位置不变）
    # 水平：h_dist=√125≈11.18 → int(11.18//5)=2 整步 + 终点 = 3 项
    # 垂直：v_dist=2 → int(2//2)=1 整步 + 终点 = 2 项（第 2 项与第 1 项重合，
    #       上游 [v_ss*(i+1)]+[z_dist] 形态逐字保留 → 2 项）
    n, path = calculate_movement_steps([0, 0, 1.5, 0.0, 0.0, 0.0],
                                       [10, 5, 3.5, 0.0, 0.0, math.radians(30)])
    assert n == 8
    # 偏航段：位置恒为 A，yaw 依次 0/15/30°
    for i, expect_yaw in enumerate([0.0, D15, 2 * D15]):
        assert path[i][:3] == pytest.approx([0.0, 0.0, 1.5])
        assert path[i][5] == pytest.approx(expect_yaw, abs=1e-9)
    # 水平段：沿 A→B 直线 5 m 步进，z 保持 1.5，yaw 保持 30°
    d = math.hypot(10, 5)
    ux, uy = 10 / (d + 1e-6), 5 / (d + 1e-6)  # 上游 h_ori 分式含 +1e-6（L243）
    assert path[3][:2] == pytest.approx([5 * ux, 5 * uy], abs=1e-9)
    assert path[4][:2] == pytest.approx([10 * ux, 10 * uy], abs=1e-9)
    assert path[5][:2] == pytest.approx([10.0, 5.0], abs=1e-9)
    for it in path[3:6]:
        assert it[2] == pytest.approx(1.5)
        assert it[5] == pytest.approx(2 * D15, abs=1e-9)
    # 垂直段：xy 恒为 B，z 依次 +2 与终点 3.5
    assert path[6][:3] == pytest.approx([10.0, 5.0, 3.5])
    assert path[7][:3] == pytest.approx([10.0, 5.0, 3.5])


def test_yaw_wrap_and_shortest_direction():
    # A yaw=+170°, B yaw=-170°：delta=-340° → rot_angle=20° 正方向（上游
    # rot_reverse 语义：|delta|≥π 走另一侧）
    n, path = calculate_movement_steps([0, 0, 1, 0, 0, math.radians(170)],
                                       [0, 0, 1, 0, 0, math.radians(-170)])
    yaws = [p[5] for p in path]
    assert yaws[0] == pytest.approx(math.radians(170), abs=1e-9)
    assert yaws[-1] == pytest.approx(math.radians(-170), abs=1e-9)  # wrap 后
    # 20° 旋转 → 子步 [0°,15°,20°] 共 3 项（170°/-175°/-170° wrap 序列）
    assert len(path) == 3
    assert yaws[1] == pytest.approx(math.radians(-175), abs=1e-9)


def test_pure_forward_5m_steps():
    n, path = calculate_movement_steps([0, 0, 1, 0, 0, 0.0], [10, 0, 1, 0, 0, 0.0])
    # 偏航 delta=0 → 1 项（0°）；水平 2 整步 + 终点 = 3 项；无垂直
    assert n == 4
    assert [p[0] for p in path] == pytest.approx([0.0, 5.0, 10.0, 10.0])


def test_timed_path_horizontal_forward_duration():
    # FORWARD 5 m → 5 s@1.0 m/s（契约 §5）；12 m 目标 = 整步 5,10 + 终点 12
    timed = build_timed_path([[12.0, 0.0, 1.5, 0.0]], (0.0, 0.0, 1.5, 0.0),
                             base_velocity=1.0, base_yaw_rate=30.0)
    pts = timed["points"]
    assert pts[0]["t"] == 0.0  # 交接点（timeline 首点 t==0、点数>=2）
    assert [p["t"] for p in pts] == pytest.approx([0.0, 5.0, 10.0, 12.0])
    assert pts[-1]["x"] == pytest.approx(12.0)
    assert pts[1]["vx"] == pytest.approx(1.0)
    assert timed["steps"] == 4  # 偏航 1（零旋转占位，上游形态）+ 水平 3
    assert timed["duration"] == pytest.approx(12.0)


def test_timed_path_rotation_merged_segment():
    # 偏航 45°：分解 4 个 15° 子步 → 合并为单段连续旋转，时长不变
    # 45°/30°/s = 1.5 s（契约 §5：TURN 15°→0.5 s，3 子步合计 1.5 s）
    timed = build_timed_path([[0.0, 0.0, 1.5, math.radians(45)]],
                             (0.0, 0.0, 1.5, 0.0), 1.0, 30.0)
    pts = timed["points"]
    assert len(pts) == 2  # 交接 + 合并旋转段
    assert pts[1]["t"] == pytest.approx(1.5)
    assert (pts[1]["x"], pts[1]["y"], pts[1]["z"]) == (0.0, 0.0, 1.5)
    assert pts[1]["yaw"] == pytest.approx(math.radians(45))
    assert (pts[1]["vx"], pts[1]["vy"], pts[1]["vz"]) == (0.0, 0.0, 0.0)


def test_timed_path_order_yaw_then_horizontal_then_vertical():
    # 偏航→水平→垂直 顺序即时间顺序（上游分解顺序，README §1）
    timed = build_timed_path([[10.0, 0.0, 3.5, math.radians(30)]],
                             (0.0, 0.0, 1.5, 0.0), 1.0, 30.0)
    pts = timed["points"]
    # t0 交接，t1 旋转完毕（30°/30°/s=1 s，位置不变）
    assert pts[1]["t"] == pytest.approx(1.0)
    assert pts[1]["yaw"] == pytest.approx(math.radians(30))
    # 水平段在旋转之后（位置开始变化）
    assert pts[2]["x"] > 0.0 and pts[2]["z"] == pytest.approx(1.5)
    # 垂直段在最后（xy 不动、z 上升）；vz=v·dz/dxy=1·2/10=0.2 → 下限保护 0.2>0.1
    last = pts[-1]
    assert last["z"] == pytest.approx(3.5)
    assert last["vz"] == pytest.approx(0.2)
    # 垂直段总时长 = dz_total/|vz| = 2/0.2 = 10 = dxy/v（比例语义）
    assert last["t"] == pytest.approx(1.0 + 10.0 + 10.0)


def test_timed_path_pure_vertical_fallback_velocity():
    timed = build_timed_path([[0.0, 0.0, 5.5, 0.0]], (0.0, 0.0, 1.5, 0.0),
                             1.0, 30.0)
    pts = timed["points"]
    assert pts[-1]["z"] == pytest.approx(5.5)
    # dxy=0 → vz=base_velocity；4 m 上升 → 4 s
    assert pts[-1]["t"] == pytest.approx(4.0)


def test_timed_path_min_altitude_clamp_keeps_timing():
    timed = build_timed_path([[10.0, 0.0, 0.5, 0.0]], (0.0, 0.0, 1.5, 0.0),
                             1.0, 30.0, min_altitude_m=1.2)
    pts = timed["points"]
    for p in pts[1:]:
        assert p["z"] >= 1.2 - 1e-12
    assert pts[0]["z"] == pytest.approx(1.5)  # 交接点不钳制
    assert pts[-1]["t"] == pytest.approx(10.0 + 1.0 / 0.1)  # 时长语义保留（vz 下限 0.1）


def test_timed_path_noop_returns_none():
    assert build_timed_path([[0.0, 0.0, 1.5, 0.0]], (0.0, 0.0, 1.5, 0.0),
                            1.0, 30.0) is None


def test_multi_waypoint_exploit_chain():
    # D22 逐段分解：3 个节点位置（3 项 B：yaw 朝下一目标）
    timed = build_timed_path([[5.0, 0.0, 1.5], [10.0, 0.0, 1.5], [10.0, 5.0, 1.5]],
                             (0.0, 0.0, 1.5, 0.0), 1.0, 30.0)
    pts = timed["points"]
    assert pts[0]["t"] == 0.0
    assert pts[-1]["x"] == pytest.approx(10.0) and pts[-1]["y"] == pytest.approx(5.0)
    assert timed["duration"] > 0
    assert timed["steps"] >= 4
