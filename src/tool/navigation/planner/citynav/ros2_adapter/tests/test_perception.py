"""perception 单测：非恒等位姿/旋转的深度反投影数值对照（契约 §3 要求：
手算 ENU 结果对照，证明 optical→FLU→ENU 重写与上游 NED 链等价）、
无效值过滤、cKDTree 统计滤波已知小样本、alpha 收缩与高度下限。"""
import math

import numpy as np
import pytest

from uav_citynav_planner.perception import (backproject_view,
                                            rotation_body_from_optical,
                                            semantic_waypoint,
                                            statistical_filter)

K8 = np.array([[4.0, 0.0, 4.0], [0.0, 4.0, 4.0], [0.0, 0.0, 1.0]])  # 8x8, fx=fy=4


def _depth_at(shape, coords, value):
    d = np.zeros(shape, dtype=np.float32)
    for (v, u) in coords:
        d[v, u] = value
    return d


def test_front_camera_center_pixel_identity_pose():
    # 前视相机 ψ=0，像素中心 (u,v)=(4,4)，d=2 → optical (0,0,2)
    # FLU：R0·(0,0,2) = (2,0,0)（前）；恒等位姿 → ENU (2,0,0)
    depth = _depth_at((8, 8), [(4, 4)], 2.0)
    mask = np.zeros((8, 8), bool)
    mask[4, 4] = True
    pts = backproject_view(depth, K8, mask, rotation_body_from_optical(0.0),
                           np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
    assert len(pts) == 1
    assert pts[0] == pytest.approx([2.0, 0.0, 0.0], abs=1e-9)


def test_front_camera_offcenter_pixel_hand_computed():
    # 像素 (u=5,v=4)（中心右 1 px），d=2：p_optical=((5-4)*2/4, 0, 2)=(0.5,0,2)
    # ψ=0 FLU：x_opt=(0,-1,0) → 0.5·(0,-1,0)+(0,0,2)·z_opt(1,0,0) = (2,-0.5,0)
    # 位姿平移 (10,5,3) 恒等旋转 → ENU (12,4.5,3)
    depth = _depth_at((8, 8), [(4, 5)], 2.0)
    mask = np.zeros((8, 8), bool)
    mask[4, 5] = True
    pts = backproject_view(depth, K8, mask, rotation_body_from_optical(0.0),
                           np.array([10.0, 5.0, 3.0, 0.0, 0.0, 0.0, 1.0]))
    assert pts[0] == pytest.approx([12.0, 4.5, 3.0], abs=1e-9)


def test_left_camera_nonidentity_pose_and_rotation():
    # 左视相机 ψ=+90（optical 前=body+y，optical 右=body+x）
    # 像素 (v=3,u=4)（中心上 1 px → optical y=-0.5·? 上=v<cy → y_opt 负=上）
    # d=2：p_optical=(0,(3-4)*2/4,2)=(0,-0.5,2)
    # R(+90)：x_opt=(1,0,0),y_opt=(0,0,-1),z_opt=(0,1,0)
    #   p_body=(0·1 + (-0.5)·0 + 2·0, 0+0+2·1, 0+(-0.5)·(-1)+0)=(0,2,0.5)
    # 机体 yaw=90°（ENU 逆时针，四元数 z=sin45,w=cos45）：Rz(90)·(0,2,0.5)=(-2,0,0.5)
    # 平移 (10,5,3) → ENU (8,5,3.5)
    depth = _depth_at((8, 8), [(3, 4)], 2.0)
    mask = np.zeros((8, 8), bool)
    mask[3, 4] = True
    s = math.sin(math.radians(45.0))
    pts = backproject_view(depth, K8, mask, rotation_body_from_optical(90.0),
                           np.array([10.0, 5.0, 3.0, 0.0, 0.0, s, s]))
    assert pts[0] == pytest.approx([8.0, 5.0, 3.5], abs=1e-9)


def test_pitched_pose_flips_vertical_axis():
    # 机体绕 ENU y 轴 +90°（右手定则，机头朝下）时，前视光轴深度点应向下：
    # 四元数 [0, sin45, 0, cos45]
    depth = _depth_at((8, 8), [(4, 4)], 2.0)
    mask = np.zeros((8, 8), bool)
    mask[4, 4] = True
    s = math.sin(math.radians(45.0))
    c = math.cos(math.radians(45.0))
    pts = backproject_view(depth, K8, mask, rotation_body_from_optical(0.0),
                           np.array([0.0, 0.0, 10.0, 0.0, s, 0.0, c]))
    # p_body=(2,0,0)；Ry(90)·(2,0,0) = (0,0,-2)（ENU：x→-z）
    assert pts[0] == pytest.approx([0.0, 0.0, 8.0], abs=1e-9)


def test_depth_invalid_filter():
    # isfinite & 0<d<50 过滤：0、负、50 及以上、nan、inf 全部剔除
    depth = np.zeros((8, 8), np.float32)
    vals = [0.0, -1.0, 50.0, 60.0, np.nan, np.inf, 10.0, 49.9]
    coords = [(0, 0), (0, 1), (0, 2), (0, 3), (1, 0), (1, 1), (1, 2), (1, 3)]
    for (v, u), val in zip(coords, vals):
        depth[v, u] = val
    mask = np.zeros((8, 8), bool)
    for (v, u) in coords:
        mask[v, u] = True
    pts = backproject_view(depth, K8, mask, rotation_body_from_optical(0.0),
                           np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0]))
    assert len(pts) == 2  # 仅 10.0 与 49.9 有效
    assert np.all(np.isfinite(pts))


def test_statistical_filter_known_small_sample():
    # 5 点紧簇 + 1 远离群（k=3 含自身）：离群 mean=100，簇内 ≈0.0667；
    # 全局 mean≈16.79，std≈33.3，阈值≈50.1 → 仅离群被剔除
    pts = np.array([[0, 0, 0], [0.1, 0, 0], [-0.1, 0, 0],
                    [0, 0.1, 0], [0, -0.1, 0], [100, 0, 0]], dtype=np.float64)
    filtered, idx = statistical_filter(pts, k=3, std_dev_multiplier=1.0)
    assert sorted(idx.tolist()) == [0, 1, 2, 3, 4]
    assert len(filtered) == 5
    assert not np.any(np.all(filtered == [100, 0, 0], axis=1))


def test_statistical_filter_matches_manual_mean_distances():
    # 手算对照：4 点方格，k=2（含自身距离 0 → mean=最近邻距离/2）
    pts = np.array([[0, 0, 0], [1, 0, 0], [0, 1, 0], [10, 0, 0]], float)
    tree_mean = np.array([0.5, 0.5, 0.5, 4.5])
    gmean, gstd = tree_mean.mean(), tree_mean.std()
    keep = np.where(tree_mean < gmean + 1.0 * gstd)[0]
    _, idx = statistical_filter(pts, k=2, std_dev_multiplier=1.0)
    assert sorted(idx.tolist()) == sorted(keep.tolist())
    assert 3 not in idx.tolist()


def test_semantic_waypoint_alpha_shrink_and_floor():
    pc = [np.array([[10.0, 0.0, 5.0], [10.0, 0.0, 3.0]])]  # 类均值 (10,0,4)
    cur = np.array([0.0, 0.0, 2.0])
    wp, ok = semantic_waypoint(pc, cur, alpha=0.6, min_altitude_m=1.2)
    assert ok
    # 0.6*(10,0,4)+0.4*(0,0,2) = (6,0,3.2)；z=3.2 ≥ 1.2 不钳制
    assert wp == pytest.approx([6.0, 0.0, 3.2], abs=1e-9)
    # 低目标：均值 z=0 → 收缩后 0.8 < 1.2 → 钳制（min_altitude_m 替代上游 z 规则）
    pc = [np.array([[10.0, 0.0, -2.0]])]
    wp, _ = semantic_waypoint(pc, cur, alpha=0.6, min_altitude_m=1.2)
    assert wp[2] == pytest.approx(1.2)


def test_semantic_waypoint_first_class_only_and_degenerate():
    # 上游 L282-283：只取第一个非空类，不是全类均值
    pcs = [np.array([[2.0, 0.0, 2.0]]), np.array([[100.0, 0.0, 2.0]])]
    wp, ok = semantic_waypoint(pcs, np.array([0.0, 0.0, 2.0]), alpha=0.6)
    assert ok and wp[0] == pytest.approx(1.2)
    # 无分割成功 → 原地
    wp, ok = semantic_waypoint([], np.array([1.0, 2.0, 3.0]))
    assert not ok and wp == pytest.approx([1.0, 2.0, 3.0])
