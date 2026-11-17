"""五目批次 → odom ENU 语义点云 → 收缩 waypoint（纯逻辑，无 ROS 依赖）。

语义来源（契约 README §5 / result-20261117.md §1-§3）：
  - 深度反投影：upstream/utils/maps.py L258-286 build_local_point_cloud +
    L289-318 build_global_point_cloud + L321-339 convert_global_pc。上游链是
    AirSim NED + camera→ego FRD 换轴（(Z,-X,-Y)）+ R(quat)·R_x(180)；本模块按
    契约重写为 ROS optical（右、下、前）→ FLU（静态旋转 T_base_optical，D6
    仅取旋转、不含杆臂平移）→ ENU（self_localization/pose）。
  - 无效值过滤：isfinite & 0 < d < 50 m（上游 convert_global_pc L331-338 的
    (depth>0)&(depth<50) 门，米制化，D7b：不再复现 /300×100=1/3 尺度）。
  - 统计滤波：upstream/utils/maps.py L53-67 statistical_filter（sklearn
    NearestNeighbors 版）。本实现用 scipy cKDTree 等义重写：k 近邻含自身
    （kneighbors 首列距离 0）、逐点 mean(dist over k)、全局 mean/std、
    阈值 mean + std_dev_multiplier·std。cKDTree 与 sklearn 在低维精确 kNN
    上结果一致，语义等价；禁 sklearn 是容器约束（任务规格）。
  - waypoint：SimRun.py L292-306——类均值 → alpha=0.6 向当前位置收缩 →
    高度下限（上游 sam 支 z>-5→z=5 数据约定 → AS2 参数化 min_altitude_m，
    契约 §5）。
"""
import math
from typing import List, Optional

import numpy as np
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation

MAX_DEPTH_M = 50.0  # 契约 §1 锁定：planner 侧 0<d<50 m 过滤


def rotation_body_from_optical(yaw_deg: float) -> np.ndarray:
    """base_link(FLU)←camera_optical 纯旋转（D6：仅取旋转，不含杆臂平移）。

    optical 轴（右、下、前）在 FLU 中的方向：x_opt=(sinψ,-cosψ,0)、
    y_opt=(0,0,-1)、z_opt=(cosψ,sinψ,0)，ψ 为相机相对机头的 ROS 逆时针 yaw。
    ψ=0 时即 ROS 标准 FLU→optical（前=x、右=-y、下=-z）。
    """
    psi = math.radians(float(yaw_deg))
    s, c = math.sin(psi), math.cos(psi)
    return np.array([[s, 0.0, c], [-c, 0.0, s], [0.0, -1.0, 0.0]])


def backproject_view(depth: np.ndarray, K: np.ndarray, mask: np.ndarray,
                     R_body_from_optical: np.ndarray,
                     pose: np.ndarray,
                     max_depth_m: float = MAX_DEPTH_M) -> np.ndarray:
    """单视角 mask 像素反投影到 odom ENU。

    depth: float32 HxW，米，沿光轴 axial（ImageType 1，32FC1）
    K:     3x3 内参（CameraInfo 实测，D7；方形图像契约）
    mask:  bool/uint8 HxW，grounding 服务返回单通道 PNG 解码
    R_body_from_optical: 3x3，base_link(FLU)←camera_optical 静态旋转（D6，
                         仅取旋转；五目各视角不同，参数传入）
    pose:  [x, y, z, qx, qy, qz, qw]，该视角图像 stamp 匹配的 odom ENU 位姿

    返回 (N,3) ENU 点。p_optical=((u-cx)d/fx,(v-cy)d/fy,d)；
    p_body=R·p_optical；p_world=R_pose·p_body+pos。
    """
    depth = np.asarray(depth, dtype=np.float64)
    mask = np.asarray(mask)
    if mask.dtype != np.bool_:
        mask = mask > 0
    h, w = depth.shape
    if K.shape != (3, 3) or R_body_from_optical.shape != (3, 3):
        raise ValueError("K/R must be 3x3")
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    if fx <= 0 or fy <= 0:
        raise ValueError("non-positive focal length")

    u, v = np.meshgrid(np.arange(w, dtype=np.float64),
                       np.arange(h, dtype=np.float64))
    valid = np.isfinite(depth) & (depth > 0.0) & (depth < max_depth_m) & mask
    idx = np.where(valid)
    if len(idx[0]) == 0:
        return np.zeros((0, 3), dtype=np.float64)

    d = depth[idx]
    # ROS optical：x 右、y 下、z 前（maps.py L274-284 的 (Z,-X,-Y) 换轴在此
    # 以 optical 原生坐标表达，等价性由 tests 非恒等样例证明）
    p_optical = np.stack(((u[idx] - cx) * d / fx,
                          (v[idx] - cy) * d / fy,
                          d), axis=-1)
    p_body = p_optical @ np.asarray(R_body_from_optical, dtype=np.float64).T

    pos = np.asarray(pose[:3], dtype=np.float64)
    quat = np.asarray(pose[3:], dtype=np.float64)  # rclpy 原生 [x,y,z,w]，无 wxyz 重排
    p_world = Rotation.from_quat(quat).apply(p_body) + pos
    return p_world


def statistical_filter(point_cloud: np.ndarray, k: int = 30,
                       std_dev_multiplier: float = 1.0):
    """cKDTree 等义重写（maps.py L53-67 sklearn 版语义，见模块头）。

    返回 (filtered_points, filter_idx)。调用方保证 len(point_cloud) >= k
    （上游 SimRun L277 `if len(...) > 30` 才滤波，L279 恒真守卫保留为不过滤分支）。
    """
    point_cloud = np.asarray(point_cloud, dtype=np.float64)
    n = len(point_cloud)
    if n == 0:
        return point_cloud, np.zeros(0, dtype=np.int64)
    k = int(min(k, n))  # sklearn n_neighbors 不得超过样本数；上游守卫等价
    tree = cKDTree(point_cloud)
    distances, _ = tree.query(point_cloud, k=k)  # 含自身（首列 0），同 kneighbors
    mean_distances = np.mean(distances, axis=1)
    global_mean = np.mean(mean_distances)
    global_std = np.std(mean_distances)
    filter_idx = np.where(mean_distances < global_mean + std_dev_multiplier * global_std)[0]
    return point_cloud[filter_idx], filter_idx


def semantic_waypoint(semantic_pc: List[np.ndarray], cur_pos: np.ndarray,
                      alpha: float = 0.6,
                      min_altitude_m: Optional[float] = None):
    """SimRun.py L292-306 语义：类均值 → alpha 收缩 → 高度下限。

    semantic_pc 为逐视角成功分割的点云列表（上游只取第一个非空类，L282-283
    `semantic_pc = semantic_pc[0]`，逐字保留）。
    返回 (route_coords(3,), seg_succ_all(bool))；seg_succ_all=False 时
    route_coords=cur_pos（上游 L292-293 原地不动语义）。
    """
    cur_pos = np.asarray(cur_pos, dtype=np.float64)
    if not semantic_pc:
        return cur_pos.copy(), False
    pc = semantic_pc[0]  # 上游取首个非空类（勿"改进"为全类均值）
    route_coords = np.mean(pc, axis=0)
    if not np.any(route_coords):        # 全零退化 → 原地（上游 L296-297）
        route_coords = cur_pos.copy()
    route_coords = alpha * route_coords + (1 - alpha) * cur_pos  # alpha=0.6 收缩
    if np.any(np.isnan(route_coords)):
        route_coords = cur_pos.copy()
    if min_altitude_m is not None and route_coords[2] < min_altitude_m:
        # 上游 `if route_coords[2] > -5: route_coords[2] = 5`（NED 数据约定）
        # → ENU 参数化下限（契约 §5：min_altitude_m 替代）
        route_coords[2] = float(min_altitude_m)
    return route_coords, True


def yaw_to_point(cur_pos: np.ndarray, target: np.ndarray) -> float:
    """SimRun.py L309-310：yaw = atan2(dy, dx)（ENU 逆时针，与 NED 数值同式）。"""
    rel = np.asarray(target, dtype=np.float64) - np.asarray(cur_pos, dtype=np.float64)
    return float(np.arctan2(rel[1], rel[0]))
