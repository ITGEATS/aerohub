"""动作分解 + waypoint 序列 → FollowTimedPath 时间化分段（纯逻辑，无 ROS 依赖）。

语义来源（逐字移植对象）：upstream/utils/utils.py L195-277
calculate_movement_steps——"15° 步进偏航 → 5 m 水平步进 → 2 m 垂直步进"，
分解顺序即执行时间顺序（README §1 既定行为，勿改）。坐标约定换成 ENU
（上游 NED 的 yaw/水平几何公式 atan2(dy,dx) 在 ENU 数值同式；垂直分量
上游 z 下正 → 本模块 ENU z 上正，仅影响符号，分解逻辑不变）。
airsim.Pose 分支删除（输入统一为 [x,y,z(,yaw)] 列表，契约 §3 rclpy 原生
四元数由 node 换算为 yaw，无 wxyz 重排）。

时间映射（契约 §5 预声明锁定）：
  - 偏航段 t=Δyaw/base_yaw_rate；15° 子步合并为单段连续旋转，
    Σ子步时长 = rot_angle/yaw_rate，时长不变语义保留。
  - 水平段 t=d/base_velocity（5 m 步 → 5 s@1.0 m/s）。
  - 垂直段 vz=base_velocity·dz/dxy（SPF 同法，比例），下限保护 MIN_VZ；
    dxy≈0 时退化 vz=base_velocity。每 2 m 子步时长 |dz_step|/|vz|，
    垂直段总时长 = dxy/base_velocity（与水平等时，比例语义）。
输出为纯 dict（便于无 ROS 单测），node.py 转 uav_planner_interfaces/TimedPath。
"""
import math
from typing import List

import numpy as np

YAW_STEP_RAD = math.radians(15.0)  # 上游 yaw_ss=15°
MIN_VZ = 0.1                       # 垂直速率下限保护（契约 §5）


def _yaw_of_7(B) -> float:
    # [x,y,z,qx,qy,qz,qw] → ENU yaw（rclpy 原生四元数序，契约 §3 无 wxyz 重排）
    x, y, z, w = float(B[3]), float(B[4]), float(B[5]), float(B[6])
    return float(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def calculate_movement_steps(A, B, h_ss: float = 5.0, v_ss: float = 2.0,
                             yaw_ss: float = 15.0):
    """(len(path), path)，path 项 [x,y,z,p,r,y]。L195-277 逐字移植（ENU）。"""
    A = np.asarray(A, dtype=np.float64)
    B = np.asarray(B, dtype=np.float64)
    if len(A) == 6:
        A_pos, A_yaw = A[:3], float(A[5])
    elif len(A) == 4:  # ENU 简化位姿 [x,y,z,yaw]（本包 waypoint 格式）
        A_pos, A_yaw = A[:3], float(A[3])
    elif len(A) > 6:
        A_pos, A_yaw = A[:3], _yaw_of_7(A)
    else:
        raise ValueError(f"invalid orientation: {A}")
    B_pos = B[:3]
    if len(B) == 6:
        B_yaw = float(B[5])
    elif len(B) == 4:
        B_yaw = float(B[3])
    elif len(B) > 6:
        B_yaw = _yaw_of_7(B)
    elif len(B) == 3:
        # 上游 L223-226：B 无朝向时 yaw 取位移方向（compute_airsim_yaw=atan2(y,x)）
        d = B_pos[:2] - A_pos[:2]
        B_yaw = float(math.atan2(d[1], d[0]))
    else:
        raise ValueError(f"invalid orientation: {B}")

    delta_yaw = B_yaw - A_yaw
    abs_delta_yaw = np.abs(delta_yaw)
    rot_angle = abs_delta_yaw if abs_delta_yaw < np.pi else 2 * np.pi - abs_delta_yaw
    rot_reverse = 1 if abs_delta_yaw < np.pi else -1
    rot_direction = np.sign(delta_yaw) * rot_reverse

    delta_pos_h = B_pos[:2] - A_pos[:2]
    delta_pos_v = B_pos[-1] - A_pos[-1]

    h_dist = np.linalg.norm(delta_pos_h)
    v_dist = np.abs(delta_pos_v)
    h_ori = delta_pos_h / (h_dist + 1e-6)
    h_step_size = int(h_dist // h_ss)
    v_step_size = int(v_dist // v_ss)
    x_step, y_step = h_ori * h_ss
    x_dist, y_dist, z_dist = np.abs(delta_pos_h[0]), np.abs(delta_pos_h[1]), v_dist
    x_dire, y_dire, z_dire = np.sign(delta_pos_h[0]), np.sign(delta_pos_h[1]), np.sign(delta_pos_v)

    delta_yaw_inter = np.append(np.arange(0, rot_angle, np.deg2rad(yaw_ss)) * rot_direction,
                                rot_angle * rot_direction)

    delta_pos_x_inter = np.array([0 + x_step * (i + 1) for i in range(h_step_size)] + [x_dist]) * x_dire if h_dist > 0 else []
    delta_pos_y_inter = np.array([0 + y_step * (i + 1) for i in range(h_step_size)] + [y_dist]) * y_dire if h_dist > 0 else []
    delta_pos_z_inter = np.array([0 + v_ss * (i + 1) for i in range(v_step_size)] + [z_dist]) * z_dire if v_dist > 0 else []

    yaw_inter = A_yaw + delta_yaw_inter
    yaw_inter = (yaw_inter + np.pi) % (2 * np.pi) - np.pi
    pos_x_inter = A_pos[0] + delta_pos_x_inter
    pos_y_inter = A_pos[1] + delta_pos_y_inter
    pos_z_inter = A_pos[2] + delta_pos_z_inter

    path: List[list] = []
    for yaw_it in np.atleast_1d(yaw_inter):
        path.append([float(A_pos[0]), float(A_pos[1]), float(A_pos[2]), 0.0, 0.0, float(yaw_it)])

    for pos_x_it, pos_y_it in zip(np.atleast_1d(pos_x_inter), np.atleast_1d(pos_y_inter)):
        path.append([float(pos_x_it), float(pos_y_it), path[-1][2], path[-1][3], path[-1][4], path[-1][5]])

    for pos_z_it in np.atleast_1d(pos_z_inter):
        path.append([path[-1][0], path[-1][1], float(pos_z_it), path[-1][3], path[-1][4], path[-1][5]])

    return len(path), path


def _split_phases(path, A_pos) -> tuple:
    """把 path 按上游构造顺序拆为 (yaw 项, 水平项, 垂直项)。"""
    i = 0
    n = len(path)
    while i < n and np.allclose(path[i][:3], A_pos, atol=1e-12):
        i += 1
    j = i
    while j < n and abs(path[j][2] - A_pos[2]) < 1e-12:
        j += 1
    return path[:i], path[i:j], path[j:]


def build_timed_path(waypoints, start, base_velocity: float = 1.0,
                     base_yaw_rate: float = 30.0,
                     min_altitude_m=None,
                     h_ss: float = 5.0, v_ss: float = 2.0, yaw_ss: float = 15.0):
    """waypoint 序列 → 一条 FollowTimedPath 修订（dict，供 node 转 msg）。

    start: (x, y, z, yaw) ENU。waypoints: [B...]，B 为 [x,y,z,yaw]（6 项）或
    [x,y,z]（3 项，朝向下一段方向；最后一段保持入参 yaw）。
    返回 {"points": [{"t","x","y","z","vx","vy","vz","yaw"}...],
          "duration": float, "steps": int}；steps 为上游分解步数（预算/卡死
    判据消费）。全程零位移且零旋转 → None（不发送空轨迹）。
    """
    yaw_rate_rad = math.radians(base_yaw_rate)
    x, y, z, yaw = (float(v) for v in start)
    moved_any = False
    steps_total = 0
    points = [{"t": 0.0, "x": x, "y": y, "z": z, "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": yaw}]
    t = 0.0

    for b_idx, B in enumerate(waypoints):
        B = list(np.asarray(B, dtype=np.float64))
        if len(B) == 3 and b_idx + 1 < len(waypoints):
            nxt = np.asarray(waypoints[b_idx + 1], dtype=np.float64)[:3]
            d = nxt - np.asarray(B[:3], dtype=np.float64)
            B = B + [math.atan2(d[1], d[0])]  # 3 项语义：朝向下一目标
        n, path = calculate_movement_steps([x, y, z, 0.0, 0.0, yaw], B, h_ss, v_ss, yaw_ss)
        steps_total += n
        yaw_items, h_items, v_items = _split_phases(path, (x, y, z))

        # 偏航段：15° 子步合并为单段连续旋转，时长 = Σ子步 = rot_angle/yaw_rate
        if len(yaw_items) > 1:
            rot_total = sum(abs(yaw_items[i + 1][5] - yaw_items[i][5])
                            for i in range(len(yaw_items) - 1))
            if rot_total > 1e-9:
                t += rot_total / yaw_rate_rad
                yaw = yaw_items[-1][5]
                points.append({"t": t, "x": x, "y": y, "z": z,
                               "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": yaw})
                moved_any = True
        elif len(yaw_items) == 1:
            yaw = yaw_items[-1][5]

        # 水平段：t=d/base_velocity，逐 5 m 子步航点（保序，不丢中间点）
        move_start_xy = (x, y)
        prev = (x, y)
        for it in h_items:
            nx, ny = float(it[0]), float(it[1])
            d = math.hypot(nx - prev[0], ny - prev[1])
            if d < 1e-9:
                continue
            t += d / base_velocity
            ux, uy = (nx - prev[0]) / d, (ny - prev[1]) / d
            x, y = nx, ny
            points.append({"t": t, "x": x, "y": y, "z": z,
                           "vx": ux * base_velocity, "vy": uy * base_velocity,
                           "vz": 0.0, "yaw": yaw})
            moved_any = True
            prev = (x, y)

        # 垂直段：vz=v·dz/dxy 比例（SPF 同法，dxy 为本段水平位移），下限保护；
        # 每子步 |dz|/|vz|，垂直段总时长 = dxy/base_velocity（比例语义）
        if v_items:
            dz_total = float(v_items[-1][2]) - z
            dxy_total = math.hypot(x - move_start_xy[0], y - move_start_xy[1])
            if dxy_total > 1e-6:
                vz = base_velocity * dz_total / dxy_total
            else:
                vz = base_velocity if dz_total >= 0 else -base_velocity
            if abs(vz) < MIN_VZ:
                vz = MIN_VZ if vz >= 0 else -MIN_VZ
            prev_z = z
            for it in v_items:
                nz = float(it[2])
                dz = nz - prev_z
                if abs(dz) < 1e-9:
                    continue
                t += abs(dz) / abs(vz)
                z = nz
                prev_z = nz
                points.append({"t": t, "x": x, "y": y, "z": z,
                               "vx": 0.0, "vy": 0.0, "vz": vz, "yaw": yaw})
                moved_any = True
        # 本段结束朝向即分解 yaw 相位终值（=B_yaw 经 wrap），下一段以此为 A_yaw

    if not moved_any:
        return None

    if min_altitude_m is not None:
        for p in points:
            if p["t"] > 0.01 and p["z"] < min_altitude_m:
                # t=0 交接点保持真实位置（behavior 连续性校验），永不钳制；
                # 钳制只改高度不改时序（SPF D12 同法）
                p["z"] = float(min_altitude_m)
                p["vz"] = 0.0

    return {"points": points, "duration": t, "steps": steps_total}
