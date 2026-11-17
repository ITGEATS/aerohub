"""ActionPoint → 时间化轨迹修订（迁移方案阶段 4 核心映射）。

语义来源：上游 src/spf/airsim/drone_space.py action_to_commands L55-143：
  "若目标方位角 |θ|>10° 先以 base_yaw_rate·mult 原地偏航对准，
   再以 base_velocity·mult 恒速前飞 T = max(d_xy/v, min_command_duration)，
   垂直分量按 dz/d_xy 比例（NED 下正；本模块输出 ENU 上正）。"
θ = atan2(dx右, dy前)，与上游 target_angle 一致；yaw 采用 ENU 逆时针约定，
机体系"右/前"分解到世界系仅依赖当前 yaw，与上游 rotate→forward 执行等价。

输出为纯 dict（便于无 ROS 单测），node.py 负责转 uav_planner_interfaces/TimedPath。
"""
import math
from typing import List, Optional

from .projector import ActionPoint

YAW_THRESHOLD_DEG = 10.0  # 上游 L104：|θ|≤10° 不旋转


def build_revision(action: ActionPoint, x0: float, y0: float, z0: float, yaw0: float,
                   base_velocity: float, base_yaw_rate: float,
                   min_command_duration: float) -> Optional[dict]:
    """一个 SPF 循环 → 一次 TimedPath 修订（2~3 点）。

    返回 None：位移过小（上游 total_distance<0.01 返回空命令）。
    返回 dict：{"points": [...], "duration": 末点时刻, "rotated": bool, "theta_deg": ...}
    points 每项 {"t","x","y","z","vx","vy","vz","yaw"}，ENU 坐标、m/s、rad。
    """
    mult = action.velocity_multiplier
    vel_scale = base_velocity * mult
    yaw_rate = base_yaw_rate * mult

    total = math.sqrt(action.dx ** 2 + action.dy ** 2 + action.dz ** 2)
    if total < 0.01:
        return None

    d_xy = math.hypot(action.dx, action.dy)
    points: List[dict] = []

    # The behavior's timeline requires the first point at t==0 and >=2 points.
    # t=0 is the handoff: current pose, zero velocity, current yaw. The rotate /
    # forward waypoints follow at their absolute offsets from this epoch.
    points.append({"t": 0.0, "x": x0, "y": y0, "z": z0,
                   "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": yaw0})
    t = 0.0
    yaw1 = yaw0

    rotated = False
    if d_xy > 0.01:
        theta = math.degrees(math.atan2(action.dx, action.dy))
        if abs(theta) > YAW_THRESHOLD_DEG:
            rotated = True
            t_rot = abs(theta) / yaw_rate
            yaw1 = yaw0 + math.radians(theta)
            points.append({"t": t_rot, "x": x0, "y": y0, "z": z0,
                           "vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw": yaw1})
            t = t_rot

    if d_xy > 0.01:
        vz_up = (action.dz / d_xy) * vel_scale  # ENU 上正（上游 NED vz=-dz/d_xy·v）
        T = max(d_xy / vel_scale, min_command_duration)
    else:
        vz_up = 0.0
        T = min_command_duration

    travel = vel_scale * T
    end_x = x0 + math.cos(yaw1) * travel
    end_y = y0 + math.sin(yaw1) * travel
    end_z = z0 + vz_up * T
    points.append({"t": t + T, "x": end_x, "y": end_y, "z": end_z,
                   "vx": vel_scale * math.cos(yaw1),
                   "vy": vel_scale * math.sin(yaw1),
                   "vz": vz_up, "yaw": yaw1})

    return {"points": points, "duration": t + T, "rotated": rotated,
            "theta_deg": math.degrees(math.atan2(action.dx, action.dy)) if d_xy > 0.01 else 0.0,
            "vz_up": vz_up, "T": T}
