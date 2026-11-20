"""Pure AeroVLA policy math. Zero ROS / zero transformers dependencies.

Fidelity sources (upstream AeroVLA @ b87aaa7, Apache-2.0), cited line by
line below against the raw fetches kept in ``log/aerovla/``:

- ``upstream-src/src_model_wrapper_aerovla_wrapper_ui.py``
    L46-76   get_semantic_direction (fuzzy hint, 7 bands)
    L194-197 landing decision ("LAND" / "<LAND>" or near-zero action)
    L208-223 _parse_action_from_text (bin dequantization, NUM_BINS=99)
- ``upstream-src/airsim_plugin_AirVLNSimulatorClientTool_AeroVLA.py``
    L301-395 move_path (action -> timed execution; CRUISE_SPEED=1.0,
    YAW_THRESHOLD=0.25 large-turn branch drops the horizontal displacement,
    0.5 s hover tail at L386)

Numeric formulas, branch conditions and string literals are kept verbatim;
the only declared deviations are the timed-trajectory durations for the
in-place rotation (upstream ``rotateToYawAsync`` L356 is a timeout-bounded
async call with no fixed duration) and are flagged with DEVIATION comments.
"""

import math
import re

import numpy as np
from scipy.spatial.transform import Rotation as R

# aerovla_wrapper_ui.py L25: self.NUM_BINS = 99
NUM_BINS = 99
# aerovla_wrapper_ui.py L26-30: self.norm_stats
NORM_STATS = {
    'forward': {'min': 0.0, 'max': 5.0},
    'down':    {'min': -5.0, 'max': 5.0},
    'yaw':     {'min': -1.1, 'max': 1.1},
}

# AirVLNSimulatorClientTool_AeroVLA.py L342-346
EPSILON = 0.01
YAW_THRESHOLD = 0.25
CRUISE_SPEED = 1.0
MICRO_MOVE_THRESHOLD = 1.0
# L386: trailing moveByVelocityAsync(0, 0, 0, duration=0.5) hover
HOVER_DURATION = 0.5
# L384: moveToZAsync(target_z, velocity=2.0)
VERTICAL_SPEED = 2.0
# DEVIATION (documented): upstream L356 rotateToYawAsync(target_yaw_deg,
# timeout_sec=3, margin=5) has no fixed duration; a timed trajectory needs
# one for a rotation-only step, so a nominal 1.0 s is used.
ROTATE_DURATION = 1.0


def dequantize_bin(bin_val, axis):
    """Single-bin dequantization, verbatim from aerovla_wrapper_ui.py L217-219.

    ``dequantize(bin_val, axis)``: clamp to [0, NUM_BINS-1], then
    ``bin/(NUM_BINS-1) * (vmax - vmin) + vmin``.
    """
    vmin, vmax = NORM_STATS[axis]['min'], NORM_STATS[axis]['max']
    return (max(0, min(NUM_BINS - 1, bin_val)) / (NUM_BINS - 1)) * (vmax - vmin) + vmin


def dequantize_bins(text):
    """Parse the model text into (fwd, down, yaw).

    Verbatim port of ``_parse_action_from_text`` (aerovla_wrapper_ui.py
    L208-223): split on the last ``"Action:"``, ``re.findall(r"\\d+")`` and
    take the LAST 3 numbers; fewer than 3 numbers yields (0.0, 0.0, 0.0)
    (upstream L212 initializes the triple and the ``if`` at L214 leaves it
    untouched).
    """
    output_part = text.split("Action:")[-1]
    matches = re.findall(r"\d+", output_part)
    fwd, down, yaw = 0.0, 0.0, 0.0

    if len(matches) >= 3:
        try:
            bin_fwd, bin_down, bin_yaw = map(int, matches[-3:])
            fwd = dequantize_bin(bin_fwd, 'forward')
            down = dequantize_bin(bin_down, 'down')
            yaw = dequantize_bin(bin_yaw, 'yaw')
        except Exception:
            # Upstream L221 uses a bare ``except: pass``; keep the swallow.
            pass
    return fwd, down, yaw


def is_landing(text, fwd, down, yaw):
    """Landing decision, verbatim from aerovla_wrapper_ui.py L194-196.

    ``"LAND" in text or "<LAND>" in text`` (string branch) OR
    ``fwd < 0.01 and abs(down) < 0.01 and abs(yaw) < 0.01`` (numeric
    branch; note fwd is compared WITHOUT abs() upstream -- fwd dequantizes
    to [0, 5] so this is equivalent, but the literal is kept).
    """
    has_land_str = "LAND" in text or "<LAND>" in text
    has_land_num = (fwd < 0.01) and (abs(down) < 0.01) and (abs(yaw) < 0.01)
    return has_land_str or has_land_num


def _quat_from(orientation):
    """Accept dict or sequence quaternion, verbatim from L50-53 / L319-322."""
    if isinstance(orientation, dict):
        return [orientation.get('x', 0), orientation.get('y', 0),
                orientation.get('z', 0), orientation.get('w', 1)]
    return orientation


def yaw_from_quat(orientation):
    """Body yaw, verbatim from move_path L324-325:
    ``R.from_quat(quat).as_euler('zyx')[0]``."""
    return float(R.from_quat(_quat_from(orientation)).as_euler('zyx')[0])


def semantic_hint(position, orientation, target_pos):
    """Fuzzy direction hint, verbatim port of ``get_semantic_direction``
    (aerovla_wrapper_ui.py L46-76).

    7 bands over the body-frame azimuth of the GT target (degrees):
    [-15,15] "straight ahead ", (15,60] "forward-right ", (60,120]
    "to your right ", (120,180] "to your right rear ", [-60,-15)
    "forward-left ", [-120,-60) "to your left ", [-180,-120)
    "to your left rear "; each literal carries a TRAILING SPACE (it is
    concatenated as ``f"Fly {hint}and ..."``). Horizontal distance < 0.01 m
    returns "" (L65-66) -- the oracle target coordinates are a declared
    runtime input (see README capability-card note).
    """
    pos = np.array(position)
    quat = _quat_from(orientation)

    vec_world = np.array(target_pos) - pos

    dist_xy = np.linalg.norm(vec_world[:2])

    r = R.from_quat(quat)
    vec_body = r.inv().apply(vec_world)

    x, y = vec_body[0], vec_body[1]
    angle_deg = np.degrees(np.arctan2(y, x))

    if dist_xy < 0.01:
        return ""
    else:
        if -15 <= angle_deg <= 15: return "straight ahead "
        if 15 < angle_deg <= 60: return "forward-right "
        if 60 < angle_deg <= 120: return "to your right "
        if 120 < angle_deg <= 180: return "to your right rear "
        if -60 <= angle_deg < -15: return "forward-left "
        if -120 <= angle_deg < -60: return "to your left "
        if -180 <= angle_deg < -120: return "to your left rear "

    return ""


def action_to_setpoints(state, fwd, down, yaw):
    """One AeroVLA action -> a timed point list (constant-velocity straight
    segment + yaw, plus the upstream 0.5 s hover tail).

    Verbatim branch port of ``move_path``
    (airsim_plugin_AirVLNSimulatorClientTool_AeroVLA.py L301-395):

    - L327-332: target_yaw = curr_yaw + yaw; world displacement
      dx = fwd*cos(target_yaw), dy = fwd*sin(target_yaw), dz = down.
    - L348-352: the step is executed only if horizontal/vertical/yaw
      displacement exceeds EPSILON.
    - L355-356: LARGE-TURN / in-place branch -- when |yaw| >=
      YAW_THRESHOLD (0.25 rad) or total_dist <= EPSILON the vehicle first
      rotates to the target yaw.
    - L358-381: small-yaw branch (|yaw| < 0.25) -- straight constant-
      velocity move: total_dist < MICRO_MOVE_THRESHOLD (1.0 m) uses
      duration 1.0 s with v = d/1.0; otherwise unit vector * CRUISE_SPEED
      (1.0 m/s) with duration = total_dist / CRUISE_SPEED; absolute target
      yaw is held via yaw_mode.
    - L383-384: large-turn branch -- NO horizontal displacement is added
      (the dx/dy computed above are dropped); only the vertical component
      runs via moveToZAsync at velocity 2.0. This is the branch the
      migration plan risk list calls out ("大转向步丢弃水平位移"); it is
      kept as-is, not "fixed".
    - L386: every step ends with a 0.5 s zero-velocity hover.

    ``state`` mirrors upstream ``curr_state``: a dict with 'position'
    (x, y, z) and 'orientation' (quaternion [x,y,z,w] or dict).
    Returns a list of dicts ``{"t": time_from_start_s, "position": (x,y,z),
    "velocity": (vx,vy,vz), "yaw": abs_yaw_rad}`` -- the first point is the
    t=0 handoff at the current pose (behavior continuity), matching the
    FollowTimedPath point convention used by the spf/yopo adapters.
    """
    curr_pos = np.array(state['position'], dtype=float)
    quat = _quat_from(state['orientation'])

    # L324-325: curr_yaw_scalar = R.from_quat(quat).as_euler('zyx')[0]
    curr_yaw = float(R.from_quat(quat).as_euler('zyx')[0])

    # L327-332
    target_yaw = curr_yaw + yaw
    dx = fwd * math.cos(target_yaw)
    dy = fwd * math.sin(target_yaw)
    dz = down

    x0, y0, z0 = (float(curr_pos[0]), float(curr_pos[1]), float(curr_pos[2]))

    # L338-340
    horizontal_dist = math.sqrt(dx ** 2 + dy ** 2)
    vertical_dist = abs(dz)
    total_dist = math.sqrt(horizontal_dist ** 2 + vertical_dist ** 2)

    # L348-350
    need_horizontal_move = horizontal_dist > EPSILON
    need_vertical_move = vertical_dist > EPSILON
    need_yaw_adjust = abs(yaw) > EPSILON

    points = [{"t": 0.0, "position": (x0, y0, z0),
               "velocity": (0.0, 0.0, 0.0), "yaw": curr_yaw}]

    if need_horizontal_move or need_vertical_move or need_yaw_adjust:
        # L355: rotate-first condition (|yaw| >= YAW_THRESHOLD or in-place).
        if abs(yaw) >= YAW_THRESHOLD or total_dist <= EPSILON:
            if vertical_dist > EPSILON:
                # L383-384: large-turn branch moves ONLY vertically at
                # velocity 2.0; the horizontal dx/dy are dropped verbatim.
                t_move = vertical_dist / VERTICAL_SPEED
                end = (x0, y0, z0 + dz)
                vel = (0.0, 0.0, VERTICAL_SPEED if dz > 0 else -VERTICAL_SPEED)
            else:
                # Rotation-only step (in-place or large turn without
                # vertical). DEVIATION: nominal ROTATE_DURATION replaces the
                # duration-less rotateToYawAsync (L356).
                t_move = ROTATE_DURATION
                end = (x0, y0, z0)
                vel = (0.0, 0.0, 0.0)
        else:
            # L358-381: small-yaw straight constant-velocity move with the
            # absolute target yaw (upstream yaw_mode is_rate=False).
            if total_dist < MICRO_MOVE_THRESHOLD:
                # L362-366: duration = 1.0; v = d / duration
                t_move = 1.0
                vel = (dx / t_move, dy / t_move, dz / t_move)
            else:
                # L368-374: unit vector * CRUISE_SPEED; duration = total/CRUISE
                t_move = total_dist / CRUISE_SPEED
                vel = ((dx / total_dist) * CRUISE_SPEED,
                       (dy / total_dist) * CRUISE_SPEED,
                       (dz / total_dist) * CRUISE_SPEED)
            end = (x0 + dx, y0 + dy, z0 + dz)
        points.append({"t": t_move, "position": end, "velocity": vel,
                       "yaw": target_yaw})
        # L386: trailing 0.5 s hover at the step end pose.
        points.append({"t": t_move + HOVER_DURATION, "position": end,
                       "velocity": (0.0, 0.0, 0.0), "yaw": target_yaw})
    else:
        # Nothing needed: upstream skips the move block and only hovers.
        points.append({"t": HOVER_DURATION, "position": (x0, y0, z0),
                       "velocity": (0.0, 0.0, 0.0), "yaw": curr_yaw})

    return points
