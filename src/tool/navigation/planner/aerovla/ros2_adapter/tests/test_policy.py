"""Host tests for the ROS-free AeroVLA policy math.

Run: python -m pytest tests -q  (no ROS, no transformers, no GPU).
Expected numeric values are hand-computed from the upstream formulas and
written literally here so a regression in the port is caught, not merely
"consistent with itself".
"""

import math

import numpy as np
import pytest

from uav_aerovla_planner import policy

# Upstream NUM_BINS-1 = 98; norm ranges forward[0,5] down[-5,5] yaw[-1.1,1.1].
# Hand-computed dequantizations (formula: clamp(b,0,98)/98*(vmax-vmin)+vmin):
#   44 -> 44/98*5            = 2.2448979591836734
#   49 -> 49/98*10 - 5       = 0.0
#   94 -> 94/98*2.2 - 1.1    = 1.010204081632653
BIN_44_FWD = 44 / 98 * 5.0
BIN_49_DOWN = 0.0
BIN_94_YAW = 94 / 98 * 2.2 - 1.1


def test_dequantize_example_matches_upstream_formula():
    fwd, down, yaw = policy.dequantize_bins("44\u258149\u258194")
    assert fwd == pytest.approx(BIN_44_FWD, abs=1e-12)
    assert down == pytest.approx(BIN_49_DOWN, abs=1e-12)
    assert yaw == pytest.approx(BIN_94_YAW, abs=1e-12)


def test_dequantize_ignores_leading_prompt_text():
    # Only the digits AFTER the last "Action:" are used (wrapper L210).
    text = "<image>\nFly straight ahead and find the target. foo\nAction: 44\u258149\u258194"
    fwd, down, yaw = policy.dequantize_bins(text)
    assert fwd == pytest.approx(BIN_44_FWD, abs=1e-12)
    assert yaw == pytest.approx(BIN_94_YAW, abs=1e-12)


def test_dequantize_takes_last_three_when_more_present():
    # matches[-3:] (wrapper L216): stray earlier digits must be ignored.
    fwd, down, yaw = policy.dequantize_bins("Action: 7 3 44\u258149\u258194")
    assert fwd == pytest.approx(BIN_44_FWD, abs=1e-12)
    assert down == pytest.approx(BIN_49_DOWN, abs=1e-12)
    assert yaw == pytest.approx(BIN_94_YAW, abs=1e-12)


def test_dequantize_boundary_bins():
    # bin 0 -> axis min; bin 98 (NUM_BINS-1) -> axis max.
    assert policy.dequantize_bins("0\u25810\u25810") == (0.0, -5.0, -1.1)
    assert policy.dequantize_bins("98\u258198\u258198") == (5.0, 5.0, 1.1)


def test_dequantize_clamps_out_of_range_bins():
    # clamp to [0, 98] (wrapper L219): bin 99 and 200 saturate at max,
    # and since re.findall(r"\d+") never yields a sign, a literal "-" is
    # simply not part of a match -- verify the clamp path directly.
    assert policy.dequantize_bins("99\u258199\u258199") == (5.0, 5.0, 1.1)
    assert policy.dequantize_bins("200\u2581200\u2581200") == (5.0, 5.0, 1.1)
    assert policy.dequantize_bin(-5, 'forward') == 0.0
    assert policy.dequantize_bin(-5, 'down') == -5.0
    assert policy.dequantize_bin(150, 'yaw') == 1.1


def test_dequantize_fewer_than_three_numbers_is_zero_action():
    # wrapper L212/L214: the triple defaults to (0,0,0) if <3 matches.
    assert policy.dequantize_bins("Action: 44\u258149") == (0.0, 0.0, 0.0)
    assert policy.dequantize_bins("no digits here") == (0.0, 0.0, 0.0)


def test_is_landing_string_branch():
    assert policy.is_landing("... 44\u258149\u258194 LAND", 2.24, 0.0, 1.0) is True
    assert policy.is_landing("<LAND> whatever", 2.24, 0.0, 1.0) is True


def test_is_landing_numeric_branch():
    # fwd<0.01 and abs(down)<0.01 and abs(yaw)<0.01 (wrapper L195).
    assert policy.is_landing("0\u25810\u25810", 0.0, -5.0, -1.1) is False  # down/yaw big
    assert policy.is_landing("49\u258149\u258149", 0.005, 0.0, 0.0) is True
    assert policy.is_landing("x", 0.005, 0.02, 0.0) is False
    assert policy.is_landing("x", 0.005, 0.0, -0.02) is False


def test_semantic_hint_seven_bands():
    pos = np.array([0.0, 0.0, 1.0])
    identity = [0.0, 0.0, 0.0, 1.0]  # yaw 0, body frame == world frame
    # angle_deg = atan2(y, x) in body frame; with yaw 0 that is world atan2.
    cases = {
        (10.0, 0.0): "straight ahead ",      # 0 deg
        (7.0, 3.0): "forward-right ",        # ~23 deg
        (1.0, 10.0): "to your right ",       # ~84 deg
        (-5.0, 1.0): "to your right rear ",  # ~169 deg
        (7.0, -3.0): "forward-left ",        # ~-23 deg
        (-1.0, -10.0): "to your left ",      # ~-96 deg
        (-5.0, -1.0): "to your left rear ",  # ~-169 deg
    }
    for (tx, ty), expected in cases.items():
        got = policy.semantic_hint(pos, identity, np.array([tx, ty, 1.0]))
        assert got == expected, (tx, ty, got)


def test_semantic_hint_band_boundaries():
    # Band edges at 15/60/120 deg (wrapper L68-74). Tested just inside/outside
    # each edge; exact-boundary float noise would make ==-style edge probes
    # non-robust, so +/-0.001 deg straddles are used instead.
    identity = [0.0, 0.0, 0.0, 1.0]
    pos = np.array([0.0, 0.0, 0.0])
    def at(deg):
        r = math.radians(deg)
        return policy.semantic_hint(pos, identity, np.array([math.cos(r), math.sin(r), 0.0]))
    assert at(14.999) == "straight ahead "   # -15<=a<=15
    assert at(15.001) == "forward-right "    # 15<a<=60
    assert at(59.999) == "forward-right "
    assert at(60.001) == "to your right "    # 60<a<=120
    assert at(119.999) == "to your right "
    assert at(120.001) == "to your right rear "  # 120<a<=180
    assert at(179.999) == "to your right rear "
    assert at(-14.999) == "straight ahead "
    assert at(-15.001) == "forward-left "    # -60<=a<-15
    assert at(-59.999) == "forward-left "
    assert at(-60.001) == "to your left "    # -120<=a<-60
    assert at(-179.999) == "to your left rear "  # -180<=a<-120


def test_semantic_hint_empty_when_over_target():
    # dist_xy < 0.01 -> "" regardless of vertical offset (wrapper L65-66).
    pos = np.array([0.0, 0.0, 5.0])
    identity = [0.0, 0.0, 0.0, 1.0]
    assert policy.semantic_hint(pos, identity, np.array([0.005, 0.0, 20.0])) == ""
    assert policy.semantic_hint(pos, identity, np.array([0.0, 0.0, 5.0])) == ""


def test_semantic_hint_uses_body_frame_under_yaw():
    # The function is verbatim NED-frame upstream (body +y = right; the
    # planner_node converts ENU poses to NED before calling it). A scipy
    # +90 deg z rotation puts world +x at body angle -90 -> "to your left".
    yaw90 = [0.0, 0.0, math.sin(math.pi / 4), math.cos(math.pi / 4)]
    hint = policy.semantic_hint(np.array([0.0, 0.0, 0.0]), yaw90, np.array([10.0, 0.0, 0.0]))
    assert hint == "to your left "


def test_semantic_hint_accepts_dict_orientation():
    identity = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'w': 1.0}
    assert policy.semantic_hint([0, 0, 0], identity, [10, 0, 0]) == "straight ahead "


def _state(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return {'position': [x, y, z],
            'orientation': [0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)]}


def test_action_small_yaw_cruise_speed_and_duration():
    # total_dist >= MICRO_MOVE_THRESHOLD(1.0): unit vector * CRUISE_SPEED(1.0),
    # duration = total_dist / CRUISE_SPEED (upstream L368-374).
    pts = policy.action_to_setpoints(_state(), fwd=3.0, down=0.0, yaw=0.0)
    # handoff + move + hover = 3 points
    assert len(pts) == 3
    move = pts[1]
    speed = math.hypot(move['velocity'][0], move['velocity'][1])
    assert speed == pytest.approx(policy.CRUISE_SPEED, abs=1e-9)
    assert move['t'] == pytest.approx(3.0, abs=1e-9)  # total_dist/CRUISE
    assert move['position'][0] == pytest.approx(3.0, abs=1e-9)
    # trailing hover at +0.5 s with zero velocity (upstream L386)
    hover = pts[2]
    assert hover['t'] == pytest.approx(3.5, abs=1e-9)
    assert hover['velocity'] == (0.0, 0.0, 0.0)


def test_action_small_yaw_micro_move_duration_one_second():
    # total_dist < 1.0: duration = 1.0, v = d/1.0 (upstream L362-366).
    pts = policy.action_to_setpoints(_state(), fwd=0.5, down=0.0, yaw=0.0)
    move = pts[1]
    assert move['t'] == pytest.approx(1.0, abs=1e-9)
    assert move['velocity'][0] == pytest.approx(0.5, abs=1e-9)
    assert move['position'][0] == pytest.approx(0.5, abs=1e-9)


def test_action_large_turn_drops_horizontal_displacement():
    # |yaw| >= YAW_THRESHOLD(0.25): rotate first, then ONLY vertical move at
    # velocity 2.0; the dx/dy computed from fwd are NOT applied (upstream
    # L355-356 + L383-384). This is the migration-plan risk branch.
    pts = policy.action_to_setpoints(_state(), fwd=3.0, down=1.0, yaw=1.0)
    move = pts[1]
    # horizontal position unchanged from start (0,0), only z moved by down.
    assert move['position'][0] == pytest.approx(0.0, abs=1e-9)
    assert move['position'][1] == pytest.approx(0.0, abs=1e-9)
    assert move['position'][2] == pytest.approx(1.0, abs=1e-9)
    # vertical velocity magnitude is 2.0 (moveToZAsync velocity=2.0).
    assert abs(move['velocity'][2]) == pytest.approx(2.0, abs=1e-9)
    assert move['velocity'][0] == 0.0 and move['velocity'][1] == 0.0
    # yaw reaches curr_yaw + action yaw.
    assert move['yaw'] == pytest.approx(1.0, abs=1e-9)


def test_action_large_turn_no_vertical_is_rotation_only():
    pts = policy.action_to_setpoints(_state(), fwd=2.0, down=0.0, yaw=0.5)
    move = pts[1]
    assert move['velocity'] == (0.0, 0.0, 0.0)
    assert move['position'] == (0.0, 0.0, 0.0)
    assert move['t'] == pytest.approx(policy.ROTATE_DURATION, abs=1e-9)
    assert move['yaw'] == pytest.approx(0.5, abs=1e-9)


def test_action_in_place_small_yaw_is_rotation_only():
    # fwd~0, down~0 but yaw>EPSILON and total_dist<=EPSILON -> rotate only.
    pts = policy.action_to_setpoints(_state(), fwd=0.0, down=0.0, yaw=0.1)
    move = pts[1]
    assert move['velocity'] == (0.0, 0.0, 0.0)
    assert move['yaw'] == pytest.approx(0.1, abs=1e-9)


def test_action_no_motion_is_single_hover():
    pts = policy.action_to_setpoints(_state(), fwd=0.0, down=0.0, yaw=0.0)
    # handoff + hover only (no move block; upstream L352 false, L386 hover).
    assert len(pts) == 2
    assert pts[1]['t'] == pytest.approx(policy.HOVER_DURATION, abs=1e-9)
    assert pts[1]['velocity'] == (0.0, 0.0, 0.0)


def test_action_first_point_is_t0_handoff_at_current_pose():
    st = _state(x=1.0, y=2.0, z=3.0, yaw=0.3)
    pts = policy.action_to_setpoints(st, fwd=2.0, down=0.0, yaw=0.0)
    assert pts[0]['t'] == 0.0
    assert pts[0]['position'] == pytest.approx((1.0, 2.0, 3.0), abs=1e-9)
    assert pts[0]['velocity'] == (0.0, 0.0, 0.0)
    assert pts[0]['yaw'] == pytest.approx(0.3, abs=1e-9)


def test_action_displacement_respects_current_yaw_frame():
    # With a 90 deg body yaw and small action yaw, fwd=2 moves along world +y.
    st = _state(yaw=math.pi / 2)
    pts = policy.action_to_setpoints(st, fwd=2.0, down=0.0, yaw=0.0)
    move = pts[1]
    assert move['position'][0] == pytest.approx(0.0, abs=1e-6)
    assert move['position'][1] == pytest.approx(2.0, abs=1e-6)


def test_action_down_negative_moves_z_down():
    # Small-yaw pure descent: total_dist=2 >= MICRO -> normalized * CRUISE,
    # so |v| = 1.0 and duration = 2.0 (upstream L368-374), NOT 2.0 m/s
    # (that velocity belongs to the large-turn moveToZAsync branch L384).
    pts = policy.action_to_setpoints(_state(z=5.0), fwd=0.0, down=-2.0, yaw=0.0)
    move = pts[1]
    assert move['position'][2] == pytest.approx(3.0, abs=1e-9)
    assert move['velocity'][2] == pytest.approx(-1.0, abs=1e-9)
    assert move['t'] == pytest.approx(2.0, abs=1e-9)
