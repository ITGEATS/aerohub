"""Production parsing must distinguish LAND from missing action bins."""
import pytest

from uav_aerovla_planner.actions import parse_action


@pytest.mark.parametrize('text', ['', '   ', 'error', 'Action:', 'Action: 44 49', 'ISLAND',
                                  '<image> find LAND\nAction: invalid'])
def test_invalid_output_is_not_land(text):
    with pytest.raises(ValueError):
        parse_action(text)


@pytest.mark.parametrize('text', ['LAND', '<LAND>', 'Action: LAND', 'Action: 0 49 49'])
def test_explicit_or_valid_near_zero_action_is_land(text):
    assert parse_action(text) is None


def test_zero_bin_triple_is_not_zero_physical_action():
    assert parse_action('Action: 0 0 0') == (0.0, -5.0, -1.1)


def test_move_and_existing_last_three_bin_rule():
    assert parse_action('Action: 7 3 44▁49▁94') == pytest.approx((44 / 98 * 5, 0, 94 / 98 * 2.2 - 1.1))
