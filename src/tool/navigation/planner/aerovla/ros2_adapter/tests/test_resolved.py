"""Host tests for the resolved task-config loader (framework contract)."""

import pytest

from uav_aerovla_planner import resolved

SAMPLE = """
task:
  name: aerovla blocks
  task_id: 11111111-2222-3333-4444-555555555555
planner:
  id: aerovla
  version: b87aaa7c9338ae878d39300a1ef930c753e1ff90
  manifest: registry/aerovla/b87aaa7c9338ae878d39300a1ef930c753e1ff90/manifest.yaml
  runtime:
    image: board-native
    adapter: uav_aerovla_planner.planner_node
    command: [python3, -m, uav_aerovla_planner.planner_node]
  parameters:
    trajectory_timeout: 8.0
    rate_hz: 2.0
    obj_desc: The orange ball.
    target_position_enu: [34.25, 33.1, -3.68]
    prompt_table: /data/aerohub/lib/aerovla/prompt_ids_blocks.json
vehicles: [drone0]
deployment:
  drone0:
    namespace: drone0
    px4_origin_in_fleet_ned: [0.0, 0.0, 0.0]
    topics:
      pose: /drone0/self_localization/pose
"""


def _write(tmp_path, text):
    path = tmp_path / "resolved.yaml"
    path.write_text(text, encoding="utf-8")
    return str(path)


def test_load_resolved_minimal_view(tmp_path):
    view = resolved.load_resolved(_write(tmp_path, SAMPLE))
    assert view["task_id"] == "11111111-2222-3333-4444-555555555555"
    assert view["vehicle_id"] == "drone0"
    assert view["namespace"] == "drone0"
    assert view["parameters"]["rate_hz"] == 2.0
    assert view["parameters"]["target_position_enu"] == [34.25, 33.1, -3.68]


def test_load_resolved_missing_task_id(tmp_path):
    broken = SAMPLE.replace("  task_id: 11111111-2222-3333-4444-555555555555\n", "")
    with pytest.raises(resolved.ResolvedConfigError):
        resolved.load_resolved(_write(tmp_path, broken))


def test_load_resolved_missing_parameters(tmp_path):
    broken = SAMPLE.replace("  parameters:\n", "  bogus:\n")
    with pytest.raises(resolved.ResolvedConfigError):
        resolved.load_resolved(_write(tmp_path, broken))


def test_load_resolved_empty_vehicles(tmp_path):
    broken = SAMPLE.replace("vehicles: [drone0]", "vehicles: []")
    with pytest.raises(resolved.ResolvedConfigError):
        resolved.load_resolved(_write(tmp_path, broken))


def test_load_resolved_missing_file():
    with pytest.raises(OSError):
        resolved.load_resolved("/nonexistent/resolved.yaml")
