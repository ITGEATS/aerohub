import os
import tempfile
import unittest
from pathlib import Path

from prepare_params import build_parameters, flatten


def config_source():
    configured = os.environ.get("FALCON_CONFIG_SOURCE")
    if configured:
        return Path(configured)
    return (Path(__file__).resolve().parents[3]
            / "src/tool/navigation/planner/falcon/upstream/falcon_planner")


class ParameterTests(unittest.TestCase):
    def test_nested_values_are_flattened_for_ros_compatibility_layer(self):
        self.assertEqual(
            flatten({"a": {"b": 2}, "enabled": True}),
            {"a.b": 2, "enabled": True})

    def test_renderer_transforms_are_intentionally_ignored(self):
        self.assertEqual(flatten({"map_config": {"T_b_c": [[1, 0], [0, 1]]}}), {})

    def test_unhandled_sequences_are_rejected(self):
        with self.assertRaises(ValueError):
            flatten({"planner": {"magic": [1, 2]}})

    def test_runtime_outputs_are_kept_outside_the_fork(self):
        source = config_source()
        self.assertTrue(source.is_dir(), f"FALCON config source is missing: {source}")
        with tempfile.TemporaryDirectory() as directory:
            runtime = Path(directory).resolve()
            parameters = build_parameters(source, runtime)
            self.assertEqual(parameters["exploration_manager.tsp_dir"], str(runtime))
            self.assertEqual(parameters["voxel_mapping.map_save_path"], str(runtime / "map.pcd"))

    def test_publish_periods_are_numeric_durations(self):
        source = config_source()
        self.assertTrue(source.is_dir(), f"FALCON config source is missing: {source}")
        with tempfile.TemporaryDirectory() as directory:
            parameters = build_parameters(source, Path(directory).resolve())
            for name in (
                    "voxel_mapping.publish_tsdf_period",
                    "voxel_mapping.publish_esdf_period",
                    "voxel_mapping.publish_occupancy_grid_period"):
                self.assertIsInstance(parameters[name], float)
                self.assertEqual(parameters[name], 1.0)


if __name__ == "__main__":
    unittest.main()
