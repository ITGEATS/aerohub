#!/usr/bin/env python3
"""Merge locked FALCON configs into one ROS 2 parameter file."""

from __future__ import annotations

import argparse
from pathlib import Path

import yaml


CONFIGS = (
    "exploration_manager/config/map/complex_office.yaml",
    "exploration_manager/config/robot/uav_model_simulator.yaml",
    "voxel_mapping/config/voxel_mapping.yaml",
    "pathfinding/config/astar.yaml",
    "fast_planner/config/fast_planner.yaml",
    "exploration_preprocessing/config/frontier_finder.yaml",
    "exploration_preprocessing/config/hgrid.yaml",
    "exploration_utils/config/perception_utils.yaml",
    "exploration_manager/config/exploration_manager.yaml",
)


def flatten(mapping, prefix=""):
    output = {}
    for key, value in mapping.items():
        name = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            output.update(flatten(value, name))
        elif isinstance(value, (str, bool, int, float)) or value is None:
            output[name] = value
        elif isinstance(value, list):
            # T_b_c and T_m_w belong to the bundled renderer, not the live
            # AirSim sensor path. Reject every other unhandled sequence.
            if name not in ("map_config.T_b_c", "map_config.T_m_w"):
                raise ValueError(f"unsupported sequence parameter: {name}")
        else:
            raise ValueError(f"unsupported parameter value: {name}")
    return output


def build_parameters(source: Path, runtime_dir: Path):
    merged = {}
    for relative in CONFIGS:
        value = yaml.safe_load((source / relative).read_text(encoding="utf-8"))
        if not isinstance(value, dict):
            raise ValueError(f"{relative} is not a mapping")
        overlap = set(merged).intersection(flatten(value))
        if overlap:
            raise ValueError(f"duplicate parameters in locked configs: {sorted(overlap)}")
        merged.update(flatten(value))
    merged.update({
        "use_sim_time": True,
        "map_config.map_dimension": 2,
        "map_config.init_x": 0.0,
        "map_config.init_y": 0.0,
        "map_config.init_z": 2.0,
        "map_config.init_yaw": 0.0,
        "map_config.map_size.map_min_x": -8.0,
        "map_config.map_size.map_min_y": -8.0,
        "map_config.map_size.map_min_z": 0.0,
        "map_config.map_size.map_max_x": 8.0,
        "map_config.map_size.map_max_y": 8.0,
        "map_config.map_size.map_max_z": 5.0,
        "map_config.map_size.box_min_x": -6.0,
        "map_config.map_size.box_min_y": -6.0,
        "map_config.map_size.box_min_z": 0.5,
        "map_config.map_size.box_max_x": 6.0,
        "map_config.map_size.box_max_y": 6.0,
        "map_config.map_size.box_max_z": 4.0,
        "map_config.map_size.vbox_min_x": -7.0,
        "map_config.map_size.vbox_min_y": -7.0,
        "map_config.map_size.vbox_min_z": 0.0,
        "map_config.map_size.vbox_max_x": 7.0,
        "map_config.map_size.vbox_max_y": 7.0,
        "map_config.map_size.vbox_max_z": 5.0,
        "uav_model.sensing_parameters.camera_intrinsics.fx": 128.0,
        "uav_model.sensing_parameters.camera_intrinsics.fy": 128.0,
        "uav_model.sensing_parameters.camera_intrinsics.cx": 127.5,
        "uav_model.sensing_parameters.camera_intrinsics.cy": 127.5,
        "uav_model.sensing_parameters.image_width": 256,
        "uav_model.sensing_parameters.image_height": 256,
        "uav_model.sensing_parameters.min_depth": 0.1,
        "uav_model.sensing_parameters.max_depth": 20.0,
        "uav_model.sensing_parameters.fov.horizontal": 1.5707963267948966,
        "uav_model.sensing_parameters.fov.vertical": 1.5707963267948966,
        "transformer.world_frame": "world",
        "transformer.sensor_frame": "camera",
        "transformer.pose_topic_type": "transform",
        "transformer.timestamp_tolerance": 0.001,
        "perception_utils.top_angle": 0.7853981633974483,
        "perception_utils.left_angle": 0.7853981633974483,
        "perception_utils.right_angle": 0.7853981633974483,
        "exploration_manager.auto_start": False,
        "exploration_manager.tsp_dir": str(runtime_dir),
        "voxel_mapping.map_save_path": str(runtime_dir / "map.pcd"),
        # The upstream YAML uses booleans for these duration fields, while the
        # mapper reads them as seconds (double).
        "voxel_mapping.publish_tsdf_period": 1.0,
        "voxel_mapping.publish_esdf_period": 1.0,
        "voxel_mapping.publish_occupancy_grid_period": 1.0,
        # Leave enough future coverage for DDS and FollowTimedPath handoff.
        "exploration_manager.fsm.replan_duration_fast": 0.25,
        "exploration_manager.fsm.replan_duration_default": 0.35,
        "exploration_manager.fsm.replan_duration_slow": 0.5,
    })
    return merged


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--runtime-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.runtime_dir.mkdir(parents=True, exist_ok=True)
    parameters = build_parameters(args.source, args.runtime_dir.resolve())
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(yaml.safe_dump({
        "/**": {"ros__parameters": parameters}
    }, sort_keys=True), encoding="utf-8")


if __name__ == "__main__":
    main()
