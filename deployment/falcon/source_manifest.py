#!/usr/bin/env python3
"""Compute the deterministic source digest embedded in FALCON images."""

from __future__ import annotations

import argparse
import hashlib
from pathlib import Path


INPUTS = (
    "src/tool/navigation/planner/falcon/upstream/falcon_planner",
    "src/tool/navigation/planner/falcon/ros2_adapter",
    "src/tool/navigation/interfaces",
    "src/behavior/follow_path",
    "deployment/falcon/Dockerfile",
    "deployment/falcon/Dockerfile.workstation",
    "deployment/falcon/build.sh",
    "deployment/falcon/check.sh",
    "deployment/falcon/compute_runtime.py",
    "deployment/falcon/entrypoint.sh",
    "deployment/falcon/entrypoint-workstation.sh",
    "deployment/falcon/mission.py",
    "deployment/falcon/prepare_params.py",
    "deployment/falcon/prepare_workstation.py",
    "deployment/falcon/run.py",
    "deployment/falcon/profile.json",
    "deployment/falcon/source_manifest.py",
    "deployment/falcon/tests",
    "simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py",
    "deployment/airsim/demos/catora_open_field/sim_startup.py",
    "deployment/airsim/as2/run.py",
    "deployment/airsim/config/as2/config.yaml",
    "deployment/airsim/config/as2/pid_speed_controller.yaml",
    "deployment/airsim/config/dds/fastdds_udp.xml",
    "deployment/robopi/vendor/msgpack",
    "deployment/robopi/vendor/msgpack-source.json",
)


def source_files(root: Path):
    files = []
    for relative in INPUTS:
        path = root / relative
        if not path.exists():
            raise FileNotFoundError(f"source manifest input is missing: {relative}")
        candidates = path.rglob("*") if path.is_dir() else (path,)
        files.extend(
            candidate for candidate in candidates
            if candidate.is_file()
            and "__pycache__" not in candidate.parts
            and candidate.suffix != ".pyc"
        )
    return sorted(set(files), key=lambda path: path.relative_to(root).as_posix())


def source_digest(root: Path) -> str:
    digest = hashlib.sha256()
    for path in source_files(root.resolve()):
        relative = path.relative_to(root.resolve()).as_posix().encode()
        digest.update(relative)
        digest.update(b"\0")
        digest.update(hashlib.sha256(path.read_bytes()).digest())
        digest.update(b"\n")
    return digest.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, required=True)
    args = parser.parse_args()
    print(source_digest(args.root))


if __name__ == "__main__":
    main()
