#!/usr/bin/env python3
"""Prepare a canonical AS2 interface subset for the workstation role."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import shutil


INTERFACES = (
    "action/Land.action",
    "action/Takeoff.action",
    "msg/ControlMode.msg",
    "msg/PlatformInfo.msg",
    "msg/PlatformStatus.msg",
    "msg/TrajectoryPoint.msg",
    "msg/TrajectorySetpoints.msg",
)

CMAKE = """cmake_minimum_required(VERSION 3.8)
project(as2_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  \"action/Land.action\"
  \"action/Takeoff.action\"
  \"msg/ControlMode.msg\"
  \"msg/PlatformInfo.msg\"
  \"msg/PlatformStatus.msg\"
  \"msg/TrajectoryPoint.msg\"
  \"msg/TrajectorySetpoints.msg\"
  DEPENDENCIES std_msgs geometry_msgs action_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
"""

PACKAGE = """<?xml version=\"1.0\"?>
<package format=\"3\">
  <name>as2_msgs</name><version>1.1.3</version>
  <description>Canonical AS2 interfaces required by the FALCON workstation client</description>
  <maintainer email=\"dev@example.com\">AeroHub</maintainer>
  <license>BSD-3-Clause</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <build_depend>rosidl_default_generators</build_depend>
  <depend>std_msgs</depend><depend>geometry_msgs</depend><depend>action_msgs</depend>
  <exec_depend>rosidl_default_runtime</exec_depend>
  <member_of_group>rosidl_interface_packages</member_of_group>
  <export><build_type>ament_cmake</build_type></export>
</package>
"""


def prepare(source: Path, output: Path) -> dict:
    if output.exists():
        shutil.rmtree(output)
    package = output / "src/as2_msgs"
    package.mkdir(parents=True)
    hashes = {}
    for relative in INTERFACES:
        data = (source / relative).read_bytes()
        target = package / relative
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_bytes(data)
        hashes[relative] = hashlib.sha256(data).hexdigest()
    (package / "CMakeLists.txt").write_text(CMAKE)
    (package / "package.xml").write_text(PACKAGE)
    manifest = {"source": "locked-onboard-as2-msg-definitions", "interfaces": hashes}
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")
    return manifest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    prepare(args.source, args.output)


if __name__ == "__main__":
    main()
