"""Isolated visible-AirSim FALCON acceptance using the fixed project images."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import subprocess
import sys
import time


ROOT = Path(__file__).resolve().parents[2]
FALCON_IMAGE = "aerohub/falcon:ros2-humble"
FALCON_WORKSTATION_IMAGE = "aerohub/falcon-workstation:ros2-humble"
sys.path.insert(0, str(ROOT / "deployment/falcon"))
sys.path.insert(0, str(ROOT / "deployment/airsim/demos/catora_open_field"))
from source_manifest import source_digest, source_files
from sim_startup import StartupRelay


def run(command, **kwargs):
    return subprocess.run([str(value) for value in command], check=True, **kwargs)


def resolve_airsim_binary(path: Path) -> Path:
    binary = path.expanduser().resolve()
    if binary.suffix == ".sh":
        packaged = binary.parent / binary.stem / "Binaries/Linux" / binary.stem
        if packaged.is_file() and os.access(packaged, os.X_OK):
            return packaged.resolve()
    return binary


def free_domain():
    occupied = set()
    ids = subprocess.check_output(["docker", "ps", "-q"], text=True).split()
    if ids:
        for container in json.loads(subprocess.check_output(
                ["docker", "inspect", *ids], text=True)):
            for value in container["Config"].get("Env", []):
                if value.startswith("ROS_DOMAIN_ID="):
                    occupied.add(int(value.split("=", 1)[1]))
    return next(value for value in range(220, 230) if value not in occupied)


def wait_px4_preflight(container, output, timeout=60.0, stable_seconds=2.0):
    deadline = time.monotonic() + timeout
    ready_since = None
    samples = 0
    latest = ""
    while time.monotonic() < deadline:
        log = subprocess.check_output(
            ["docker", "logs", container], stderr=subprocess.STDOUT, text=True)
        if "Ready for takeoff!" not in log:
            ready_since = None
            time.sleep(0.25)
            continue
        latest = subprocess.check_output([
            "docker", "exec", container,
            "/opt/px4-gazebo/bin/px4-listener", "--instance", "70",
            "vehicle_status", "-n", "1"], text=True, timeout=10)
        samples += 1
        if "pre_flight_checks_pass: true" in latest.lower():
            ready_since = ready_since or time.monotonic()
            if time.monotonic() - ready_since >= stable_seconds:
                (output / "preflight-uorb.txt").write_text(latest)
                (output / "preflight.json").write_text(json.dumps({
                    "ready": True,
                    "source": "target firmware uORB vehicle_status",
                    "samples": samples,
                    "stable_seconds": time.monotonic() - ready_since,
                }, indent=2) + "\n")
                return
        else:
            ready_since = None
        time.sleep(0.25)
    (output / "preflight-uorb.txt").write_text(latest)
    raise TimeoutError("PX4 pre_flight_checks_pass did not remain true")


def source_hashes():
    import hashlib
    return {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in source_files(ROOT)
    }


def image_label(image, name):
    value = subprocess.check_output([
        "docker", "image", "inspect", "--format",
        f"{{{{index .Config.Labels {json.dumps(name)}}}}}", image,
    ], text=True).strip()
    if not value or value == "<no value>":
        raise RuntimeError(f"{image} is missing image label {name}")
    return value


def inspect_mounts(container):
    data = json.loads(subprocess.check_output(
        ["docker", "inspect", container], text=True))[0]
    return sorted(({
        "type": mount["Type"],
        "source": str(Path(mount["Source"]).resolve()),
        "destination": mount["Destination"],
        "rw": bool(mount["RW"]),
    } for mount in data.get("Mounts", [])), key=lambda item: item["destination"])


def require_mounts(container, expected):
    observed = inspect_mounts(container)
    if observed != sorted(expected, key=lambda item: item["destination"]):
        raise RuntimeError(
            f"runtime mount policy failed for {container}: {observed}")
    return observed


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--binary", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--case", choices=("normal", "depth-loss", "planner-exit"),
        default="normal")
    args = parser.parse_args()
    display = os.environ.get("DISPLAY")
    if not display:
        raise RuntimeError("visible acceptance requires a DISPLAY")
    if subprocess.run(
            ["xdpyinfo"], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL).returncode != 0:
        raise RuntimeError(f"visible acceptance cannot connect to DISPLAY={display}")
    if os.environ.get("AIRSIM_RENDER_MODE") != "windowed":
        raise RuntimeError("visible acceptance requires AIRSIM_RENDER_MODE=windowed")

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    current_source_digest = source_digest(ROOT)
    for image in (FALCON_IMAGE, FALCON_WORKSTATION_IMAGE):
        embedded = image_label(image, "io.aerohub.source.digest")
        if embedded != current_source_digest:
            raise RuntimeError(
                f"{image} was built from {embedded}, current source is "
                f"{current_source_digest}; rebuild before acceptance")
    binary = resolve_airsim_binary(args.binary)
    domain = free_domain()
    settings = json.loads((
        ROOT / "simulator/airsim/scenarios/super_forest/settings.json").read_text())
    settings["ApiServerPort"] = 43482
    vehicle = settings["Vehicles"]["PX4"]
    vehicle.update(TcpPort=4893, ControlPortLocal=15743, ControlPortRemote=15783)
    vehicle["Sensors"].pop("Lidar", None)
    front = vehicle["Cameras"]["FrontCamera"]
    # FALCON plans from the vehicle origin. Keep the depth ray origin there so
    # the planner start voxel is observed instead of left behind the camera.
    front.update(X=0.0, Y=0.0, Z=0.0)
    front["CaptureSettings"] = [{
        "ImageType": 1, "Width": 256, "Height": 256, "FOV_Degrees": 90,
    }]
    vehicle["Cameras"] = {"FrontCamera": front}
    (output / "settings.json").write_text(json.dumps(settings, indent=2) + "\n")
    px4_settings = json.loads(json.dumps(settings))
    px4_settings["Vehicles"]["PX4"]["TcpPort"] = 4993
    (output / "px4-settings.json").write_text(json.dumps(px4_settings, indent=2) + "\n")
    (output / "fleet.yaml").write_text(
        "vehicles:\n  drone0:\n    fmu_prefix: /drone0\n    target_system_id: 71\n")
    profile = {
        "binary": str(binary),
        "settings": str(output / "settings.json"),
        "overhead_pose_source": "object",
        "project": "AirSimNH",
        "args": ["-vulkan", "-nosound", "-windowed", "-ResX=960", "-ResY=540"],
    }
    (output / "profile.json").write_text(json.dumps(profile, indent=2) + "\n")

    images = ("flight_board:v0.1.0", FALCON_IMAGE, FALCON_WORKSTATION_IMAGE)
    manifest = {
        "case": args.case, "ros_domain": domain,
        "display": display, "render_mode": "windowed",
        "source_hashes": source_hashes(),
        "source_digest": current_source_digest,
        "fork_revision": subprocess.check_output([
            "git", "-C", str(ROOT / "src/tool/navigation/planner/falcon/upstream"),
            "rev-parse", "HEAD"], text=True).strip(),
        "parent_revision": subprocess.check_output([
            "git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
        "parent_dirty": bool(subprocess.check_output([
            "git", "-C", str(ROOT), "status", "--porcelain"], text=True)),
        "image_ids": {image: subprocess.check_output([
            "docker", "image", "inspect", "--format", "{{.Id}}", image],
            text=True).strip() for image in images},
        "criteria": {
            "actual_depth_and_timestamp_matched_pose": True,
            "minimum_motion_m": 1.0,
            "free_map_growth_points": ">0",
            "bounds_enu_m": {"min": [-8, -8, 0], "max": [8, 8, 5]},
            "collision_count": 0,
            "normal_completion": args.case == "normal",
            "expected_fault": args.case if args.case != "normal" else None,
            "stop_confirmation": True,
            "land_disarmed": True,
        },
    }
    (output / "manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")

    names = [
        "aerohub-falcon-vehicle", "aerohub-falcon-clock",
        "aerohub-falcon-compute", "aerohub-falcon-workstation"]
    for name in names:
        if subprocess.run(["docker", "inspect", name], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0:
            raise RuntimeError("reserved container already exists: " + name)
    manager = [sys.executable, ROOT / "simulator/airsim/manager/manager.py"]
    owned = []
    runtime_mounts = {}
    relay = None
    scene_started = False
    try:
        run(manager + ["start", "--profile", output / "profile.json",
                       "--run-dir", output / "scene", "--timeout", "90"],
            env=dict(os.environ, AIRSIM_RENDER_MODE="windowed"))
        scene_started = True
        relay = StartupRelay(("127.0.0.1", 4993), ("127.0.0.1", 4893))
        for name, mode in zip(names[:2], ("vehicle", "clock")):
            command = [
                "docker", "run", "--rm", "--detach", "--init", "--network", "host",
                "--name", name, "-e", f"ROS_DOMAIN_ID={domain}",
                "-e", "VEHICLE_NAME=PX4", "-e", "VEHICLE_NAMESPACE=drone0",
                "-e", "PX4_INSTANCE=70", "-e", "PX4_SYSTEM_ID=71",
                "-e", "XRCE_AGENT_PORT=8963", "-e", "AIRSIM_RPC_PORT=43482",
                "-e", "FLIGHT_BOARD_SENSORS=1", "-e", "FLIGHT_BOARD_CAMERAS=1",
                "-v", f"{output / 'px4-settings.json'}:/config/settings.json:ro",
                "flight_board:v0.1.0", mode,
            ]
            run(command, stdout=subprocess.DEVNULL)
            owned.append(name)
            runtime_mounts[name] = require_mounts(name, [{
                "type": "bind",
                "source": str((output / "px4-settings.json").resolve()),
                "destination": "/config/settings.json",
                "rw": False,
            }])
        wait_px4_preflight(names[0], output)

        compute_command = [
            "docker", "run", "--rm", "--detach", "--init", "--network", "host",
            "--name", names[2], "-e", f"ROS_DOMAIN_ID={domain}",
            "-e", "AS2_EXECUTION_BACKEND=follow_path",
            "-e", "AS2_TIMED_CONTINUITY_TOLERANCE=0.25",
            "-e", "AS2_TIMED_YAW_CONTINUITY_TOLERANCE=0.20",
            "-e", "AS2_FLEET_FILE=/results/fleet.yaml",
            "-e", "AS2_CONFIG_FILE=/opt/falcon/runtime/as2_config.yaml",
            "-e", "AS2_CONTROLLER_CONFIG_FILE=/opt/falcon/runtime/pid.yaml",
            "-e", "AIRSIM_RPC_PORT=43482",
            "-v", f"{output}:/results:rw",
            FALCON_IMAGE, "compute",
        ]
        run(compute_command, stdout=subprocess.DEVNULL)
        owned.append(names[2])
        runtime_mounts[names[2]] = require_mounts(names[2], [{
            "type": "bind", "source": str(output),
            "destination": "/results", "rw": True,
        }])

        workstation_command = [
            "docker", "run", "--detach", "--init", "--network", "host",
            "--name", names[3], "-e", f"ROS_DOMAIN_ID={domain}",
            "-e", f"FALCON_CASE={args.case}",
            "-v", f"{output}:/results:rw",
            FALCON_WORKSTATION_IMAGE, "mission",
        ]
        run(workstation_command, stdout=subprocess.DEVNULL)
        owned.append(names[3])
        runtime_mounts[names[3]] = require_mounts(names[3], [{
            "type": "bind", "source": str(output),
            "destination": "/results", "rw": True,
        }])
        (output / "runtime-mounts.json").write_text(
            json.dumps(runtime_mounts, indent=2, sort_keys=True) + "\n")
        exit_code = int(subprocess.check_output(
            ["docker", "wait", names[3]], text=True).strip())
        with (output / "mission.log").open("w") as stream:
            subprocess.run(["docker", "logs", names[3]], stdout=stream,
                           stderr=subprocess.STDOUT)
        if exit_code:
            raise subprocess.CalledProcessError(exit_code, workstation_command)
    finally:
        for name in reversed(owned):
            with (output / f"{name}.log").open("w") as stream:
                subprocess.run(["docker", "logs", name], stdout=stream,
                               stderr=subprocess.STDOUT)
            subprocess.run(["docker", "stop", "--time", "10", name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            subprocess.run(["docker", "rm", "--force", name],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if relay is not None:
            relay.close()
        if scene_started:
            run(manager + ["stop", "--run-dir", output / "scene"])
        remaining = [name for name in names if subprocess.run(
            ["docker", "inspect", name], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL).returncode == 0]
        (output / "cleanup.json").write_text(json.dumps({
            "containers_absent": not remaining,
            "remaining_containers": remaining,
            "scene_stopped": scene_started,
        }, indent=2) + "\n")
        if remaining and sys.exc_info()[0] is None:
            raise RuntimeError("FALCON containers remained after cleanup: " + str(remaining))


if __name__ == "__main__":
    main()
