#!/usr/bin/env python3
"""Own the D2SLAM visible three-vehicle AirSim/PX4 acceptance matrix."""
from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import threading
import time

import yaml

ROOT = Path(__file__).resolve().parents[2]
CONFIG = ROOT / "deployment/swarm/configs/d2_slam.yaml"
PROFILE = ROOT / "simulator/airsim/scenarios/d2_slam/profile.json"
DDS = ROOT / "deployment/airsim/config/dds/fastdds_udp.xml"
AS2_CONFIG = ROOT / "deployment/airsim/config/as2/config.yaml"
AS2_CONTROLLER = ROOT / "deployment/airsim/config/as2/pid_speed_controller.yaml"
AS2_RUN = ROOT / "deployment/airsim/as2/run.py"
PROBE = ROOT / "deployment/d2_slam/ground_probe.py"
MISSION = ROOT / "deployment/d2_slam/flight_validation.py"
VALIDATION = ROOT / "deployment/d2_slam/validation.yaml"
AIRSIM_RPC = ROOT / "simulator/airsim/sensors/uav_as2_sensor_adapter/rpc.py"
WORKSPACE = Path(
    os.environ.get("D2_SLAM_WORKSPACE", ROOT / "log/workspaces/d2_slam")
).resolve()
PX4_WORKSPACE = Path(os.environ.get("PX4_MSGS_OVERLAY_WORKSPACE", WORKSPACE / "px4_msgs"))
BOARD = os.environ.get("FLIGHT_BOARD_IMAGE", "flight_board:v0.1.0")
ONBOARD = os.environ.get("ONBOARD_COMPUTER_IMAGE", "onboard_computer:v0.1.0")
DOMAIN = 92
PREFIX = "aerohub-d2-e2e-"


def execute(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def container_exists(name: str) -> bool:
    return subprocess.run(
        ["docker", "container", "inspect", name], capture_output=True).returncode == 0


def manager(action: str, output: Path) -> list[str]:
    command = [sys.executable, str(ROOT / "simulator/airsim/manager/manager.py"), action,
               "--profile", str(output / "profile.json"), "--run-dir", str(output / "scene")]
    binary = os.environ.get("AIRSIM_BINARY")
    if binary:
        command.extend(("--binary", binary))
    return command


def run_detached(name: str, image: str, environment: dict[str, str], volumes, command):
    args = ["docker", "run", "--detach", "--init", "--network", "host", "--name", name]
    for key, value in environment.items():
        args.extend(("--env", f"{key}={value}"))
    for source, target, mode in volumes:
        args.extend(("--volume", f"{source}:{target}:{mode}"))
    args.extend(("--entrypoint", "bash", image, "-lc", command))
    execute(args, stdout=subprocess.DEVNULL)


def image_id(image: str) -> str:
    return subprocess.check_output(
        ["docker", "image", "inspect", "--format", "{{.Id}}", image], text=True).strip()


def extract_msgpack(output: Path) -> Path:
    target = output / "python-overlay"
    target.mkdir()
    name = PREFIX + "python-source"
    execute(["docker", "create", "--name", name, BOARD], stdout=subprocess.DEVNULL)
    try:
        execute(["docker", "cp", name + ":/usr/lib/python3/dist-packages/msgpack", target])
    finally:
        subprocess.run(["docker", "rm", "--force", name], capture_output=True)
    if not (target / "msgpack/__init__.py").is_file():
        raise RuntimeError("flight_board:v0.1.0 does not provide the msgpack runtime")
    return target


def source_manifest(output: Path, topology: dict) -> None:
    paths = [
        ROOT / "deployment/d2_slam", ROOT / "deployment/swarm/configs/d2_slam.yaml",
        ROOT / "deployment/swarm/orchestrator.py", ROOT / "deployment/swarm/topology.py",
        ROOT / "simulator/airsim/scenarios/d2_slam",
        ROOT / "src/basic_function/state_estimator/d2_slam/plugin",
        ROOT / "src/basic_function/state_estimator/d2_slam/ros1_proxy",
        ROOT / "src/basic_function/state_estimator/d2_slam/ros2_adapter",
    ]
    files = []
    for path in paths:
        files.extend(path.rglob("*") if path.is_dir() else [path])
    hashes = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in files if path.is_file() and "__pycache__" not in path.parts
    }
    upstream = ROOT / "src/basic_function/state_estimator/d2_slam/upstream"
    upstream_diff = subprocess.check_output(["git", "-C", upstream, "diff", "--binary"])
    manifest = {
        "parent_commit": subprocess.check_output(
            ["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip(),
        "parent_dirty": bool(subprocess.check_output(
            ["git", "-C", ROOT, "status", "--porcelain"])),
        "d2_slam_commit": subprocess.check_output(
            ["git", "-C", upstream, "rev-parse", "HEAD"], text=True).strip(),
        "d2_slam_dirty": bool(subprocess.check_output(
            ["git", "-C", upstream, "status", "--porcelain"])),
        "d2_slam_diff_sha256": hashlib.sha256(upstream_diff).hexdigest(),
        "images": {"flight_board": image_id(BOARD), "onboard_computer": image_id(ONBOARD)},
        "workspace": str(WORKSPACE),
        "px4_msgs_overlay": {
            "workspace": str(PX4_WORKSPACE),
            "source_state_sha256": hashlib.sha256(
                (PX4_WORKSPACE / "source-state.env").read_bytes()).hexdigest(),
        },
        "display": os.environ.get("DISPLAY"),
        "render_mode": os.environ.get("AIRSIM_RENDER_MODE"),
        "ros_domain_id": DOMAIN,
        "topology": topology,
        "source_hashes": hashes,
    }
    (output / "source-manifest.json").write_text(json.dumps(manifest, indent=2) + "\n")


def gpu_evidence(vehicles: list[dict]) -> dict:
    raw = subprocess.check_output([
        "nvidia-smi", "--query-compute-apps=pid,process_name,used_memory",
        "--format=csv,noheader,nounits"], text=True)
    applications = []
    for line in raw.splitlines():
        fields = [field.strip() for field in line.split(",")]
        if len(fields) == 3 and fields[0].isdigit():
            applications.append({"pid": int(fields[0]), "process": fields[1],
                                 "used_memory_mib": int(fields[2])})
    per_vehicle = {}
    for vehicle in vehicles:
        name = f"aerohub-d2-slam-{vehicle['name']}"
        output = subprocess.check_output(
            ["docker", "top", name, "-eo", "pid,args"], text=True)
        pids = []
        for line in output.splitlines()[1:]:
            pid, _, command = line.strip().partition(" ")
            if pid.isdigit() and "d2vins_node" in command:
                pids.append(int(pid))
        matches = [entry for entry in applications if entry["pid"] in pids]
        per_vehicle[vehicle["name"]] = {
            "container": name, "d2vins_pids": pids, "gpu_processes": matches,
            "passed": bool(pids) and len(matches) == len(pids),
        }
    return {"nvidia_compute_apps": applications, "vehicles": per_vehicle,
            "passed": all(item["passed"] for item in per_vehicle.values())}


def collect_container(output: Path, name: str) -> None:
    if not container_exists(name):
        return
    with (output / f"{name}.log").open("w") as stream:
        subprocess.run(["docker", "logs", name], stdout=stream, stderr=subprocess.STDOUT)
    with (output / f"{name}.inspect.json").open("w") as stream:
        subprocess.run(["docker", "inspect", name], stdout=stream, stderr=subprocess.STDOUT)


def graph_evidence(output: Path, authority: bool) -> dict:
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source /opt/px4_ws/install/setup.bash && "
        "source /root/aerostack2_ws/install/setup.bash && "
        "source /workspace/ros2/install/setup.bash && "
        "ros2 node list && "
        "ros2 topic info -v /drone0/d2_slam/odometry && "
        "ros2 topic info -v /drone0/d2_slam/landmarks && "
        "ros2 topic info -v /drone0/d2_slam/collaboration && "
        "ros2 topic info -v /drone0/self_localization/pose && "
        "ros2 param get /drone0/state_estimator plugin_name"
    )
    completed = subprocess.run([
        "docker", "run", "--rm", "--network", "host",
        "--env", f"ROS_DOMAIN_ID={DOMAIN}",
        "--volume", f"{WORKSPACE}:/workspace:ro",
        "--entrypoint", "bash", ONBOARD, "-lc", command,
    ], text=True, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    (output / "ros-graph.txt").write_text(completed.stdout)
    result = {
        "returncode": completed.returncode,
        "authority_expected": authority,
        "d2_slam_plugin_reported": "String value is: d2_slam" in completed.stdout,
    }
    result["passed"] = completed.returncode == 0 and (
        result["d2_slam_plugin_reported"] if authority else True)
    (output / "ros-graph.json").write_text(json.dumps(result, indent=2) + "\n")
    return result


def validate_shadow_gate(path: Path, current_manifest: dict) -> dict:
    path = path.resolve()
    result = json.loads((path / "result.json").read_text())
    manifest = json.loads((path / "source-manifest.json").read_text())
    record = {
        "shadow_path": str(path),
        "shadow_passed": result.get("passed") is True,
        "same_d2_slam_commit": manifest.get("d2_slam_commit") == current_manifest["d2_slam_commit"],
        "same_source_hashes": manifest.get("source_hashes") == current_manifest["source_hashes"],
        "same_images": manifest.get("images") == current_manifest["images"],
    }
    record["passed"] = all(value for key, value in record.items() if key != "shadow_path")
    return record


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=(
        "ground", "shadow", "authority", "sensor_fault", "algorithm_fault",
        "communication_fault", "stop", "status"))
    parser.add_argument("--output", type=Path, default=ROOT / "log/d2-slam/ground")
    parser.add_argument("--shadow-evidence", type=Path)
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    config = None
    topology = None
    names = [PREFIX + "clock", PREFIX + "as2", PREFIX + "mission", PREFIX + "probe",
             PREFIX + "python-source"] + [
        PREFIX + f"sensor{index}" for index in range(3)] + [
        PREFIX + f"vehicle{index}" for index in range(3)]
    if args.action == "stop":
        for name in names:
            subprocess.run(["docker", "rm", "--force", name], capture_output=True)
        subprocess.run(manager("stop", output), check=False)
        return 0
    if args.action == "status":
        subprocess.run(manager("status", output), check=False)
        subprocess.run(["docker", "ps", "--filter", f"name={PREFIX}"], check=False)
        subprocess.run(["docker", "ps", "--filter", "label=aerohub.task=d2-slam"], check=False)
        return 0
    if os.environ.get("DISPLAY") != ":1" or os.environ.get("AIRSIM_RENDER_MODE") != "windowed":
        raise RuntimeError("D2SLAM acceptance requires DISPLAY=:1 and AIRSIM_RENDER_MODE=windowed")
    if any(output.iterdir()):
        raise RuntimeError("choose a fresh empty output directory")
    for name in names:
        if container_exists(name):
            raise RuntimeError(f"reserved container exists: {name}")
    for image in (BOARD, ONBOARD):
        execute(["docker", "image", "inspect", image], stdout=subprocess.DEVNULL)
    for artifact in (
        WORKSPACE / "catkin_ws/devel/.private/d2vins/lib/d2vins/d2vins_node",
        WORKSPACE / "ros2/install/setup.bash",
        WORKSPACE / "onnxruntime-linux-x64-gpu-1.18.1/lib/libonnxruntime.so",
        PX4_WORKSPACE / "install/setup.bash",
        PX4_WORKSPACE / "source-state.env"):
        if not artifact.exists():
            raise RuntimeError(f"D2SLAM workspace artifact is missing: {artifact}")
    for catkin_entry in (
            WORKSPACE / "catkin_ws/devel/setup.bash",
            WORKSPACE / "catkin_ws/devel/lib/d2vins/d2vins_node"):
        if not catkin_entry.exists() and not catkin_entry.is_symlink():
            raise RuntimeError(
                f"D2SLAM container-relative catkin entry is missing: {catkin_entry}")

    sys.path.insert(0, str(ROOT))
    from deployment.swarm.orchestrator import build_services
    from deployment.swarm.topology import load_config, render_airsim_settings, resolve_topology
    from deployment.airsim.demos.catora_open_field.sim_startup import launch_vehicles

    config = load_config(CONFIG)
    os.environ["D2_SLAM_WORKSPACE"] = str(WORKSPACE)
    topology = resolve_topology(config)
    scenario = topology["scenarios"][0]
    settings = render_airsim_settings(config)
    (output / "settings.json").write_text(json.dumps(settings, indent=2) + "\n")
    profile = json.loads(PROFILE.read_text())
    profile["settings"] = str(output / "settings.json")
    (output / "profile.json").write_text(json.dumps(profile, indent=2) + "\n")
    fleet = {"vehicles": {
        vehicle["name"]: {"fmu_prefix": "/" + vehicle["name"],
                          "target_system_id": vehicle["px4_system_id"]}
        for vehicle in topology["vehicles"]}}
    (output / "fleet.yaml").write_text(yaml.safe_dump(fleet, sort_keys=False))
    source_manifest(output, topology)
    current_manifest = json.loads((output / "source-manifest.json").read_text())
    msgpack_overlay = extract_msgpack(output) if args.action != "ground" else None
    authority_gate = None
    if args.action == "authority":
        if args.shadow_evidence is None:
            raise RuntimeError("authority requires --shadow-evidence from the same source and images")
        authority_gate = validate_shadow_gate(args.shadow_evidence, current_manifest)
        (output / "authority-gate.json").write_text(json.dumps(authority_gate, indent=2) + "\n")
        if not authority_gate["passed"]:
            raise RuntimeError("authority shadow gate did not pass for the current source and images")

    stop_event = threading.Event()
    services = build_services(config, topology, scenario, output, stop_event)
    relays = []
    result = {"passed": False, "failure": None}
    try:
        execute(manager("start", output) + ["--timeout", "90"])
        state = json.loads((output / "scene/state.json").read_text())
        window = subprocess.check_output(
            ["bash", "-lc", "DISPLAY=${DISPLAY:?} xwininfo -root -tree | grep -i AirSimNH"],
            text=True)
        (output / "window-evidence.txt").write_text(
            "command=" + json.dumps(state["command"]) + "\n" + window)
        relays = launch_vehicles(
            BOARD, ROOT / "simulator/airsim/scenarios/d2_slam", output, PREFIX, DDS,
            sensors=False, settings_path=output / "settings.json", vehicles=topology["vehicles"],
            ros_domain_id=DOMAIN, rpc_port=42471, relay_port_base=4780)
        execute([
            "docker", "run", "--detach", "--init", "--network", "host",
            "--name", PREFIX + "clock", "--env", f"ROS_DOMAIN_ID={DOMAIN}",
            "--env", f"VEHICLE_NAME={topology['vehicles'][0]['airsim_name']}",
            "--env", "AIRSIM_RPC_PORT=42471", BOARD, "clock"],
            stdout=subprocess.DEVNULL)
        for vehicle in topology["vehicles"]:
            index = vehicle["index"]
            command = (
                "source /opt/ros/humble/setup.bash && "
                "source /opt/platform_sensors/install/setup.bash && "
                "source /workspace/ros2/install/setup.bash && "
                "exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args "
                "-p use_sim_time:=true -p rpc_port:=42471 "
                f"-p vehicle_name:={vehicle['airsim_name']} "
                f"-p vehicle_namespace:={vehicle['name']} "
                "-p settings_file:=/config/settings.json "
                "-p camera_rate_hz:=10.0 -p imu_rate_hz:=50.0 "
                f"-r /tf_static:=/{vehicle['name']}/tf_static")
            run_detached(
                PREFIX + f"sensor{index}", BOARD,
                {"ROS_DOMAIN_ID": str(DOMAIN),
                 "FASTRTPS_DEFAULT_PROFILES_FILE": "/run/dds.xml"},
                [(DDS, "/run/dds.xml", "ro"), (WORKSPACE, "/workspace", "ro"),
                 (output / "settings.json", "/config/settings.json", "ro")], command)
        as2_environment = {
            "ROS_DOMAIN_ID": str(DOMAIN), "AS2_FLEET_FILE": "/results/fleet.yaml",
            "AS2_RUNTIME_DIR": "/results/as2-runtime", "AS2_TF_SCOPE": "per-vehicle",
        }
        if args.action == "authority":
            as2_environment.update({
                "AS2_STATE_ESTIMATOR_PLUGIN": "d2_slam",
                "AS2_STATE_ESTIMATOR_LAUNCH_PACKAGE": "uav_d2_slam",
                "AS2_STATE_ESTIMATOR_LAUNCH_FILE": "d2_slam.launch.py",
                "AS2_STATE_ESTIMATOR_CONFIG_FILE":
                    "/workspace/ros2/install/share/uav_d2_slam/config/estimator.yaml",
            })
        run_detached(
            PREFIX + "as2", ONBOARD,
            as2_environment,
            [(output, "/results", "rw"), (WORKSPACE, "/workspace", "ro"),
             (AS2_RUN, "/opt/aerohub/run_as2.py", "ro"),
             (ROOT / "deployment/airsim/as2/scoped_tf.launch.py",
              "/opt/aerohub/scoped_tf.launch.py", "ro"),
             (AS2_CONFIG, "/config/config.yaml", "ro"),
             (AS2_CONTROLLER, "/config/pid.yaml", "ro")],
            "source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && "
            "source /root/aerostack2_ws/install/setup.bash && "
            "source /workspace/ros2/install/setup.bash && "
            "AS2_CONFIG_FILE=/config/config.yaml AS2_CONTROLLER_CONFIG_FILE=/config/pid.yaml "
            "exec python3 /opt/aerohub/run_as2.py")
        for service in services:
            service.start()
        deadline = time.monotonic() + float(config["validation"]["startup_timeout_s"])
        for service in services:
            if not service.ready.wait(max(0.0, deadline - time.monotonic())):
                raise RuntimeError(f"compute service startup timed out: {service.identity}")
        time.sleep(15)
        failures = [service.record() for service in services if service.error]
        if failures:
            raise RuntimeError(f"compute service failed during startup: {failures}")
        graph = None
        while time.monotonic() < deadline:
            graph = graph_evidence(output, args.action == "authority")
            if graph["passed"]:
                break
            failures = [service.record() for service in services if service.error]
            if failures:
                raise RuntimeError(f"compute service failed while waiting for ROS graph: {failures}")
            time.sleep(2)
        if graph is None or not graph["passed"]:
            raise RuntimeError("ROS graph or state-estimator plugin readiness timed out")
        gpu = gpu_evidence(topology["vehicles"])
        (output / "gpu-processes.json").write_text(json.dumps(gpu, indent=2) + "\n")
        if not gpu["passed"]:
            raise RuntimeError("not every D2VINS process is executing on the NVIDIA GPU")
        if args.action == "ground":
            probe_command = [
                "docker", "run", "--rm", "--init", "--network", "host",
                "--name", PREFIX + "probe", "--env", f"ROS_DOMAIN_ID={DOMAIN}",
                "--env", "D2_PROBE_OUTPUT=/results/ground-probe.json",
                "--volume", f"{output}:/results:rw", "--volume", f"{PROBE}:/probe.py:ro",
                "--entrypoint", "bash", ONBOARD, "-lc",
                "source /opt/ros/humble/setup.bash && "
                "source /root/aerostack2_ws/install/setup.bash && exec python3 /probe.py"]
            execute(probe_command)
            probe = json.loads((output / "ground-probe.json").read_text())
            result = {"schema_version": 1, "scenario": "ground", "probe": probe,
                      "gpu": gpu, "graph": graph,
                      "services": [service.record() for service in services],
                      "passed": probe["passed"] and gpu["passed"] and graph["passed"],
                      "failure": None}
        else:
            injector_errors = []

            def inject_fault():
                marker = output / "fault-ready.json"
                deadline = time.monotonic() + 180
                while not marker.exists() and time.monotonic() < deadline:
                    time.sleep(0.1)
                if not marker.exists():
                    injector_errors.append("timed out waiting for fault-ready.json")
                    return
                try:
                    if args.action == "sensor_fault":
                        execute(["docker", "stop", "--timeout", "3", PREFIX + "sensor1"])
                        injection = {"type": "sensor_container_stop", "target": PREFIX + "sensor1"}
                    elif args.action == "algorithm_fault":
                        execute(["docker", "exec", "aerohub-d2-slam-drone1", "bash", "-lc",
                                 "kill -KILL $(pgrep -x d2vins_node)"])
                        injection = {"type": "d2vins_sigkill", "target": "drone1"}
                    elif args.action == "communication_fault":
                        for vehicle in ("drone0", "drone2"):
                            execute(["docker", "exec", f"aerohub-d2-slam-{vehicle}",
                                     "bash", "-lc", "kill -STOP $(pgrep -x d2vins_node)"])
                        injection = {"type": "peer_d2vins_sigstop",
                                     "paused_peers": ["drone0", "drone2"],
                                     "observer": "drone1"}
                    else:
                        return
                    injection["injected_wall_time"] = time.time()
                    (output / "fault-injected.json").write_text(
                        json.dumps(injection, indent=2) + "\n")
                except BaseException as exc:
                    injector_errors.append(f"{type(exc).__name__}: {exc}")

            injector = None
            if args.action.endswith("fault"):
                injector = threading.Thread(target=inject_fault, name="d2-fault-injector")
                injector.start()
            mission_command = [
                "docker", "run", "--rm", "--init", "--network", "host",
                "--name", PREFIX + "mission", "--env", f"ROS_DOMAIN_ID={DOMAIN}",
                "--env", f"D2_VALIDATION_SCENARIO={args.action}",
                "--env", "D2_VALIDATION_OUTPUT=/results", "--env", "AIRSIM_RPC_PORT=42471",
                "--volume", f"{output}:/results:rw",
                "--volume", f"{MISSION}:/opt/d2/flight_validation.py:ro",
                "--volume", f"{VALIDATION}:/config/validation.yaml:ro",
                "--volume", f"{AIRSIM_RPC}:/opt/d2/airsim_rpc.py:ro",
                "--volume", f"{msgpack_overlay}:/opt/flight-board-python:ro",
                "--volume", f"{PX4_WORKSPACE}:/px4-overlay:ro",
                "--entrypoint", "bash", ONBOARD, "-lc",
                "source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && "
                "source /px4-overlay/install/setup.bash && "
                "source /root/aerostack2_ws/install/setup.bash && "
                "source /workspace/ros2/install/setup.bash && "
                "cd /opt/d2 && PYTHONPATH=/opt/d2:/opt/flight-board-python:${PYTHONPATH:-} "
                "exec python3 flight_validation.py",
            ]
            mission_command[mission_command.index("--entrypoint"):mission_command.index("--entrypoint")] = [
                "--volume", f"{WORKSPACE}:/workspace:ro"]
            with (output / "mission-console.log").open("w") as stream:
                execute(mission_command, stdout=stream, stderr=subprocess.STDOUT)
            if injector is not None:
                injector.join(timeout=30)
                if injector.is_alive():
                    injector_errors.append("fault injector did not terminate")
            if injector_errors:
                raise RuntimeError("; ".join(injector_errors))
            validation = json.loads((output / "validation-result.json").read_text())
            overhead = json.loads((output / "scene/overhead-view-state.json").read_text())
            overhead_passed = (
                float(overhead["max_centroid_displacement_m"])
                >= float(yaml.safe_load(VALIDATION.read_text())["gates"][
                    "overhead_centroid_displacement_min_m"]))
            (output / "overhead-validation.json").write_text(json.dumps({
                "max_centroid_displacement_m": overhead["max_centroid_displacement_m"],
                "passed": overhead_passed,
            }, indent=2) + "\n")
            result = {
                "schema_version": 1, "scenario": args.action,
                "validation": validation, "gpu": gpu, "graph": graph,
                "authority_gate": authority_gate,
                "services": [service.record() for service in services],
                "overhead": overhead, "overhead_passed": overhead_passed,
                "passed": validation["passed"] and gpu["passed"] and graph["passed"]
                          and overhead_passed,
                "failure": None,
            }
    except BaseException as exc:
        result["failure"] = f"{type(exc).__name__}: {exc}"
    finally:
        stop_event.set()
        for service in services:
            if service.ident is not None:
                service.join(timeout=12)
        for name in names:
            collect_container(output, name)
            subprocess.run(["docker", "rm", "--force", name], capture_output=True)
        subprocess.run(["docker", "rm", "--force", PREFIX + "probe"], capture_output=True)
        subprocess.run(manager("stop", output), check=False)
        for relay in relays:
            relay.close()
        remaining = subprocess.check_output(
            ["docker", "ps", "-a", "--format", "{{.Names}}"], text=True).splitlines()
        result["cleanup"] = {
            "remaining_owned_containers": [name for name in remaining
                                            if name.startswith(PREFIX) or
                                            name.startswith("aerohub-d2-slam-")],
            "service_threads_alive": [service.identity for service in services
                                      if service.is_alive()],
        }
        result["passed"] = bool(result.get("passed")) and not any(result["cleanup"].values())
        (output / "result.json").write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
