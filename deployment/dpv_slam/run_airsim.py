#!/usr/bin/env python3
"""Own one visible AirSim/PX4/DPV-SLAM/AS2 acceptance session."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import socket
import subprocess
import sys
import threading
import time

import yaml


ROOT = Path(__file__).resolve().parents[2]
PREFIX = "aerohub-dpv-slam-"
CONTAINERS = [PREFIX + name for name in (
    "clock", "vehicle", "sensors", "truth", "algorithm", "as2",
    "freshness", "mission")]
BOARD = "flight_board:v0.1.0"
DPV_IMAGE = os.environ.get("DPV_SLAM_IMAGE", "aerohub/dpv-slam:ros2-humble")
DOMAIN = 82

SETTINGS = ROOT / "deployment/dpv_slam/airsim-settings.json"
PROFILE = ROOT / "deployment/dpv_slam/airsim-profile.json"
UPSTREAM_ROOT = ROOT / "src/basic_function/state_estimator/dpv_slam/upstream"


def execute(command, **kwargs):
    return subprocess.run(list(map(str, command)), check=True, **kwargs)


def manager(action, output):
    return [sys.executable, ROOT / "simulator/manager/manager.py", action,
            "--profile", PROFILE, "--run-dir", output / "scene"]


def container_exists(name):
    return subprocess.run(
        ["docker", "container", "inspect", name], stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL).returncode == 0


def run_detached(name, image, environment, volumes, command, gpu=False):
    args = ["docker", "run", "--detach", "--init", "--network", "host",
            "--name", name]
    if gpu:
        args += ["--runtime", "nvidia", "--env", "NVIDIA_VISIBLE_DEVICES=0",
                 "--env", "NVIDIA_DRIVER_CAPABILITIES=compute,utility"]
    for key, value in environment.items():
        args += ["--env", f"{key}={value}"]
    for source, target, mode in volumes:
        args += ["--volume", f"{source}:{target}:{mode}"]
    args += ["--entrypoint", "bash", image, "-lc", command]
    execute(args, stdout=subprocess.DEVNULL)


def cleanup(output):
    for name in reversed(CONTAINERS):
        if not container_exists(name):
            continue
        subprocess.run(["docker", "stop", "--timeout", "10", name],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        with (output / f"{name}.log").open("w") as stream:
            subprocess.run(["docker", "logs", name], stdout=stream,
                           stderr=subprocess.STDOUT)
        with (output / f"{name}.inspect.json").open("w") as stream:
            subprocess.run(["docker", "inspect", name], stdout=stream,
                           stderr=subprocess.STDOUT)
        subprocess.run(["docker", "rm", "--force", name],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(manager("stop", output), check=False, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)


def port_available(port):
    try:
        with socket.socket() as sock:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(("127.0.0.1", port))
        return True
    except OSError:
        return False


def write_cleanup_evidence(output):
    evidence = {
        "containers_remaining": [
            name for name in CONTAINERS if container_exists(name)],
        "ports_available": {
            str(port): port_available(port)
            for port in (45471, 5080, 8891)
        },
        "wall_time": time.time(),
    }
    evidence["success"] = (
        not evidence["containers_remaining"]
        and all(evidence["ports_available"].values()))
    (output / "cleanup.json").write_text(json.dumps(evidence, indent=2) + "\n")


def validate_inputs():
    required = {
        "settings": SETTINGS,
        "profile": PROFILE,
        "dpvo_source": UPSTREAM_ROOT / "dpvo/dpvo.py",
    }
    for label, path in required.items():
        if not path.exists():
            raise RuntimeError(f"{label} artifact is missing: {path}")


def source_manifest(output, scenario):
    paths = [ROOT / "deployment/dpv_slam", ROOT / "deployment/flight_board",
             ROOT / "deployment/airsim/as2", ROOT / "src/basic_function/state_estimator/dpv_slam",
             ROOT / "simulator/airsim/sensors"]
    files = []
    for path in paths:
        files.extend(item for item in path.rglob("*") if item.is_file())
    hashes = {
        str(path.relative_to(ROOT)): hashlib.sha256(path.read_bytes()).hexdigest()
        for path in files if "__pycache__" not in path.parts
    }
    manifest = {
        "scenario": scenario,
        "commit": subprocess.check_output(
            ["git", "-C", ROOT, "rev-parse", "HEAD"], text=True).strip(),
        "dpvo_commit": subprocess.check_output(
            ["git", "-C", UPSTREAM_ROOT,
             "rev-parse", "HEAD"], text=True).strip(),
        "dirty": bool(subprocess.check_output(
            ["git", "-C", ROOT, "status", "--porcelain"])),
        "render": {"display": os.environ.get("DISPLAY"),
                   "mode": os.environ.get("AIRSIM_RENDER_MODE")},
        "ros_domain": DOMAIN,
        "ports": {"airsim_rpc": 45471, "px4_tcp": 5080,
                  "agent": 8891, "system_id": 21},
        "runtime_policy": "self-contained image; no source or workspace mounts",
        "source_hashes": hashes,
    }
    (output / "source-manifest.json").write_text(
        json.dumps(manifest, indent=2) + "\n")
    with (output / "dirty.patch").open("w") as stream:
        subprocess.run(["git", "-C", ROOT, "diff", "--binary"], stdout=stream,
                       check=True)


def wait_px4(output):
    deadline = time.monotonic() + 60
    while time.monotonic() < deadline:
        log = subprocess.check_output(
            ["docker", "logs", PREFIX + "vehicle"], stderr=subprocess.STDOUT,
            text=True)
        if "Ready for takeoff!" in log:
            (output / "px4-startup.json").write_text(json.dumps({
                "ready": True,
                "firmware_commit": "61907962c9220437ec01a98fa26bfbf5cbb254b5",
                "source": "flight_board:v0.1.0",
            }, indent=2) + "\n")
            return
        time.sleep(0.25)
    raise RuntimeError("PX4 did not become ready")


def graph_evidence(output):
    command = (
        "source /opt/ros/humble/setup.bash && "
        "source /opt/px4_ws/install/setup.bash && "
        "source /root/aerostack2_ws/install/setup.bash && "
        "source /opt/dpv_ws/install/setup.bash && "
        "ros2 node list; ros2 topic info -v "
        "/drone0/sensor_measurements/FrontCamera/rgb/image; "
        "ros2 topic info -v /drone0/dpv_slam/odometry; "
        "ros2 topic info -v /drone0/self_localization/pose")
    base = [
        "docker", "run", "--rm", "--network", "host",
        "--env", f"ROS_DOMAIN_ID={DOMAIN}",
        "--entrypoint", "bash", DPV_IMAGE, "-lc", command,
    ]
    deadline = time.monotonic() + 60
    attempts = []
    while True:
        completed = subprocess.run(base, text=True, capture_output=True)
        text = completed.stdout + "\nSTDERR:\n" + completed.stderr
        attempts.append(text)
        ready = (
            completed.returncode == 0
            and "/drone0/dpv_slam" in completed.stdout
            and "Subscription count: 0" not in completed.stdout.split(
                "Type: nav_msgs/msg/Odometry", 1)[0]
            and "Unknown topic" not in completed.stderr
        )
        if ready:
            (output / "ros-graph.txt").write_text(text)
            (output / "ros-graph-attempts.txt").write_text(
                "\n\n===== ATTEMPT =====\n".join(attempts))
            return
        if time.monotonic() >= deadline:
            (output / "ros-graph-attempts.txt").write_text(
                "\n\n===== ATTEMPT =====\n".join(attempts))
            raise RuntimeError("ROS graph did not become complete")
        time.sleep(2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("run", "stop", "status"))
    parser.add_argument("--scenario", choices=(
        "shadow", "authority", "sensor_fault", "algorithm_fault"),
        default="shadow")
    parser.add_argument("--output", type=Path,
                        default=ROOT / "log/dpv_slam/airsim-acceptance")
    args = parser.parse_args()
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    if args.action == "stop":
        cleanup(output)
        return
    if args.action == "status":
        subprocess.run(manager("status", output), check=False)
        execute(["docker", "ps", "--filter", "name=" + PREFIX])
        return
    if os.environ.get("DISPLAY") != ":1" or os.environ.get(
            "AIRSIM_RENDER_MODE") != "windowed":
        raise RuntimeError("acceptance requires DISPLAY=:1 and windowed rendering")
    if any((output / name).exists() for name in (
            "result.json", "failure.json", "gate-report.json")):
        raise RuntimeError("choose a fresh output directory")
    if any(container_exists(name) for name in CONTAINERS):
        raise RuntimeError("a reserved DPV-SLAM container already exists")
    for image in (BOARD, DPV_IMAGE):
        execute(["docker", "image", "inspect", image], stdout=subprocess.DEVNULL)
    validate_inputs()
    source_manifest(output, args.scenario)
    (output / "fleet.yaml").write_text(yaml.safe_dump({"vehicles": {
        "drone0": {"fmu_prefix": "/drone0", "target_system_id": 21}}}))
    images = {
        label: subprocess.check_output(
            ["docker", "image", "inspect", "--format", "{{.Id}}", image],
            text=True).strip()
        for label, image in (("flight_board", BOARD), ("dpv_slam", DPV_IMAGE))
    }
    (output / "images.json").write_text(json.dumps(images, indent=2) + "\n")
    try:
        execute(manager("start", output) + ["--timeout", "90"])
        scene = json.loads((output / "scene/state.json").read_text())
        environment = Path(f"/proc/{scene['pid']}/environ").read_bytes().replace(
            b"\0", b"\n").decode(errors="replace")
        window = subprocess.check_output(
            ["bash", "-lc", "DISPLAY=:1 xwininfo -root -tree | grep -i AirSimNH"],
            text=True)
        (output / "window-evidence.txt").write_text(
            "command=" + json.dumps(scene["command"]) + "\n" + environment + window)
        runtime_settings = output / "scene/settings.json"

        common = {"ROS_DOMAIN_ID": str(DOMAIN)}
        clock_env = dict(common, AIRSIM_RPC_PORT="45471", VEHICLE_NAME="PX4")
        run_detached(PREFIX + "clock", BOARD, clock_env, [],
                     "source /opt/ros/humble/setup.bash && "
                     "source /opt/platform_sensors/install/setup.bash && "
                     "exec python3 /opt/flight_board/supervisor.py clock")
        vehicle_env = dict(common, VEHICLE_NAME="PX4", VEHICLE_NAMESPACE="drone0",
                           PX4_INSTANCE="20", PX4_SYSTEM_ID="21",
                           XRCE_AGENT_PORT="8891", AIRSIM_RPC_PORT="45471",
                           AIRSIM_SETTINGS="/config/settings.json",
                           FLIGHT_BOARD_SENSORS="0")
        run_detached(PREFIX + "vehicle", BOARD, vehicle_env,
                     [(runtime_settings, "/config/settings.json", "ro")],
                     "source /opt/ros/humble/setup.bash && "
                     "source /opt/platform_sensors/install/setup.bash && "
                     "exec python3 /opt/flight_board/supervisor.py vehicle")
        wait_px4(output)

        sensor_command = (
            "source /opt/ros/humble/setup.bash && source /opt/dpv_ws/install/setup.bash && "
            "exec ros2 run uav_as2_sensor_adapter airsim_sensor_node --ros-args "
            "-p use_sim_time:=true -p settings_file:=/config/settings.json "
            "-p vehicle_name:=PX4 -p vehicle_namespace:=drone0 "
            "-p rpc_port:=45471 -p camera_rate_hz:=15.0 "
            "-p camera_parallelism:=4 -p imu_rate_hz:=100.0 -p lidar_rate_hz:=10.0")
        run_detached(PREFIX + "sensors", DPV_IMAGE, common,
                     [(runtime_settings, "/config/settings.json", "ro")], sensor_command)
        truth_command = (
            "source /opt/ros/humble/setup.bash && source /opt/dpv_ws/install/setup.bash && "
            "exec python3 /opt/dpv/runtime/airsim_truth_bridge.py --ros-args "
            "-p use_sim_time:=true -p rpc_port:=45471")
        run_detached(PREFIX + "truth", DPV_IMAGE, common, [], truth_command)

        algorithm_command = (
            "export LD_LIBRARY_PATH=$(find /opt/dpv/python/nvidia -type d -name lib -print | paste -sd:):"
            "/opt/dpv/cuda/lib:/opt/dpv/cuda/lib64:${LD_LIBRARY_PATH:-} && "
            "source /opt/ros/humble/setup.bash && source /opt/dpv_ws/install/setup.bash && "
            "exec ros2 run uav_dpv_slam_ros2 dpv_slam_node --ros-args "
            "-r __ns:=/drone0 --params-file /opt/dpv/config/algorithm.yaml")
        run_detached(PREFIX + "algorithm", DPV_IMAGE, common,
                     [(output, "/results", "rw")],
                     algorithm_command, gpu=True)

        as2_env = dict(common, AS2_FLEET_FILE="/results/fleet.yaml",
                       AS2_EXTERNAL_ODOM="0",
                       AS2_CONFIG_FILE="/opt/dpv/runtime/as2_config.yaml",
                       AS2_CONTROLLER_CONFIG_FILE="/opt/dpv/runtime/pid.yaml")
        if args.scenario != "shadow":
            as2_env.update({
                "AS2_STATE_ESTIMATOR_PLUGIN": "dpv_slam",
                "AS2_STATE_ESTIMATOR_CONFIG_FILE": "/opt/dpv/config/estimator.yaml",
                "AS2_STATE_ESTIMATOR_LAUNCH_PACKAGE": "uav_dpv_slam",
                "AS2_STATE_ESTIMATOR_LAUNCH_FILE": "dpv_slam.launch.py",
            })
        as2_command = (
            "source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && "
            "source /root/aerostack2_ws/install/setup.bash && "
            "source /opt/dpv_ws/install/setup.bash && "
            "exec python3 /opt/dpv/runtime/run_as2.py")
        run_detached(PREFIX + "as2", DPV_IMAGE, as2_env,
                     [(output, "/results", "rw")], as2_command)

        if args.scenario != "shadow":
            freshness_command = (
                "source /opt/ros/humble/setup.bash && "
                "source /root/aerostack2_ws/install/setup.bash && "
                "source /opt/dpv_ws/install/setup.bash && "
                "exec python3 /opt/dpv/runtime/canonical_freshness.py --namespace drone0 "
                "--output /results/canonical-freshness.json")
            run_detached(PREFIX + "freshness", DPV_IMAGE, common,
                         [(output, "/results", "rw")],
                         freshness_command)

        time.sleep(8)
        graph_evidence(output)
        mission_environment = dict(common, DPV_SLAM_SHADOW_RESULTS="/results",
                                   DPV_SLAM_SCENARIO=args.scenario)
        mission_command = (
            "source /opt/ros/humble/setup.bash && source /opt/px4_ws/install/setup.bash && "
            "source /root/aerostack2_ws/install/setup.bash && "
            "source /opt/dpv_ws/install/setup.bash && "
            "exec python3 /opt/dpv/runtime/shadow_validation.py")
        mission_args = ["docker", "run", "--init", "--network", "host",
                        "--name", PREFIX + "mission"]
        for key, value in mission_environment.items():
            mission_args += ["--env", f"{key}={value}"]
        for source, target, mode in ((output, "/results", "rw"),):
            mission_args += ["--volume", f"{source}:{target}:{mode}"]
        mission_args += ["--entrypoint", "bash", DPV_IMAGE, "-lc", mission_command]

        injector_errors = []
        injector = None
        if args.scenario in ("sensor_fault", "algorithm_fault"):
            target = PREFIX + ("sensors" if args.scenario == "sensor_fault" else "algorithm")

            def inject_fault_inner():
                marker = output / "fault-ready.json"
                deadline = time.monotonic() + 180
                while not marker.is_file() and time.monotonic() < deadline:
                    time.sleep(0.1)
                if not marker.is_file():
                    injector_errors.append("fault-ready marker timeout")
                    return
                completed = subprocess.run(
                    ["docker", "stop", "--timeout", "1", target],
                    text=True, capture_output=True)
                (output / "fault-injection.json").write_text(json.dumps({
                    "scenario": args.scenario, "target_container": target,
                    "returncode": completed.returncode, "stdout": completed.stdout,
                    "stderr": completed.stderr, "wall_time": time.time(),
                }, indent=2) + "\n")
                if completed.returncode:
                    injector_errors.append(
                        f"docker stop failed for {target}: "
                        f"returncode={completed.returncode}, "
                        f"stderr={completed.stderr.strip()!r}")

            def inject_fault():
                try:
                    inject_fault_inner()
                except BaseException as error:
                    injector_errors.append(repr(error))
                    (output / "fault-injection-error.json").write_text(json.dumps({
                        "scenario": args.scenario,
                        "target_container": target,
                        "error": repr(error),
                        "wall_time": time.time(),
                    }, indent=2) + "\n")

            injector = threading.Thread(target=inject_fault, daemon=True)
            injector.start()
        with (output / "mission-console.log").open("w") as stream:
            execute(mission_args, stdout=stream, stderr=subprocess.STDOUT)
        if injector:
            injector.join(10)
            if injector.is_alive():
                injector_errors.append(
                    "fault injector did not finish within 10 seconds after mission exit")
            if injector_errors:
                raise RuntimeError("fault injector failed: " + "; ".join(injector_errors))
        if container_exists(PREFIX + "freshness"):
            execute(["docker", "stop", "--timeout", "10", PREFIX + "freshness"],
                    stdout=subprocess.DEVNULL)
        gate_command = [
            "docker", "run", "--rm", "--volume", f"{output}:/results:rw",
            "--entrypoint", "python3", DPV_IMAGE,
            "/opt/dpv/runtime/gate_result.py", "/opt/dpv/runtime/validation.yaml",
            "/results/result.json", "--output", "/results/gate-report.json"]
        if args.scenario == "authority":
            gate_command += ["--authority", "--authority-freshness",
                             "/results/canonical-freshness.json"]
        if args.scenario in ("shadow", "authority"):
            execute(gate_command)
    finally:
        cleanup(output)
        write_cleanup_evidence(output)


if __name__ == "__main__":
    main()
