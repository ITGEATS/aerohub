# FAST-LIVO2 deployment

This component runs the locked native ROS 2 Humble FAST-LIVO2 implementation,
the AeroHub input/output adapters, and the AS2 plugin from one self-contained
image. ROS 1, a Proxy, and host source/build overlays are not part of the
supported PC reproduction or delivery topology.

## Build

Prerequisites are `onboard_computer:v0.1.2` and `flight_board:v0.1.0`.
External downloads on the build host must use its local proxy configuration;
do not store proxy addresses in the repository.

```bash
ONBOARD_COMPUTER_IMAGE=onboard_computer:v0.1.2 \
FAST_LIVO2_IMAGE=aerohub/fast-livo2:ros2-humble \
  bash deployment/fast_livo2/prepare_workspace.sh
```

Despite its compatibility name, `prepare_workspace.sh` builds and verifies an
image. It does not create a runtime workspace mount. The image contains the
locked FAST-LIVO2 source, Sophus/vikit, ROS 2 adapters, AS2 plugin, sensor
adapter, configuration, and acceptance helpers.

## Runtime contract

The algorithm consumes AirSim IMU, RGB, and instantaneous XYZ LiDAR data under
`/drone0/sensor_measurements`. It publishes candidate odometry on
`/drone0/fast_livo2/odometry` and registered points on
`/drone0/perception/fast_livo2/registered_points`. The registered points are a
visualizable point set, not an occupancy map or ESDF.

The locked ROS 2 implementation is CPU/OpenMP and has no CUDA execution path.
AirSim does not provide LiDAR per-point time, so this configuration does not
claim rotating-scan deskew equivalence.

For a no-input process smoke without mounts:

```bash
docker run --rm --network host -e ROS_DOMAIN_ID=219 \
  aerohub/fast-livo2:ros2-humble
```

## Visible AirSim acceptance

Each command owns its containers and writes evidence below `log/`. The only
runtime bind mount is the selected evidence directory. PX4 comes from
`flight_board:v0.1.0`; repository PX4 source is not mounted.

```bash
export DISPLAY=:1
export AIRSIM_RENDER_MODE=windowed

python3 deployment/fast_livo2/run_airsim.py run --scenario shadow
python3 deployment/fast_livo2/run_airsim.py run --scenario authority
python3 deployment/fast_livo2/run_airsim.py run --scenario sensor_fault
python3 deployment/fast_livo2/run_airsim.py run --scenario algorithm_fault

python3 deployment/fast_livo2/run_airsim.py status
python3 deployment/fast_livo2/run_airsim.py stop
```

`shadow` must pass the predeclared accuracy gate before `authority` may select
FAST-LIVO2 as the sole AS2 state/TF provider. A container smoke or nonempty
point cloud is not a reproduction-success result. The current component status
is `reproduced`: the full matrix passed on image
`sha256:a97bfa6cc0464a6b3bc3596e92bf03f7ba0c94e72fa7df3097bae8a614613a62`.
`component.yaml` lists the accepted evidence. Physical-board and real-flight
deployment remain unvalidated.
