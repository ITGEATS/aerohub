# PIN-SLAM deployment

PIN-SLAM runs as a native ROS 2 Humble Python node in a self-contained GPU
image derived from `onboard_computer:v0.1.2`. The image contains the locked
upstream source, complete hash-locked Python environment, AeroHub Adapter, AS2
plugin, sensor adapter, configuration, and validation programs. No source,
workspace, ROS overlay, or ROS 1 runtime is mounted when the component runs.

## Prepare and build

Prepare the ignored wheelhouse once, then build the image. These files are
build inputs only and are copied into image layers.

```bash
PIN_SLAM_WORKSPACE="$PWD/log/workspaces/pin_slam" \
  bash deployment/pin_slam/download_wheels.sh

PIN_SLAM_WHEELHOUSE="$PWD/log/workspaces/pin_slam/wheelhouse" \
ONBOARD_COMPUTER_IMAGE=onboard_computer:v0.1.2 \
PIN_SLAM_IMAGE=aerohub/pin-slam:ros2-humble \
  bash deployment/pin_slam/prepare_workspace.sh
```

The build verifies every wheel hash, the Humble package prefixes, absence of
Noetic/catkin output, and CUDA visibility in the final image.

## Runtime contract

The node consumes `/drone0/sensor_measurements/Lidar/points`, preserves its
sample time, and publishes `/drone0/pin_slam/odometry`, `/path`,
`/neural_points`, and `/status`. The neural-point topic is a bounded RViz view
of the native implicit map, not a collision map. PX4 local odometry remains the
high-rate local control source; PIN-SLAM supplies the global correction.

For a no-input GPU smoke without mounts:

```bash
docker run --rm --gpus all --network host -e ROS_DOMAIN_ID=181 \
  aerohub/pin-slam:ros2-humble
```

## Visible AirSim acceptance

```bash
export DISPLAY=:1
export AIRSIM_RENDER_MODE=windowed

python3 deployment/pin_slam/run_airsim.py run --scenario shadow
python3 deployment/pin_slam/run_airsim.py run --scenario authority
python3 deployment/pin_slam/run_airsim.py run --scenario sensor_fault
python3 deployment/pin_slam/run_airsim.py run --scenario algorithm_fault

python3 deployment/pin_slam/run_airsim.py status
python3 deployment/pin_slam/run_airsim.py stop
```

The runner uses `flight_board:v0.1.0` for PX4 and the component image for all
compute processes. It mounts only generated scenario/evidence output. The
complete matrix passed on image
`sha256:a276960113485a124ed5fc95ba47d7f457a0ae0d73cd1d9c74c1179bf130145b`;
`component.yaml` lists the accepted evidence. Physical-board and real-flight
deployment remain unvalidated.
