# DPV-SLAM deployment

DPV-SLAM runs the locked DPVO source, its three CUDA extensions, the native ROS
2 Humble Adapter, and the AS2 plugin from one component image derived from
`onboard_computer:v0.1.2`. Runtime source/workspace/overlay mounts and ROS 1
processes are not supported.

## Prepare and build

`prepare_models.sh` verifies the official model and downloads the complete
Python/CUDA artifact set into the ignored artifact directory using tracked URLs
and SHA256 values. `prepare_workspace.sh` consumes those artifacts only during
`docker build`; they are not runtime mounts.

```bash
DPV_SLAM_ARTIFACT_ROOT="$PWD/log/workspaces/dpv_slam" \
  bash deployment/dpv_slam/prepare_models.sh

DPV_SLAM_ARTIFACT_ROOT="$PWD/log/workspaces/dpv_slam" \
ONBOARD_COMPUTER_IMAGE=onboard_computer:v0.1.2 \
DPV_SLAM_IMAGE=aerohub/dpv-slam:ros2-humble \
  bash deployment/dpv_slam/prepare_workspace.sh
```

The build checks all Python and CUDA artifact hashes, compiles `cuda_corr`,
`cuda_ba`, and `lietorch_backends` for the declared CUDA architecture, builds
the Humble packages, and verifies imports plus CUDA in the final image.

## Runtime contract

The node consumes synchronized RGB and CameraInfo plus time-paired PX4 local
odometry under `/drone0/sensor_measurements`. It publishes
`/drone0/dpv_slam/odometry`, `/path`, `/sparse_points`, and `/status`. Metric
scale is frozen only after the configured motion baseline and fit gate pass;
the sparse points are visual landmarks, not an occupancy representation.

For a no-input GPU smoke without mounts:

```bash
docker run --rm --gpus all --network host -e ROS_DOMAIN_ID=182 \
  aerohub/dpv-slam:ros2-humble
```

## Visible AirSim acceptance

```bash
export DISPLAY=:1
export AIRSIM_RENDER_MODE=windowed

python3 deployment/dpv_slam/run_airsim.py run --scenario shadow
python3 deployment/dpv_slam/run_airsim.py run --scenario authority
python3 deployment/dpv_slam/run_airsim.py run --scenario sensor_fault
python3 deployment/dpv_slam/run_airsim.py run --scenario algorithm_fault

python3 deployment/dpv_slam/run_airsim.py status
python3 deployment/dpv_slam/run_airsim.py stop
```

PX4 is supplied by `flight_board:v0.1.0`; compute processes use only the
component image. The runner mounts generated evidence output only. The matrix
passed on image
`sha256:5341db3f2f08f7d5376bddcf7288d7b843900dcdbf0cfbc9e1d4cae1b618c783`;
the accepted evidence paths are listed in `component.yaml`. Physical-board and
real-flight deployment remain unvalidated.
