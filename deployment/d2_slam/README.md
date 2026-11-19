# D2SLAM deployment

This deployment packages the AeroHub D2SLAM Fork as native ROS 2 Humble.
Every PC-simulation vehicle runs one self-contained compute image; no ROS 1
runtime, catkin workspace, source checkout, build overlay, or host dependency
directory is mounted at runtime.

## Runtime architecture

```text
AirSim RGB + DepthPerspective + IMU (ROS 2)
  -> ROS 2 sensor adapter
  -> native ROS 2 D2VINS / D2PGO / D2Comm
  -> ROS 2 D2SLAM adapter
  -> high-rate odometry, sparse landmarks and collaboration status
  -> native AS2 State Estimator plugin when authority is selected
```

D2SLAM remains one shared process group per vehicle. Its state-estimation and
sparse-landmark cards describe two external capabilities of that group, not
two algorithm copies. The AS2 plugin uses D2SLAM as a global correction while
PX4 local odometry remains the high-rate body-state source.

D2VINS exposes two state streams with different semantics. Native `odometry`
is emitted with optimized keyframes and can have roughly 0.5 s gaps; it is
useful as an optimization result but is not a control-rate freshness source.
The Adapter therefore consumes the official high-rate `imu_propagation`
stream and republishes it as `/droneN/d2_slam/odometry`. The accepted runs
measured about 49 Hz with at most 0.045 s source-stamp gaps. Topic selection
must follow the algorithm's official downstream use, not just a matching ROS
message type.

## Build

Build the PX4-1.18-compatible onboard base first, then the algorithm image:

```bash
HTTP_PROXY=<robot1-proxy> HTTPS_PROXY=<robot1-proxy> \
  ONBOARD_COMPUTER_IMAGE=onboard_computer:v0.1.1 \
  bash deployment/onboard_computer/build.sh

HTTP_PROXY=<robot1-proxy> HTTPS_PROXY=<robot1-proxy> \
  ONBOARD_COMPUTER_IMAGE=onboard_computer:v0.1.1 \
  D2_SLAM_IMAGE=aerohub/d2-slam:ros2-humble-native \
  bash deployment/d2_slam/prepare_workspace.sh
```

`prepare_workspace.sh` is retained as the public build entry for compatibility;
it builds an image rather than an external workspace. The image contains the
locked native packages, Adapter, AS2 plugin, AirSim sensor package, runtime
configuration, and validation programs. The optional `quadcam_depth_est`
TensorRT package is not part of the AirSim RGB-D topology and is not built.
The accepted RGB-D profile uses `onnxruntime-cpu`; it assigns three ONNX
intra-op threads to each vehicle so three estimators fit robot1's physical CPU
budget without cross-instance oversubscription. A CUDA device is not an
algorithm requirement for this profile (AirSim rendering still uses the host
GPU).

## Run and validate

```bash
export DISPLAY=:1 AIRSIM_RENDER_MODE=windowed
export AIRSIM_BINARY=<AirSimNH executable>
export D2_SLAM_IMAGE=aerohub/d2-slam:ros2-humble-native

python3 deployment/d2_slam/run_airsim.py shadow \
  --output log/d2-slam/shadow
python3 deployment/d2_slam/run_airsim.py authority \
  --output log/d2-slam/authority \
  --shadow-evidence log/d2-slam/shadow
python3 deployment/d2_slam/run_airsim.py sensor_fault \
  --output log/d2-slam/sensor-fault
python3 deployment/d2_slam/run_airsim.py algorithm_fault \
  --output log/d2-slam/algorithm-fault
python3 deployment/d2_slam/run_airsim.py communication_fault \
  --output log/d2-slam/communication-fault
```

The only compute-container bind mount is the per-run evidence directory at
`/runtime/output` or `/results`. AirSim settings are generated run inputs and
are mounted as a single read-only file into the sensor process; they are not a
software workspace or dependency overlay.

The matrix checks independent sensor input, native ROS 2 graph readiness,
three-vehicle collaboration, sparse landmarks, trajectory accuracy, visible
AirSim, AS2 authority, fault handling, landing/disarm, and owned-resource
cleanup. Authority requires a passing shadow run from the same source and
image identities.

## Accepted evidence

The native Humble matrix passed with image
`sha256:baf21bd2cc05ee01a28dabc63ccede00512a9b88f6b62a669bfaf94769f530fd`:

- `log/d2-slam/ground-imu-propagation-final-20261118/result.json`: native graph,
  real sensor input, landmarks and both remote peers passed.
- `log/d2-slam/airsim-shadow-imu-propagation-final-20261118/result.json`: three
  vehicles ran at 49.68--49.73 Hz; position RMSE was 0.085, 0.135 and 0.093 m.
- `log/d2-slam/airsim-authority-imu-propagation-final-20261118/result.json`: AS2
  reported `plugin_name=d2_slam`; position RMSE was 0.091, 0.144 and 0.088 m.
- `log/d2-slam/airsim-sensor-fault-imu-propagation-final-20261118/result.json`,
  `airsim-algorithm-fault-imu-propagation-final-20261118/result.json` and
  `airsim-communication-fault-imu-propagation-final-20261118/result.json`:
  sensor loss, D2VINS exit and peer loss were detected, followed by landing,
  disarm and owned-resource cleanup.

These paths are ignored runtime evidence on the validation host. Failed
diagnostic runs remain alongside them and are not acceptance evidence.

## Representation and recovery

`/droneN/d2_slam/landmarks` is a sparse visual-landmark view for RViz and
landmark consumers. It is not an occupancy map, ESDF, dense reconstruction, or
proof of free space and must not feed collision checking directly.

Remove D2SLAM authority before stopping the algorithm. A stale global
correction is frozen while fresh PX4 local state supports controlled landing.
Restart only while motion is stopped because restart creates a new
collaborative-map epoch.

The accepted deployment is three PC-simulated compute containers using
`deployment/swarm/`. Hybrid deployment with a physical compute board remains
unvalidated.
