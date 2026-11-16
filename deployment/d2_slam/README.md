# D2SLAM deployment

This deployment integrates the ROS 1 D2SLAM implementation into AeroHub's
ROS 2 Humble and AS2 runtime. It reuses the repository's `deployment/swarm/`
infrastructure and runs three independent compute containers. It does not
build an algorithm-specific image.

## Runtime architecture

```text
AirSim RGB + DepthPerspective + IMU (ROS 2)
  -> per-vehicle ROS 2 adapter
  -> Unix-domain socket
  -> ROS 1 proxy
  -> native ROS 1 D2VINS/D2SLAM
  -> ROS 1 proxy / Unix-domain socket
  -> ROS 2 odometry, landmarks and collaboration evidence
  -> native AS2 State Estimator plugin
```

The bridge is deliberately narrower than a general `ros1_bridge`: it carries
only the inputs and outputs needed by D2SLAM, preserves acquisition stamps and
frames, bounds its queues, reports drops/reconnects, and does not replay stale
state after reconnect. D2SLAM remains one shared algorithm runtime: the state
and collaborative-landmark cards are separate external capabilities, not
separate algorithm processes.

The AS2 plugin treats D2SLAM as a low-rate global correction. PX4 local
odometry remains the high-rate `odom -> base_link` and body-twist source;
D2SLAM updates `map -> odom`. Only one AS2 state-estimator host may own
canonical pose, twist and TF for a vehicle.

## Locked environment

- visible AirSim 1.8.1 (`DISPLAY=:1`, `AIRSIM_RENDER_MODE=windowed`);
- PX4 SITL from `flight_board:v0.1.0`;
- ROS 2/AS2 and the external ROS 1/CUDA workspace in
  `onboard_computer:v0.1.0`;
- three `pc_simulation` vehicles from
  `deployment/swarm/configs/d2_slam.yaml`;
- NVIDIA GPU execution for every D2VINS process.

Prepare the ignored external workspace with:

```bash
export D2_SLAM_WORKSPACE="$PWD/log/workspaces/d2_slam"
bash deployment/d2_slam/prepare_workspace.sh
```

The preparation entry locks the ROS 1 and CUDA packages, builds the native
catkin workspace and the ROS 2 Adapter/plugin overlays using the fixed onboard
image. The catkin artifacts contain absolute paths, so the tool container must
mount the workspace both at `/workspace` and at its original host path. The
runtime uses the same two read-only mounts. Do not copy build products into the
repository or create a D2SLAM image.

## Run and validate

Set the machine-local AirSim binary and PX4 message overlay as needed; these
paths are local configuration and must not be committed:

```bash
export DISPLAY=:1 AIRSIM_RENDER_MODE=windowed
export AIRSIM_BINARY=<AirSimNH executable>
export D2_SLAM_WORKSPACE=<prepared D2SLAM workspace>
export PX4_MSGS_OVERLAY_WORKSPACE=<overlay matching flight_board PX4>

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

Every invocation requires a fresh output directory and owns its AirSim
instance, three flight-board containers, three compute containers, adapters,
mission process and evidence. The authority action accepts only a passing
shadow directory with the same runtime source hashes, D2SLAM revision and
image IDs. The declared matrix checks:

- independent real RGB, DepthPerspective and IMU input for all three vehicles;
- three real GPU D2VINS processes, nonempty landmarks and both remote peers;
- fixed-scale trajectory accuracy against AirSim truth;
- visible AirSim and a moving fleet-centroid overhead view;
- native AS2 authority output with high-rate local-state composition;
- sensor loss, algorithm exit and peer-communication loss;
- controlled fleet landing, disarm and owned-resource cleanup.

The validator records both source timestamps and local arrival time. ROS
callbacks run in an executor independent of AirSim truth RPC calls so evidence
collection cannot manufacture stream gaps.

## Sensor and representation boundaries

The front camera is placed 0.4 m forward and the same body-to-camera extrinsic
is configured in D2SLAM. This prevents the vehicle's arms and propellers from
dominating tracking; changing the camera pose requires changing both files and
rerunning the full matrix. Depth is AirSim `DepthPerspective`, converted to
the native `16UC1` millimetre contract without reducing it to 8-bit data.

`/droneN/d2_slam/landmarks` contains initialized sparse visual landmarks from
the collaborative estimator. It is suitable for RViz and landmark consumers.
It is not an occupancy grid, dense reconstruction, ESDF or proof of free
space, and must not be connected directly to collision checking.

## Stop and recovery

Use `run_airsim.py stop --output <run-directory>` only for resources owned by
that run. Remove D2SLAM authority before stopping the algorithm. A stale or
failed global correction is frozen while fresh PX4 local state remains
available for a controlled landing; failure of PX4 local odometry is outside
that degradation claim. Restart D2SLAM only while motion is stopped because a
restart creates a new collaborative map epoch.
