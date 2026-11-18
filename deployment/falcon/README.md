# FALCON deployment

FALCON is delivered as two self-contained images. The compute image
`aerohub/falcon:ros2-humble` is built from `onboard_computer:v0.1.1` and
contains the locked native ROS 2 Humble FALCON Fork, AeroHub adapter, planner
interfaces, FollowTimedPath extension and AS2 launch runtime. The task client
image `aerohub/falcon-workstation:ros2-humble` is built from
`osrf/ros:humble-desktop-full-jammy`; it regenerates the required AS2 message
types from the same locked onboard base and contains no FALCON planner process.
Its AirSim RPC client uses the repository-locked pure-Python `msgpack` vendor
copy instead of installing an untracked package at runtime.
Runtime containers do not mount the repository, source workspaces,
build/install overlays or host dependency directories.

`flight_board:v0.1.0` supplies PX4 SITL, the AirSim clock and sensor bridge.
AirSim runs on the host with `DISPLAY=:1` and
`AIRSIM_RENDER_MODE=windowed`. The integrated planner is CPU-only: AirSim
provides live depth, so the upstream CUDA renderer is not part of this runtime.

## Build and static checks

```bash
bash deployment/falcon/build.sh
bash deployment/falcon/check.sh
```

The build checks that the FALCON submodule is clean and at the locked Fork
revision, computes one deterministic digest over all image inputs, embeds it
in both images, and compiles each role in its prescribed base image. The
static check runs without host mounts and verifies the Humble-only boundary,
zero skipped tests, regenerated workstation interfaces, role separation,
parameter generation and actual planner node construction.

## Visible AirSim validation

Use a new output directory for every case:

```bash
DISPLAY=:1 AIRSIM_RENDER_MODE=windowed \
python3 deployment/falcon/run.py \
  --binary /path/to/AirSimNH/LinuxNoEditor/AirSimNH.sh \
  --case normal \
  --output log/falcon/self-contained-normal

DISPLAY=:1 AIRSIM_RENDER_MODE=windowed \
python3 deployment/falcon/run.py \
  --binary /path/to/AirSimNH/LinuxNoEditor/AirSimNH.sh \
  --case depth-loss \
  --output log/falcon/self-contained-depth-loss

DISPLAY=:1 AIRSIM_RENDER_MODE=windowed \
python3 deployment/falcon/run.py \
  --binary /path/to/AirSimNH/LinuxNoEditor/AirSimNH.sh \
  --case planner-exit \
  --output log/falcon/self-contained-planner-exit
```

The runner first requires the embedded source digest in both images to match
the current worktree. It mounts only generated AirSim settings into the
flight-board container and the independent result directory at `/results`
into the compute and workstation containers. It inspects the actual container
mounts and rejects every unlisted mount. Evidence records image IDs, source
hashes, mount inspection, the visible AirSim window, PX4 preflight state, ROS
authorities, paired depth/odometry, map and frontier growth, rolling B-splines,
FollowTimedPath references, movement, bounds, collision state, fault
injection, stop confirmation, landing, disarm and resource cleanup.

The required matrix is:

| Case | Required outcome |
| --- | --- |
| Normal | Live depth and odometry grow the map, FALCON issues rolling trajectories, FollowTimedPath moves within bounds, exploration completes, then lands and disarms. |
| Depth loss | An actual sensor-stream interruption is detected, execution stops, then the vehicle lands and disarms. |
| Planner exit | The native planner process is killed, trajectory coverage expires safely, execution stops, then the vehicle lands and disarms. |

## Accepted result

The complete matrix passed with visible AirSim, image-embedded PX4 SITL and
the same final image pair and source digest:

| Case | Matched depth/odometry | Map growth | Movement | Safe stop and land/disarm | Evidence |
| --- | ---: | ---: | ---: | --- | --- |
| Normal | 556 | 640,886 points | 5.363 m | Passed | `log/falcon/self-contained-final-normal-r1/` |
| Depth loss | 93 before fault | 188,663 points | 1.579 m | Passed | `log/falcon/self-contained-final-depth-loss-r1/` |
| Planner exit | 174 before fault | 190,470 points | 1.777 m | Passed | `log/falcon/self-contained-final-planner-exit-r1/` |

The compute image is
`sha256:57d923b4315f0d41165181b28a9977ebe61b2662d280abcd518e07a96d690e3e`,
the workstation image is
`sha256:3eb4ad8667859b06b78eedf9e0c50710ae316b886b89a4e5cdd73770a3604f00`,
and the flight image is
`sha256:1eff3190ca7870d0267ddb1531365dd7c68d85c6e52ade3d25b17b84b4758843`.
Both FALCON images embed source digest
`3e9e67ed9b8676aff3a76d7c34745beed26be75acb024f38218048a6280edd6f`.
Each evidence directory includes the inspected runtime mounts and cleanup
result; all three runs ended with their owned containers absent and the scene
stopped.

This establishes PC reproduction under the declared topology. Physical-board
deployment, real flight and resolution of the upstream license remain outside
the accepted scope.
