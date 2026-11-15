# FALCON deployment

This deployment uses the project-standard environments only:

- `flight_board:v0.1.0` supplies PX4 SITL, the AirSim clock and sensor bridge;
- `onboard_computer:v0.1.0` supplies ROS 2 Humble, AS2 and the compute runtime;
- AirSim runs on the host with `DISPLAY` set to the active graphical desktop
  and `AIRSIM_RENDER_MODE=windowed` for acceptance. The runner verifies that
  the selected X display is reachable before creating evidence.

No FALCON-specific image is built. `build.sh` compiles the locked Fork,
adapter, interfaces and FollowTimedPath extension into the external
`log/falcon/install` overlay. Additional Ubuntu development libraries are
downloaded as versioned, checksummed `.deb` files under ignored
`log/falcon/deps` and extracted into the temporary build/runtime container.
The same script builds canonical `as2_msgs` from the locked AeroStack2
submodule into `log/falcon/workstation/install` using the fixed ROS desktop
image. This keeps the workstation client on its assigned image without relying
on compute-container installation paths.

The integrated FALCON path is its CPU planner, voxel mapper and frontier/
coverage logic. The upstream CUDA renderer is not part of this package because
AirSim supplies the live scene and depth stream, so no FALCON CUDA algorithm
test applies. Visible AirSim validation remains mandatory.

## Build and checks

```bash
bash deployment/falcon/build.sh
bash deployment/falcon/check.sh
```

The generated ROS 2 parameters are derived from the YAML files inside the
locked Fork. `prepare_params.py` flattens their namespaces for the compatibility
layer and applies the AirSim camera geometry, exploration bounds, writable LKH
directory and future handoff interval. It does not edit the locked configs.

## Visible AirSim validation

```bash
DISPLAY=:1 AIRSIM_RENDER_MODE=windowed \
python3 deployment/falcon/run.py \
  --binary /path/to/AirSimNH/LinuxNoEditor/AirSimNH.sh \
  --case normal \
  --output log/falcon/e2e-normal

DISPLAY=:1 AIRSIM_RENDER_MODE=windowed \
python3 deployment/falcon/run.py \
  --binary /path/to/AirSimNH/LinuxNoEditor/AirSimNH.sh \
  --case depth-loss \
  --output log/falcon/e2e-depth-loss

DISPLAY=:1 AIRSIM_RENDER_MODE=windowed \
python3 deployment/falcon/run.py \
  --binary /path/to/AirSimNH/LinuxNoEditor/AirSimNH.sh \
  --case planner-exit \
  --output log/falcon/e2e-planner-exit
```

Each case requires a new, nonexistent output directory. Do not reuse evidence
from an earlier source revision or validation attempt.

The runner creates an isolated ROS domain and ports, derives a dedicated
AirSim settings file with a 256x256 90-degree `DepthPlanar` front camera, starts
the fixed flight-board containers, and requires the target PX4 firmware's uORB
`vehicle_status.pre_flight_checks_pass` to remain true for two seconds before
launching the upper chain. It then launches AS2 and FALCON from the external
overlay, performs native takeoff, and enables exploration. It records source,
image and artifact hashes, PX4 preflight evidence, topic authorities,
depth/pose pairing, FALCON map growth, B-spline revisions, FollowTimedPath
feedback, vehicle motion, completion/failure, collision state, landing and
final disarm.

The `--binary` argument may point either to the packaged `AirSimNH.sh` launcher
or directly to `AirSimNH/Binaries/Linux/AirSimNH`. The runner resolves the
launcher to the managed ELF so lifecycle identity and cleanup remain reliable.

Do not claim reproduction success from build or node startup alone. The final
merged revision must pass every applicable case in the validation matrix,
including actual AirSim sensor consumption, generated coverage trajectories,
movement through FollowTimedPath, completion/progress criteria, stale sensor
and planner-exit stopping, collision/bounds checks, and clean landing/disarm.

## Accepted result

The locked Fork revision and this integration passed the complete declared
visible-AirSim matrix on `DISPLAY=:1` with windowed rendering:

| Case | Result | Evidence |
| --- | --- | --- |
| Normal exploration | FALCON consumed paired depth/odometry, grew its map, issued rolling trajectories through FollowTimedPath, moved 3.71 m, then landed and disarmed | `log/falcon/e2e-normal-20261116-display1-r34/result.json` |
| Depth loss | Stale input stopped execution, followed by controlled landing and disarm | `log/falcon/e2e-depth-loss-20261116-display1-r1/result.json` |
| Planner exit | Confirmed planner process death stopped trajectory execution, followed by controlled landing and disarm | `log/falcon/e2e-planner-exit-20261116-display1-r4/result.json` |

Planner-exit detection has a timing-dependent terminal surface. After the
runner proves that the injected process is dead, the active execution may
report replacement timeout, trajectory-coverage expiry, or FollowTimedPath
failure. All three are accepted only when the adapter subsequently confirms
stop and the vehicle lands and disarms; the error text alone is not evidence
of successful fault handling.

The port preserves the configured collision and dynamic limits. A frontier
whose optimized and fallback trajectories both violate clearance is retired
instead of repeatedly selected. Rolling replanning waits for FollowTimedPath
stop acknowledgement before rebuilding from measured hover state. Static
trajectories that exceed configured velocity or acceleration limits are
retimed across the complete position and yaw time axes before publication;
rolling trajectories that cannot be retimed continuously stop and replan.

The accepted scope is PC simulation with the fixed project images. RoboPi and
physical-flight deployment are not covered. The selected upstream branch has
no resolved root license declaration, so redistribution remains subject to
that recorded limitation.
