# FALCON exploration integration

FALCON is a coverage-path-guided autonomous exploration planner. Its upstream
planner and voxel mapper run in `upstream/`; AeroHub owns only the platform and
execution boundary in `ros2_adapter/`.

## Runtime chain

```text
AirSim DepthPlanar + AS2 local odometry
  -> uav_falcon_adapter (timestamp pairing and camera pose)
  -> falcon_ros2 (voxel map, frontier/coverage routing, local B-spline)
  -> uav_falcon_adapter (B-spline PVA/yaw sampling)
  -> FollowTimedPath
  -> AS2 controller/platform -> image-embedded PX4 SITL
```

The adapter does not turn the exploration result into endpoint waypoints. It
preserves FALCON's future trajectory epoch, samples its non-uniform position
B-spline and yaw B-spline, and supplies position, velocity, acceleration and
yaw to the existing timed FollowPath extension. FALCON trajectory IDs are
checked independently while the adapter assigns sequential FollowTimedPath
execution revisions. Late, malformed or out-of-order trajectories are
rejected; completion or an unsafe active trajectory stops the execution
session.

The Fork retains FALCON's clearance checks and adds bounded recovery at the
algorithm boundary. If both the optimized and fallback path for a frontier
violate the configured clearance, that frontier becomes dormant so it cannot
cause an endless unsafe replan loop. A rolling replan that requires a reset
first waits for the current FollowTimedPath session to confirm stop, then plans
from measured hover state. Position and yaw B-splines are also checked against
the configured velocity and acceleration limits before publication. A static
trajectory is retimed on its complete time axis when necessary; a rolling
trajectory that cannot preserve state continuity requests a stop and replans
instead of relaxing the adapter's position, velocity or acceleration gates.

## Input contract

- `sensor_measurements/odom`: `nav_msgs/Odometry` in the fixed vehicle-local
  ENU frame. The adapter rotates child-frame linear velocity into that fixed
  frame for FALCON.
- `sensor_measurements/depth`: `sensor_msgs/Image`, `32FC1` axial depth in
  metres. DepthPerspective is not interchangeable with this stream.
- Depth and odometry retain AirSim sampling time. A depth frame is submitted
  only after it can be bracketed by odometry within the configured tolerance.
  The static body-to-optical-camera transform is then composed at that exact
  sampling time.

FALCON's voxel/TSDF representation is private planning state. Visualization
topics do not constitute a reusable AeroHub map capability in this integration.

## Source provenance

`upstream/` is the `Air-Suck/FALCON` Fork, based on upstream branch
`ros1-noetic` at `312eb4d32c6c7af1a482f94a0a204aa2bb150cca`. The Fork contains
the required ROS 2 build/runtime port because the planner core is directly
coupled to ROS 1 APIs. The selected upstream branch does not provide a usable
root license declaration and its package manifests contain `TODO`; the
integration therefore records the license as unresolved instead of inferring
one from another branch.

Build, launch and validation commands are maintained in
`deployment/falcon/README.md`.

## Validation status

The locked Fork passed the declared PC-simulation matrix with visible AirSim,
the PX4 SITL embedded in `flight_board:v0.1.0`, the compute chain in
`onboard_computer:v0.1.0`, real AirSim depth/odometry, FALCON map growth and
FollowTimedPath motion. Normal exploration, depth-stream loss and planner exit
all reached confirmed stop, native landing and disarm. RoboPi, real flight and
resolution of the upstream license are outside the accepted scope.
