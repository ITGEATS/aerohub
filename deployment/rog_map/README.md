# ROG-Map deployment

ROG-Map, its native ROS 2 Humble Adapter/interfaces, navigation inputs, AS2
integration, sensor adapter, and acceptance programs are delivered in one
self-contained image based on `onboard_computer:v0.1.1`. Runtime source,
workspace, build/install overlay, and ROS 1 mounts are not supported.

## Build and run

```bash
ONBOARD_COMPUTER_IMAGE=onboard_computer:v0.1.1 \
ROG_MAP_IMAGE=aerohub/rog-map:ros2-humble \
  bash deployment/rog_map/prepare_workspace.sh

ROS_DOMAIN_ID=174 ROG_MAP_NAMESPACE=drone0 \
  bash deployment/rog_map/run.sh
```

The standalone command starts the image without mounts. The map consumes the
AirSim point cloud under the vehicle namespace and provides robot-centered 3D
occupancy plus inflated collision queries for the navigation consumer. Its
occupancy semantics are not interchangeable with raw point sets or neural
implicit maps.

## Visible AirSim acceptance

```bash
export DISPLAY=:1
export AIRSIM_RENDER_MODE=windowed

python3 deployment/rog_map/run_airsim.py run --scenario normal
python3 deployment/rog_map/run_airsim.py run --scenario sensor_fault
python3 deployment/rog_map/run_airsim.py run --scenario algorithm_fault

python3 deployment/rog_map/run_airsim.py status
python3 deployment/rog_map/run_airsim.py stop
```

The runner uses PX4 from `flight_board:v0.1.0` and the ROG-Map component image
for compute processes. Only generated scenario/evidence output is mounted.
Process survival is only a smoke result; reproduction requires real AirSim
point consumption, stable occupancy, a successful downstream query/planning
consumer, failure recovery, and complete evidence for the declared matrix.
That matrix passed on image
`sha256:8801b77bad61fc194ad1bab9700db3b15da711da982748da339b5e13c92d89aa`;
`component.yaml` lists the accepted evidence. Physical-board and real-flight
deployment remain unvalidated.
