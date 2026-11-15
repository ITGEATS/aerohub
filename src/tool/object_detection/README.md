# Object Detection Tool: public single observation

```python
drone.load_module("object_detection")
response = drone.object_detection.detect(category="person", timeout=2.0)
if response.success:
    for item in response.result.detections:
        print(item.detection.category, item.detection.confidence)
        if item.position_valid:
            print(item.position)
else:
    print(response.code, response.reason)
```

The AS2 module is synchronous and requires the DroneInterface executor to spin
separately, as with navigation. Do not call blocking methods from that executor's
callbacks. `ready` means service discovery only, not camera/model readiness.

## Public contract

`tools/object_detection/observe` uses `DetectSnapshot`:

- Request: `category`, a finite positive `timeout` in wall-clock seconds, and
  `estimate_position` (bool, default false). Set it true to attempt 3D positioning.
- Each request chooses `person`, `car`, `truck` or `bus`; other categories return
  UNSUPPORTED. This reflects the deployed YOLO profile, not a universal class list.
- The adapter captures the first RGB frame **received after acceptance**. It never
  reuses a cached earlier frame. This is a receipt guarantee, not proof that a
  buffered camera acquired the image after the call; acquisition stamps are preserved.
- Camera, registered geometry and output frame are deployment bindings. The caller
  supplies neither an image nor internal IDs/generations. Depth/calibration/TF
  are required only for the positioning option.
- Response: success/code/reason plus ObjectObservation, with RGB/depth acquisition
  headers, position_requested, model identity and detections. In 2D the header
  is the RGB optical frame/stamp; depth_header is empty.
- SUCCEEDED includes valid empty detections. PARTIAL is also success: 2D completed
  but at least one position is unavailable. Check every position_valid; ignore
  position/depth values when false. reason/position_error retain measurement failures.
- BUSY, NOT_READY, STOPPED, INVALID_REQUEST, UNSUPPORTED, TIMEOUT and DETECTION_FAILED
  are failed calls. No retries or request queue are introduced.
- Timeout covers waiting for RGB, inference and measurement. The client includes
  discovery in its budget and allows a bounded 0.5s response transport margin.
  A timed-out adapter task retains its local gate until processing finishes;
  its late response is discarded. This does not cancel a running GPU kernel.
- System stop/clock generation changes invalidate pending observations. Standard
  single-service calls do not expose per-request cancellation. No flight commands,
  target tracking, identity association or prediction are introduced.

The old image-taking DetectObjects service and stream remain internal/compatibility
interfaces. They share the implementation and inference lock. Disable the legacy
stream for task-call deployments; running both can return BUSY. No global GPU or
Mission arbiter is claimed. Process enable/disable is system-managed, not a public
Agent lifecycle operation.

## Build and load

Use the existing fixed Humble image and an external overlay:

```bash
bash deployment/object_detection/build_tool_overlay.sh
```

The script builds interfaces, ROS adapters, AS2 Python API and this module; it
prepares locked CPython3.10 dependencies locally and does not rebuild the image.
Within the same image/overlay environment, source AS2 and
`log/object-detection-tool/workspace/install/setup.bash`, add the workspace's
`python_deps` to PYTHONPATH and set
`AS2_MODULES_PATH=<workspace>/install/share/uav_object_detection_python_api/modules`.
Load this module together with navigation by joining module directories with `:`.
Deploy the independent YOLO worker and ROS inference facade as before, then run
`uav_object_detection` with `deployment/object_detection/params.example.yaml`.
The namespace determines the public service address.

Card v1 and concrete Demo binding: `cards/tool/object-detection.yaml` and
`deployment/object_detection/component.yaml`. Runtime/accuracy evidence and limits
are recorded in `deployment/object_detection/VALIDATION-tool.md`.

For positioning, call:

```python
response = drone.object_detection.detect(
    category="person", timeout=2.0, estimate_position=True)
```

## Existing RGB-D implementation

The public operation defaults to RGB-only detection (`estimate_position=False`):
category, confidence, box and instance mask. No camera calibration, depth camera
or historical TF is required, and no projection is performed.
`ObjectObservation.position_requested` distinguishes unrequested positioning from
a failed positioning attempt. In 2D, header equals image_header, depth_header is
empty, and position_valid=false with an empty position_error.

With `estimate_position=True`, the following existing RGB-D implementation is
used. The first explicit 3D or legacy call lazily activates depth/CameraInfo/TF
consumers; these remain for subsequent 3D calls until process exit. A later 2D
call still skips projection and never depends on their availability.
Legacy DetectObjects and streaming retain their original RGB-D semantics.

This Tool calls a separately deployed YOLO26n-seg Inference ROS service, selects
the requested category (or the legacy stream configuration), measures depth inside each instance mask and reports
unfiltered 3D positions. It subscribes to standard ROS RGB/depth/CameraInfo and
queries historical TF; there is no AirSim API, settings.json, Elastic dependency,
target KF, identity association, predicted velocity or motion-command publisher.

## Files and interfaces

- `interfaces/`: uav_object_detection_interfaces, typed DetectObjects service and
  ObjectDetectionFrame/ObjectDetection measurement messages.
- `ros2_adapter/`: uav_object_detection node and pure depth/projection routines.
- Deployment/parameter example: deployment/object_detection/.
- Capability: cards/tool/object-detection.yaml.

`<namespace>/tools/object_detection/detect` takes request_id, generation and an
RGB8/BGR8 Image. It returns per-instance category/confidence/box/mask plus:

- position_valid and position in meters, with the output frame and **depth capture
  stamp** in result.header;
- original RGB image_header and paired depth_header; caller/tool/worker generation
  and model identity remain distinct;
- selected depth, depth spread and valid pixel count; spread is diagnostic, not
  a complete localization covariance or another confidence score;
- position_error if depth/calibration/TF is unavailable. Never consume a position
  when position_valid is false. A failed 3D result may retain its 2D detections.

An empty detection list with valid=true is a measured no-detection result, not a
predicted target. A downstream navigation component owns association, KF and loss
policy. This Tool does not publish target odometry or refresh target observations.

## Inputs and launch

Build and source the ROS overlay as described in the model README, then run:

```bash
ros2 run uav_object_detection object_detection --ros-args -r __ns:=/drone0 \
  --params-file deployment/object_detection/params.example.yaml
```

The example uses simulation time and single observations, assumes
an existing /clock; positioning additionally needs a provider for
drone0/odom -> drone0/FrontCamera_optical,
and references AirSim-style ROS topic names. All topics/frames are parameters;
no algorithm-side simulator dependency exists. For a physical camera, configure
its registered/rectified topics, actual CameraInfo, output frame and clock.
The example defaults to stream_enabled=false for single-observation use. Optional legacy streaming can be enabled explicitly. registered_depth must be
explicitly true after checking RGB/depth registration; the Tool cannot infer it.

`camera_reliable` defaults to false (sensor-data QoS). For reliable camera
transport, configure the publisher to offer reliable QoS and set this parameter
to true; it applies to RGB, depth and both CameraInfo subscriptions. The AirSim
ground harness at `deployment/object_detection/airsim/` configures both ends.

For explicitly requested positioning, supported depth: 32FC1 meters or 16UC1 with configurable scale (default .001 m).
Default depth is planar optical Z; set depth_is_perspective only for radial range.
RGB/depth must have matching registered geometry, optical frame and intrinsics.
Only rectified/zero-distortion images are supported. CameraInfo is selected no
later than each acquisition stamp, rather than reusing future calibration.

Depth pairing keeps stage2's 0.05 s skew constraint, 60-frame bounded history and
nearest matching sample. TF is queried at the exact integer depth stamp with a
0.25 s arrival wait inherited from the stage2/SUPER integration. No latest lookup,
extrapolation, fixed source-age deadline or extra observation-confirmation rule
is introduced. The complete camera TF contains the installation extrinsic once;
do not apply an additional camera-to-body transform after it.

Measurement depth functions preserve the pinned upstream mask erosion/ROI fallback
and histogram rules, with source/license notices installed by the package.
Existing stage2 pose/registration limitations are not solved by this migration.

## Lifecycle and validation

`tools/object_detection/enable` (SetBool) stops/resumes new calls and streaming.
Stop clears pending input, changes tool generation and rejects late results;
it does not cancel an already running GPU kernel or stop aircraft execution.
On clock rollback, pending data, calibration/depth histories and TF are cleared.
The status topic is `tools/object_detection/status`; streamed measurements are
`perception/object_detection/detections`. Local request locking and latest-only
queuing do not constitute global GPU/mission resource arbitration.

The independent ROS probe at deployment/object_detection/ros_probe.py uses recorded
RGB and real CUDA inference, but deliberately synthetic depth/TF. It checks wire
semantics and known geometry, not sensor accuracy. Tests in tests/ exercise actual
ROS messages/TF2, unit conversion, capture-time interpolation, missing inputs and
cancellation. See model validation.json for executed checks and remaining gaps.
