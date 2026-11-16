# Object Detection Tool: public single observation

Version 0.4.0 provides query-based RGB detection and optional RGB-D positioning
through the AS2 Module, using deployment-selected YOLO or Grounding DINO. Current
checks cover real CUDA saved-data replay and ROS contracts; earlier YOLO live
AirSim evidence predates this schema. See [query validation](../../../deployment/object_detection/VALIDATION-query.md)
for current results and incomplete platform acceptance.

```python
drone.load_module("object_detection")
response = drone.object_detection.detect(query="person", timeout=2.0)
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

- Request: `query`, a finite positive `timeout` in wall-clock seconds, and
  `estimate_position` (bool, default false). Set it true to attempt 3D positioning.
- `detector_backend` is selected in deployment configuration: `yolo` (default) or
  `grounding_dino`. Query content never switches models or triggers fallback.
- YOLO strips/lowercases query and exactly resolves `person`, `car`, `truck`, `bus`.
  No aliases/plurals/punctuation mapping: other queries return `UNSUPPORTED_QUERY`.
- DINO strips/lowercases query and appends a final `.` only if absent. The text is
  forwarded with deployment thresholds (box .30, text .25 by default). No phrase
  equality filtering or relabeling is applied; all grounded boxes are retained.
- Empty/whitespace/control-character queries are rejected as `INVALID_QUERY` before
  capture (the Python module raises ValueError for empty text).
- The adapter captures the first RGB frame **received after acceptance**. It never
  reuses a cached earlier frame. Nonpositive, duplicate, backward, future and
  preacceptance acquisition stamps are rejected in the configured ROS clock domain.
  Original source stamps are preserved; publisher clock correctness is a deployment condition.
- Camera, registered geometry and output frame are deployment bindings. The caller
  supplies neither an image nor internal IDs/generations. Depth/calibration/TF
  are required only for the positioning option.
- Response: success/code/reason plus ObjectObservation, with RGB/depth acquisition
  headers, position_requested, model identity and detections. `original_query` preserves
  the supplied text; `normalized_query` records the backend interpretation;
  `detector_backend`, request ID and caller/tool/worker generations identify the call.
  `detection.category` remains the actual model label/phrase, which may differ from query. In 2D the header
  is the RGB optical frame/stamp; depth_header is empty.
- SUCCEEDED includes valid empty detections. PARTIAL is also success: 2D completed
  but at least one position is unavailable. Check every position_valid; ignore
  position/depth values when false. reason/position_error retain measurement failures.
- BUSY, NOT_READY, STOPPED, INVALID_REQUEST, INVALID_QUERY, UNSUPPORTED_QUERY, TIMEOUT and DETECTION_FAILED
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
bash deployment/object_detection/build_query_overlay.sh
```

The query overlay builds Inference, Detect and Tracking schemas, all adapters and
Python modules, plus the locked AS2 Python API in `onboard_computer:v0.1.1`.
Source ROS, AS2 and `log/object-detection-query/workspace/install/setup.bash` in
**every** consumer; this is a breaking schema change, with no public `category`
compatibility field. Stop old consumers before loading it. Set `AS2_MODULES_PATH`
to the installed detection and tracking `share/<package>/modules` directories,
joined with `:`. The AS2 API also needs its existing locked Python dependencies
(this build prepares them in the workspace's `python_deps`; add it to PYTHONPATH).

For YOLO use `params.example.yaml`; for DINO use
`deployment/object_detection/params.grounding-dino.example.yaml` and the independent
worker/facade in the [model README](../../basic_function/inference/models/grounding_dino/README.md).
The DINO example uses a 10s inference budget; choose the public observation timeout
explicitly (e.g. 15s for offline replay). These are deployment budgets, not latency
or Tracking freshness guarantees. Worker/model identity changes fail closed and
require explicit adapter/Tool restart and new caller sessions.

Card v1 and concrete Demo binding: `cards/tool/object-detection.yaml` and
`deployment/object_detection/component.yaml`. Runtime/accuracy evidence and limits
are recorded in `deployment/object_detection/VALIDATION-tool.md`.

For positioning, call:

```python
response = drone.object_detection.detect(
    query="car", timeout=2.0, estimate_position=True)
```

## Existing RGB-D implementation

The public operation defaults to RGB-only detection (`estimate_position=False`):
actual model label/phrase, confidence, box and optional instance mask. DINO has no mask. No camera calibration, depth camera
or historical TF is required, and no projection is performed.
`ObjectObservation.position_requested` distinguishes unrequested positioning from
a failed positioning attempt. In 2D, header equals image_header, depth_header is
empty, and position_valid=false with an empty position_error.

With `estimate_position=True`, the following existing RGB-D implementation is
used. The first explicit 3D or legacy call lazily activates depth/CameraInfo/TF
consumers; these remain for subsequent 3D calls until process exit. A later 2D
call still skips projection and never depends on their availability.
Legacy DetectObjects and streaming retain their original RGB-D semantics.

This Tool calls the configured independent Inference ROS service. `backends.py`
owns query resolution, typed request construction and output validation. YOLO
retains the pinned mask depth policy (including its original center-ROI fallback);
DINO uses the separate conservative box foreground policy described below. It subscribes to standard ROS RGB/depth/CameraInfo and
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

## Box-only positioning and query validation

DINO keeps original unclipped XYXY pixels; only the depth sampling region is clipped
to the image. `position_method=box_foreground_surface_v1` identifies an estimated
visible foreground surface point, **not an object center or ground truth**. No
segmentation mask or localization covariance is invented. YOLO positions carry
`yolo_mask_gemini2_reference`; RGB-only results have an empty position_method.

Box-depth v1 uses the inner half of the box: at least 20 valid pixels, at least 60%
valid coverage, axial Z in (0.2,50)m. The nearest supported 0.25m histogram bin
(at least max(5,10% of valid pixels)) seeds a +/-0.25m cluster; it must contain at
least 20 pixels and 60% of valid ROI samples, with p90-p10 <=0.5m and >=60% support
in the middle quarter. The remaining box ring needs >=20 valid samples and median
at least 0.5m behind foreground. Missing/competing depths, flat background, no
center support and unavailable calibration/TF invalidate positioning, preserving
2D boxes and a per-object reason. This conservative policy can reject real objects.
Projection retains the same registered geometry, depth-capture stamp and exact
historical TF checks as YOLO; a frame name is never substituted for a transform.

Current query/backend evidence and replay commands:
[query validation](../../../deployment/object_detection/VALIDATION-query.md).
Existing YOLO live evidence predates this schema; DINO live sensing, 3D accuracy,
moving-target consumption and flight remain unverified. Overall status is partial.
