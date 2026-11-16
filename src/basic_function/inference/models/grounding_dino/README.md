# Grounding DINO tiny — Inference Engine

Text-grounded, open vocabulary **2D boxes only** using the complete official
`IDEA-Research/grounding-dino-tiny` Transformers snapshot. The architecture comes
from the pinned Transformers dependency; no fork or copied architecture. The Engine itself has no depth, target association or motion responsibility.
The separately configured [Detect query backend](../../../../tool/object_detection/README.md)
now consumes this service; its saved RGB-D replay evidence does not extend this
Engine baseline into live sensing, 3D accuracy or flight acceptance.

## Prepare

Linux x86-64, Python 3.10, uv, compatible NVIDIA driver and GPU are required for
CUDA. Dependencies and CUDA wheel sources are fixed by `pyproject.toml`/`uv.lock`.
The supplied snapshot config records `4.40.0.dev0`; the tested direct official
baseline and migration both use **Transformers 4.40.1**, not that historical env.
No model files are downloaded. Supply the complete local snapshot once:

```bash
GROUNDING_DINO_SOURCE=/path/to/grounding-dino-tiny \
  src/basic_function/inference/models/grounding_dino/setup.sh --check-cuda
```

`artifacts.json` pins SHA256 for full `model.safetensors`, config, processor,
tokenizer and vocabulary. Setup verifies the source and links it under ignored
`artifacts/snapshot`; no weights are copied. Existing different or corrupt files
fail rather than being overwritten. `.venv` is independent and ignored. Setup
works outside the repo, repeats safely, and starts no services. Without
`--check-cuda` it performs a CPU preparation check; CUDA checking loads and warms
up the actual GPU with no fallback. Runtime defaults to `cuda:0`; an explicit
`GROUNDING_DINO_DEVICE=cpu` is only for preparation/diagnosis.
`GROUNDING_DINO_MODEL` may override the runtime snapshot directory, but the same
hash manifest remains mandatory. There are no tracked personal source paths.

## Start

From the repository root, worker terminal:

```bash
bash deployment/inference/start_grounding_dino.sh
```

It listens on `127.0.0.1:8003`; `INFERENCE_HOST`/`INFERENCE_PORT` override this.
`/health/live` reports process availability; `/health/ready` becomes available
only after verified load and warmup, with model/device/version/generation/busy.
The shared FastAPI runtime owns one inference slot, retained until the actual
computation finishes even if its caller disconnects. Shutdown closes the backend.

Build the ROS facade in the **existing fixed Docker image**, without rebuilding:

```bash
docker run --rm --init --name grounding-dino-ros-build \
  -v "$PWD:/workspace" --entrypoint bash onboard_computer:v0.1.0 -lc \
  'source /opt/ros/humble/setup.bash && bash /workspace/deployment/inference/build_grounding_dino_ros.sh'
```

Start the facade separately (same host networking reaches the host worker):

```bash
docker run --rm --init --name grounding-dino-ros --network host \
  -e ROS_DOMAIN_ID=227 -e ROS_LOCALHOST_ONLY=0 \
  -v "$PWD:/workspace:ro" --entrypoint bash onboard_computer:v0.1.0 -lc \
  'source /opt/ros/humble/setup.bash && source /workspace/log/grounding-dino-inference/ros_ws/install/setup.bash && exec ros2 run uav_inference grounding_dino_node --ros-args -r __ns:=/drone0 -p endpoint:=http://127.0.0.1:8003 -p timeout_s:=10.0'
```

Service `/drone0/inference/grounding_dino/infer` uses
`uav_inference_interfaces/srv/InferGroundingDino`. Status is published on
`/drone0/inference/grounding_dino/status`. Stop owned adapter and worker with
SIGINT and confirm exit. Model changes require stopping, restoring source/lock
and verified snapshot, rebuilding overlay, then restarting facade and caller
sessions. No automatic lifecycle manager or global GPU resource arbiter exists.
All consumers of the extended ROS package must load this overlay.

Registered model ID: `grounding-dino-tiny` in the shared
[manifest](../../manifest.json). Select it with the dedicated
[configuration](../../../../../config/inference.grounding-dino.example.yaml):

```bash
python3 deployment/inference/setup_models.py --list
python3 deployment/inference/setup_models.py --config config/inference.grounding-dino.example.yaml --dry-run
python3 deployment/inference/setup_models.py --config config/inference.grounding-dino.example.yaml --check-cuda
```

The final command prepares this model and runs CUDA warmup. Registration was
explicitly requested after the existing Engine validation; implementation and
registration remain uncommitted. Component and capability references:
[component](../../../../../deployment/inference/grounding_dino.component.yaml),
[card](../../../../../cards/inference/grounding-dino.yaml). Existing local YAML
and default empty selection are retained.

## Contract

Request: request ID, caller generation, positive acquisition stamp and optical
frame in RGB8/BGR8 Image (up to 2048×2048, valid row stride), text prompt and two
explicit finite thresholds strictly between zero and one. Offline saved-image
replay is allowed: Engine preserves the provided stamp and has no task freshness
policy. The caller owns source provenance, clock, session and freshness policy.

Prompt is lowercased, stripped and given a final dot if absent. Separate concepts
with dots, for example `car. truck. pickup truck.`. Character bound is 1024;
tokenization over 256 tokens is rejected, never silently truncated. The official
processor resizes the **entire** RGB image (shortest edge 800, longest 1333),
rescales and normalizes it. No crop or task class filtering is applied.

The official postprocessor keeps queries with maximum sigmoid token score
**strictly greater than box_threshold**; tokens strictly above text_threshold
form the decoded phrase. No NMS, closed-set class remapping, or masks are added.
`Detection.category` is this phrase (possibly empty at aggressive text thresholds),
not a persistent target identity. Confidence is a token score, not calibrated
object existence probability. All selected queries are returned in model order.
Boxes are official original-image pixel XYXY, **unclipped**, so edge predictions
can exceed the image bounds. Consumers must choose and record any clipping.
The `mask` has the source header and zero dimensions/data; there is no fabricated
segmentation. DetectionFrame includes source/request/caller and worker identity,
model_version, original geometry and timing breakdown. Response echoes original
and normalized prompt and thresholds.

`model_version` hashes all snapshot files plus framework and CPU preprocessing,
precision, prompt and postprocess semantics. Threshold values are request-specific
and echoed rather than creating a different loaded-model identity for every call.
The facade pins worker version/generation, rejects mismatched or late responses,
and requires explicit restart after worker replacement. No automatic retry:
HTTP timeout does not cancel GPU work. Caller session changes require the caller
to discard obsolete responses. Backend transfer/forward/sync timings are nested
within worker/HTTP/adapter/DDS totals; do not add these overlapping figures.

## Validation

Tracked summary: [validation.json](validation.json). Full evidence is in
`log/grounding-dino-inference/`; process record in `skill/temp/grounding-dino-inference/`.
Reproduce the real-image baseline with the prepared model environment:

```bash
src/basic_function/inference/models/grounding_dino/.venv/bin/python \
  deployment/inference/tests/grounding_dino_baseline.py \
  --image log/yopov2-tracking/side-observe-20261116-02/probe/hover-parked.png \
  --output log/grounding-dino-inference/baseline.json
```

This saved full real image is Engine validation input, not live sensor, detection
accuracy, Tool/Tracking, AirSim/PX4 closed loop, flight or RoboPi acceptance.

The checked run returned two official phrases (`car pickup truck`, `truck`) with
confidence approximately 0.398/0.383. All 900 raw query outputs and all selected
boxes passed the predeclared numerical thresholds. HTTP inference and 12 invalid
requests (including nonfinite thresholds), real timed-out-call busy/429/drain,
and independent fixed-image ROS calls passed. Full-image DDS roundtrips were
about 3 seconds despite ~110 ms worker time; there is no real-time guarantee.
Optional kernel-loading warnings are emitted by this Transformers version during
construction; `disable_custom_kernels=True` selects its official portable GPU
attention, and no custom kernel/toolchain or upstream source patch is required.
Repository-wide card validation still reports existing legacy-card errors; the
new v1 Grounding DINO card has no validator errors. See the tracked summary for
all limits and the retained initial failures.
