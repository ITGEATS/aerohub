# Detect query / Grounding DINO Tool — 2026-11-16

Detect **0.4.0** and Tracking **0.2.0** implement the approved query interface.
Overall platform acceptance remains **partial**. Existing uncommitted Engine work
was preserved on `feat/target-tracking`; no commit, branch change or push was made.

```python
response = drone.object_detection.detect(query='car', estimate_position=True, timeout=15.)
goal = drone.target_tracking.track(query='car', timeout=10.)
```

Public DetectSnapshot and TrackObject use `query` only. `category` remains the
actual model output label/phrase. Original/normalized query and backend identity
are separately returned in ObjectObservation. Deployment selects YOLO or DINO;
unknown fixed YOLO queries return UNSUPPORTED_QUERY; empty text is rejected.
DINO normalization strips/lowercases and appends a final dot if absent. Grounded
phrases are preserved even when different from the original query, including an
empty decoded phrase. Thresholds were fixed before testing at box=.30, text=.25.
There is no fallback, alias mapping, phrase equality filter or fabricated mask.

## Executed matrix

Evidence root: `log/object-detection-query/` (ignored runtime artifacts).
Predeclared matrix and retained failures: `skill/temp/object-detection-query/README.md`.

| Check | Result | Evidence |
| --- | --- | --- |
| Fixed image external overlay, matching interfaces and consumers | 9 packages built; Inference, Detect, Tracking, Python modules and locked AS2 API | build.log |
| Actual generated ROS messages, DDS and TF2 contracts | 42 tests, no skips | tests-final.log |
| Fixed YOLO normalization/lookup and unchanged masks | supported exact lookup, unknown/empty rejection, mask geometry regression passed | tests-final.log |
| DINO dispatch and query/phrase semantics | normalized prompt echoed; mismatched/empty model phrases preserved; bad prompt/request/model/worker identity rejected | tests-final.log |
| Fresh single capture and lifetime | cached/preacceptance input rejected; timeout/BUSY/stop/late reply behavior passed | tests-final.log |
| Separate box-depth policy and historical TF | synthetic separated foreground accepted with nonidentity transform; missing/competing/background/center failures invalid | tests-final.log |
| Tracking serial sessions and query propagation | query-only Goal/Service, valid synthetic positions, cancellation, late generation, stale/future captures and clock reset passed | tests-final.log |
| Real CUDA HTTP + full original-image baseline | actual cuda:0 boxes match prior direct baseline; 12 invalid requests, timed-out-call busy/drain/recovery passed | http.json, http.log |
| Real CUDA Engine → Tool → AS2 Detect Module | three different fresh replay captures; query car accepted; exact reference box and model/worker identity preserved | replay-final.json, replay-final.log |
| Actual AS2 Tracking Module → Action → Detect | car query accepted; 95 feedback messages, 0 valid 3D states; unavailable positions never initialized EKF | replay-final.json |
| In-scope capability cards | Detect, Tracking and Engine cards have no validator errors; 57 unrelated legacy-card errors remain | cards-scope.json, cards-final.log |

Test breakdown: YOLO contract 4, box depth 2, projection 8, public snapshots 9,
backend/query 8, Tracking core 6, Tracking ROS 5. Deterministic Engine responses
are explicitly synthetic contract tests; they do not prove model accuracy.

Image: existing `onboard_computer:v0.1.1`,
`sha256:c8756265a76d827633e132824922ccdf2801c2e4bae864a08d4a54215d4c601e`.
No image rebuild. Isolated test/replay DDS domains 231/232, localhost only.
Actual CUDA model: `grounding-dino-tiny`, model_version
`a582d4cebaa628f841a7914b9543262b7f89d82428498ecf73d8d204cfb81530`.
Existing pinned Torch 2.6.0+cu124 / Transformers 4.40.1 environment and snapshot
were reused without downloading or changing weights. HTTP transport uses the
existing worker; ROS tests use real generated messages and DDS. Tool replay
hosts the ROS facade, Tool and publisher in one container/process with a separate
AS2 executor; it does **not** establish cross-machine transport performance.

## Saved pickup result and limits

Full saved `hover-parked.png` from
`log/yopov2-tracking/side-observe-20261116-02/probe/` was replayed losslessly with
its saved planar depth, recorded nonidentity camera→earth transform and render
projection intrinsics. Original RGB PNG SHA256:
`2d5e19dd3762b50eacf55949a44622b97d2a037eac25fa96967cef1aa26303f7`.
All source hashes and original recorded geometry are in replay-final.json.
The source optical frame is retained. **Replay stamps are synthetic**, and the
recorded TF is held at those stamps; this does not reconstruct a live TF stream
or establish original RGB/depth/pose synchronization or actual 3D accuracy.

`query='car'` yields one actual model label `car`, confidence **0.755788**,
XYXY approximately **[102.168, 193.080, 539.905, 363.719]**. The normalized query is
`car.`. The Engine reference and all Tool boxes agree at atol=.02px, rtol=1e-5.
An RGB-only call succeeds. Both positioned calls return **PARTIAL** with
`position_valid=false`, `position_method=box_foreground_surface_v1`, and
`ambiguous competing foreground depths`. The foreground gates were not relaxed.
This validates conservative rejection, **not successful real 3D localization**.
Actual Tracking reports no usable 3D positions, as expected. Its TIMEOUT result
means the observation window ended; it is not a target-tracking success claim.
Public Detect roundtrips were approximately 467/505/500ms in this local replay;
no performance threshold or continuous Tracking freshness guarantee was tested.

The box policy and its fixed thresholds are specified in the
[Tool README](../../src/tool/object_detection/README.md#box-only-positioning-and-query-validation).
It measures visible surface support, not an object center. YOLO retains its
separate original mask/ROI algorithm. Check position_valid on every detection.

## Reproduce without flight

```bash
bash deployment/object_detection/build_query_overlay.sh
bash deployment/object_detection/test_query_overlay.sh
# Separate terminal, existing prepared CUDA environment:
bash deployment/inference/start_grounding_dino.sh
```

In the fixed image, mount the repo at `/repo`, overlay workspace at `/work`, and
an existing writable evidence directory at `/evidence`; use an isolated DDS domain.
Source `/opt/ros/humble/setup.bash`, `/root/aerostack2_ws/install/setup.bash`, and
`/work/install/setup.bash`. Set `PYTHONPATH=/work/python_deps:$PYTHONPATH` and
`AS2_MODULES_PATH` to the two installed module directories, joined with `:`:

```bash
export AS2_MODULES_PATH=/work/install/uav_object_detection_python_api/share/uav_object_detection_python_api/modules:/work/install/uav_target_tracking_python_api/share/uav_target_tracking_python_api/modules
python3 /repo/deployment/object_detection/probe_query_cuda.py \
  --input-dir /repo/log/yopov2-tracking/side-observe-20261116-02/probe \
  --output /evidence/replay-new.json
```

All changed schema consumers must source the same overlay and restart; old public
category clients are intentionally incompatible. Production DINO selection uses
`params.grounding-dino.example.yaml`; the model worker and ROS facade are started
separately. The replay probe starts only owned software nodes, not a simulator,
physical devices, navigation or flight clients. Stop the owned worker after use.

Not executed: live DINO sensor input, successful real box-depth accuracy,
valid-3D DINO tracking, moving-target/occlusion/platform fault recovery,
AirSim + PX4 flight and physical-board acceptance. These remain incomplete;
prior YOLO platform evidence does not close the new DINO gaps.
