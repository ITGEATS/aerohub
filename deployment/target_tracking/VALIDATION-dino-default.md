# Target Tracking Demo DINO defaults — 2026-11-16

Deployment wiring updated; platform acceptance **incomplete**. Prior YOLO validation
and screenshots remain historical. No scene, simulator, takeoff, hover or flight
was started during this change. No branch switch, commit or push.

Only this Demo selects `grounding_dino`, query `car`, model `grounding-dino-tiny`,
port8003. All three probes create GroundingDinoNode and load params.yaml. Mission
continues to use Tracking → DetectSnapshot → InferGroundingDino, never a model API.
The shared Object Detection harness retains its optional backend's YOLO default.
The query workspace contains matching Inference/Detect/Tracking schemas/consumers;
old tracking/object-detection overlays are not sourced by this Demo.

| Executed check | Result |
| --- | --- |
| Mission logic regression | 7 passed |
| New launch plans/configuration, default car, no YOLO node construction, shared YOLO default | 6 passed |
| Existing generated ROS/DDS/TF2/Detect/Tracking regression | 42 passed, no skips |
| Actual Demo params + DINO/Detect/Tracking ROS node construction | 1 passed |
| Existing PX4 message/flight evidence serialization regression (no flight) | 1 passed |
| Actual CUDA saved RGB-D → Engine → Detect Module → Tracking | Interface/protection assertions passed; no valid 3D tracking |
| Live DINO sensing, valid real 3D tracking, moving Mission, hover/flight | Not run |

57 unit/ROS tests passed, plus one successful real CUDA replay scenario. Syntax
compilation and diff whitespace checks passed. This run reused the existing
onboard_computer:v0.1.1 query overlay, without rebuilding images or model weights.

Replay uses the saved side-observe-20261116-02 pickup image/depth, recorded TF held
at synthetic replay stamps. Full source hashes and worker identity are in
`log/target-tracking-dino-switch/replay.json`. One car at confidence .755788;
three unique replay captures, RGB-only success, both positioned calls PARTIAL with
`ambiguous competing foreground depths`, position_valid=false. Tracking produced
95 feedback messages and zero valid 3D states; TIMEOUT ends its window, not a
tracking success. No masks fabricated and no depth/freshness gates relaxed.

Retained failed attempts: host configuration tests initially lacked msgpack
(resolved by importing it after dry-run); DDS domain233 overflow (replayed on230);
host flight-evidence test lacked rclpy (passed in actual ROS container). Logs are
preserved in `log/target-tracking-dino-switch/`. Owned CUDA worker exited normally;
test containers were removed. Existing wonderful_mendel and aerohub-course-wmm-px4
remained running. No test workers remain.

Recheck:

```bash
python3 -m unittest discover -s deployment/airsim/demos/target_tracking -p test_mission.py -v
python3 -m unittest discover -s deployment/target_tracking -p test_demo_config.py -v
bash deployment/object_detection/test_query_overlay.sh
# In the same ROS image with /repo and the query workspace mounted at /work:
python3 /repo/deployment/target_tracking/test_demo_ros.py -v
python3 /repo/deployment/target_tracking/airsim/mountains/test_flight_evidence.py -v
```

Saved CUDA replay preparation/commands are in
[Detect query validation](../object_detection/VALIDATION-query.md#reproduce-without-flight).
[Demo deployment](README.md#dino-default) provides the future observation-first and
explicit hover commands. Full moving-target/flight acceptance remains incomplete.
