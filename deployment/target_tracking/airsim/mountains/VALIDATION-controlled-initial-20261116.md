# Controlled initial viewpoint retry — 2026-11-16

Status: **not passed / moving observation incomplete**. Real CUDA Grounding DINO,
query `car` / prompt `car.`, existing unified query overlay, native AS2/PX4
Takeoff/hover/Land. No Navi pursuit, truth injection, runtime teleport or gate changes.

The predeclared gates remain 3m native takeoff and stable hover, at least four distinct
valid parked acquisition stamps/capture IDs, then 12s moving observation with at least
four valid acquisitions and >1m estimated displacement; measured convoy stop and
native landing/disarming. See each run's `checks.json` for complete predicates.

| Run | Initial fixture | Result |
| --- | --- | --- |
| `controlled-observe-20261116-01` | X6, Y0, yaw180, camera pitch−25 | Spawn outside bridge at lower terrain; no vehicle in hover image; static gate failed |
| `controlled-observe-20261116-02` | Validated X2.6, Y0, yaw180, camera pitch−75 | Full pickup visible; all 25 inference calls returned boxes; primary box score range 0.576–0.803; all 29 returned boxes rejected by `ambiguous competing foreground depths`; zero valid tracked acquisition |

Paths are under `log/yopov2-tracking/`. Both runs passed native takeoff, stable hover,
native Land, landed/disarmed, physical 2.7–3.3m hover height and no overhead contact.
Measured physical hover ranges were 2.908–3.071m in run02 and 2.908–3.066m in run01.
Run02's independent camera/ROS optical TF and projection-intrinsics checks also passed.
Neither run started the convoy; the moving phase was not executed. Both measured
convoy stop successfully, removed owned containers and workers, and confirmed the
IDs/StartedAt of the two pre-existing containers unchanged.

Evidence per run: `probe/result.json`, `probe/inference-calls.json`,
`probe/projection-calls.json`, `probe/hover-parked.png`, `physical-evaluation.json`,
`cleanup.json`; run02 additionally `camera-evaluation.json`. Images and raw depth
from the manual hover capture are not paired by identity with the later inference
calls: do not treat them as exact per-call projection inputs.

The saved older side-view audit is in `controlled-depth-audit-20261116/`:
actual nearest cluster covers 27.1% of inner valid pixels; even the best arbitrary
0.5m interval covers only 42.2%. This demonstrates that rejection can arise from
surface depth variation; it does not prove two physical objects occupy that ROI.
Run02 independently shows that improving visibility alone does not establish 3D validity.

Next technical investigation: record each projection's exact depth, box, calibration
and TF; distinguish continuous sloped vehicle surfaces, open cargo bed and background
before choosing a revised measurement method. Keep the surface/centroid distinction,
invalid-data behavior and independent real-data acceptance checks explicit. No depth
policy change is claimed or implemented by this retry.
