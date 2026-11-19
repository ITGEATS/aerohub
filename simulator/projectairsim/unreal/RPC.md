# HUD v1 runtime interface

POST JSON-RPC requests to `http://127.0.0.1:<display-port>/aerohub/rpc`.
This is an independent HTTP endpoint, not ProjectAirSim's NNG transport.
Control programs call this endpoint directly; the Python SDK and UE console are not required.
`hud_rpc.py` is an optional diagnostic example.
Requests use explicit id and object params. Batches/notifications are not supported.

| Method | Parameters |
| --- | --- |
| SetTitle | text; side optional: left (default) / right; empty clears only that side; up to 512 UTF-16 units |
| ShowMessage | text, duration_seconds optional (3 default, 0 persistent, maximum 86400) |
| ClearMessage | none |
| GetDisplayState | none |
| SetRunState | vehicle_id, state: idle/running/finished |
| ControlRun | vehicle_id, event: start/finish/reset; updates run state and timer together |
| ControlTimer | vehicle_id, event: start/finish/reset |
| SetView | vehicle_id, camera_id, image_type; __global__ needs only vehicle_id |
| RegisterViews | views array containing vehicle_id/camera_id/image_type/component |

`RegisterViews` is an adapter bootstrap operation, normally performed by
`display_session.py` after expanding the native scene using the installed SDK.
Only streaming-enabled captures become main-view candidates. The adapter maps the
locked upstream UE capture component names to the original sensor identities;
there is no access to upstream private state and no upstream modification.

The plugin owns the main observation camera once views are registered.
Tab cycles these candidates, G toggles Global/previous vehicle view.
It consumes these keys rather than relying on the upstream Tab index.
Actual Tab/G input was verified against the packaged program.
Global uses reserved __global__ identity, label Global, TopCamera, RGB. Its
position is updated from native actors' actual world positions every game tick,
above their centroid, looking vertically downward. It hides vehicle instruments.

The main camera copies native capture view pose/FOV/postprocess, so RGB and
DepthVis remain distinguishable. Independent windows remain client-side native
ImageDisplay windows; they do not change when Tab/G is pressed.

Run state and wall timer are independent per vehicle. Timer includes elapsed
wall time; reset clears only that vehicle's timer and peak. The old mission
HTTP paths act on the currently observed vehicle and reject Global. Legacy timer calls do not infer run state. Normal business callers use ControlRun:
start enters RUNNING, starts a new timer and clears peak; repeated start preserves
the active timer and peak. finish enters FINISHED and freezes elapsed time, repeated
finish preserves the result; idle finish is rejected without mutation. reset enters
IDLE and clears elapsed time/peak. All changes happen in one game-thread callback.
Timer uses the server monotonic wall clock; all state is independent per vehicle.

Two independent titles persist until cleared or the UE world ends. Left uses warm gold,
right uses cyan; both use the same CJK Bold, size, dark outline and top inset.
Right title is aligned to the right edge. Both are drawn below the HUD.
GetDisplayState returns title_left and title_right; legacy title aliases title_left.
An invalid side is rejected without modifying either title. Center messages replace the
previous message, use wall time, wrap to at most three lines. Both are drawn
below the main HUD. Noto CJK regular with its distribution license is shipped.
A state response acknowledges the display model, not guaranteed presentation.

Control programs should use the SDK-free [ProjectAirSimExtensionClient](../EXTENSION_CLIENT.md),
which wraps these methods and distinguishes transport, protocol and server errors.

## Real display session

Use the configured projectairsim_python interpreter:
```bash
python simulator/projectairsim/display_session.py --scene /path/scene.jsonc \
  --load --topic-port 10089 --service-port 10090 --display-port 9091
```
--load explicitly reloads the robot scene; do not use it during flight.
Without --load, the supplied scene must match the already loaded world.
This session owns the native Pair0 topic connection; do not run a ROS bridge
on the same topic connection simultaneously.

Initial small-window choices are code-defined in display_session.py. The session
runs without terminal input and forwards actual telemetry. It does not set task
state; the control program owns state, timer, title and message updates via RPC.
Small-window groups are managed in the client process with ImageWindows.show /
close; these Python operations are separate from the UE HUD RPC endpoint.

For example, a control program can send this HTTP request body:
```json
{"jsonrpc":"2.0","id":1,"method":"SetTitle","params":{"text":"场景观察"}}
```
For the right title, send `SetTitle` with `{"side":"right","text":"机群状态"}`.
Omitting side preserves existing left-title callers. Clearing either title leaves
the other title intact.

Use `ControlRun` for normal task lifecycle events. `SetRunState` and `ControlTimer`
remain low-level compatibility operations; do not mix them with ControlRun.
No terminal command loop or UE `~` console is part of the display control flow.

The main bar retains the original compact translucent panel and thin outline.
Vehicle identity is the primary label; state and wall time retain dedicated
positions. Below it, a smaller strip shows the raw camera name and a quieter
image-format badge. The speed dial keeps its established geometry.

Current limitation: the legacy upstream T trace remains associated with its
upstream observation index, not this plugin's chosen view. Do not use it as
multi-vehicle trace management. This iteration does not implement scene markers.


## Verification and known environment warnings

2026-11-15: actual packaged Vulkan runtime with two unarmed SimpleFlight robots;
explicit run state, independent timers, title/message update and expiry,
main-view identity and actual Tab/G input were checked. Global camera pose was
queried independently via native GetObjectPose before/after a controlled
4 m ground teleport and matched the moving centroid to 2 cm with vertical pitch.
Vehicle restored afterward; no arming or flight.

The initial GUI thread-recreation strategy blocked in native stop/join.
The group manager now spawns one native ImageDisplay worker per group; three
real RGB/depth close/reopen cycles confirmed OS window creation and removal.

Runtime logs retain a handled startup ensure in UE FIoDispatcherImpl, missing
optional PixelStreaming console variables, and initial spawn geometry warnings.
They did not prevent the above checks but are not reported as fixed.


Reload regression: destroyed native capture components must never be selected.
The adapter filters invalid/unregistered components and destroying actors, then
rechecks the weak pointer after reselection. Three actual world load/reload
cycles passed after fixing this boundary; failures and successful logs retained.
Final build and checks are indexed by log/projectairsim/hud-v1/manifest.json.

## Explicit scene follow override for Global

Default Global behavior remains the native UAV-centroid overhead view. An individual
scene may opt in through its display JSON:

```json
{"global_camera_tag":"WestRunnerGlobalFollow"}
```

Exactly one live `ACameraActor` in the current world must have this actor tag.
Global copies that camera component's pose, FOV and postprocess each tick; this is an
observation override, not vehicle control. No target or duplicate tags report
`view_available=false` and `NO FOLLOW TARGET`; they must not be called successful
tracking. State adds `global_mode` (`uav_centroid`/`actor_follow`),
`global_camera_tag` and `global_target_present`. The override does not change any
vehicle's telemetry, timer or mission state. Sequencer Camera Cuts must not compete
with the observation layer. Empty/absent configuration preserves the default.
