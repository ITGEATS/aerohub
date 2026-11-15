# UE5 observation display

## 当前 HUD v1（2026-11-15）

本版完成主视口身份、按车辆隔离的状态/墙钟计时/速度峰值、
Global 质心俯视、动态标题与中央提示，移除仪表底部重复状态。
Tab 在本项目观察适配层登记的原生 streaming capture 间轮换，G 切换 Global。
不修改上游，不读取上游 private 状态；原始相机位姿和后处理由原生 UE 组件提供。

可读性修订：WALL 改为蓝白粗体；相机行字号和字重提高、背景加深，
RGB 用亮青色、深度格式用暖金色。实际 1280×720 DepthVis 高反差背景已审核。
左右标题均采用独立 Noto CJK Bold 与深色描边，左侧暖金色、右侧亮青色；
字体与许可随包部署。SetTitle 通过 side 选择，默认 left，支持独立清空。
标题源码依据与验证记录见 [TITLE_STYLE.md](TITLE_STYLE.md)。

生命周期主入口为 ControlRun（start/finish/reset），同一回调原子更新指定车辆
状态和计时；正常重复调用保持结果，IDLE 下 finish 拒绝。旧独立接口保留兼容。
实际双机计时隔离、重复调用、重新开始及非法事件检查通过，无飞行操作。

调用入口与完整语义见 [RPC.md](RPC.md)。中文字体 Noto CJK 及许可随包部署。
HTTPServer 回调由其核心 Tick 执行；开发及运行验证未建立额外后台线程修改 UE 对象。
独立 RPC 与现有计时 HTTP 兼容，任务生命周期事件由调用方上报，服务端维护对应显示状态和计时。

当前运行证据位于 log/projectairsim/hud-v1/：
插件编译、实际场景包 Build/Cook/Stage/Archive、双机静态真值遥测、RPC 状态/
计时隔离、消息过期、实际 Tab/G、Global 控制地面瞬移后的质心与俯视检查均通过。
2026-11-15 视觉修订：恢复原细边框半透明紧凑主栏；车辆为主标签，
相机原名为下方次级行，图像类型为弱化徽标；速度表几何保持。
已在实际 1280×720 窗口审核完整画面，RPC 回归通过，用户视觉验收待确认。
客户端移除终端输入循环，任务状态由控制程序通过 RPC 上报。
这不是飞行验收。后文为历史记录，
其中早期“未编译”和单车限制由本节及 RPC.md 替代。



开发前阅读 [Native capabilities and interface development reference](../../../docs/ProjectAirSim原生显示接口核查.md)，核对原生能力、来源版本、通信限制与本项目扩展边界。

`AerohubDisplay` is an independent UE 5.2 plugin. It uses public UE actor,
scene-capture and HUD interfaces, without modifying ProjectAirSim core code.
Its source lives here; generated binaries are installed in the external Blocks
project. The project must be rebuilt/repackaged to include it in a packaged sim.

Build and install (restart the editor once after installing a new module):

```bash
bash simulator/projectairsim/build_display.sh \
  ~/UE_installed_build ~/ProjectAirSim/unreal/Blocks/Blocks.uproject \
  log/projectairsim/display-plugin
```

Configuration is in `config/simulator/*.yaml`, under `display`:

```yaml
display:
  enabled: true
  vehicle: Drone1
  camera_enabled: false
  camera: DownCamera
  camera_width: 320
  speed_max_mps: 20
  port: 8991
```

The default layout keeps mission state and wall time in a compact top panel and
puts observed speed in a dedicated lower-right dial. `speed_max_mps` sets the
dial scale and can be changed without recompiling. The dial borrows the visual
idea of a large numeric readout, radial ticks and a moving needle from
XXLiu-HNU/Speed-Visualization, but is independently implemented with UE Canvas;
no source code or graphics are copied from that project.

`attach` writes the resolved display settings to the selected UE project's
`Saved/Aerohub/display.json`. The HUD reloads settings every 0.5 game seconds;
port changes require restarting Play. Set `projectairsim.project_resource` when
using a project other than Blocks. A manager-started packaged host gets its
configuration path through `AEROHUB_DISPLAY_CONFIG`, without changing the map.

The camera inset is not drawn in the current v5 layout. Camera capture settings
remain available for the simulator sensor bridge.

The HUD exposes a local HTTP interface (UE HTTPServer defaults to loopback):

| Request | Meaning |
| --- | --- |
| GET `/aerohub/status` | Vehicle presence, observed speed, timer and camera target state |
| POST `/aerohub/telemetry` | Manager-owned speed reader supplies vehicle and speed_mps |
| POST `/aerohub/mission/start` | Start wall-clock timer from idle/finished |
| POST `/aerohub/mission/finish` | Freeze a running timer |
| POST `/aerohub/mission/reset` | Reset display timer to idle |

The manager wraps this interface:

```bash
bash simulator/simulator.sh display --event status
bash simulator/simulator.sh display --event start
bash simulator/simulator.sh display --event finish
bash simulator/simulator.sh display --event reset
```

These events change the observation display only. They do not start/cancel a
planner, arm/disarm PX4, teleport a vehicle, or reset the simulator. A mission
client can emit them alongside its actual lifecycle operations. Invalid timer
transitions return HTTP 400. Repeated start while running does not reset time.
This interface supplements the existing ProjectAirSim RPC; world/object/camera
operations continue to use upstream APIs.

Current limits:

- Timer measures monotonic wall time, including pauses; the compact label is
  TIME. It is not a simulator-time benchmark.
- Speed is the magnitude of upstream ProjectAirSim ground-truth `twist.linear`
  in m/s, forwarded by `display_telemetry.py` using service-only RPC. It is not
  PX4 telemetry. Data older than one wall second displays `--`.
- Peak is accumulated since startup/reset. The top panel explicitly labels
  TIMER IDLE/RUNNING/FINISHED and WALL time; it does not report planner health.
  The gauge labels speed SIM GT and independently reports LIVE DATA, STALE DATA,
  WAITING DATA or NO VEHICLE. Stale data hides the current value, needle and colored fill. The review viewport is 1280×720.
- Presence does not mean PX4 is connected or ready.
- One local player/world per port; simultaneous PIE worlds need distinct ports.
- The plugin replaces the default HUD; projects with an existing custom HUD
  should integrate the drawing rather than enabling both independently.
- Editor, Development and Shipping compilation was checked on UE 5.2.1. Actual
  runtime checks used a standalone editor game window. The old packaged Blocks
  binary does not acquire this plugin without rebuilding it.

## Runtime validation (2026-11-12)

UE 5.2.1 standalone editor game, separate ports 9989/9990 and display 8992:

- Actual Blocks rendering, vehicle discovery and native RGB render-target display
  checked in a saved window capture.
- Native PX4 scene load and camera target discovery checked without PX4 flight.
- Actual image rendering checked with the upstream SimpleFlight sensor scene and
  native C++ ROS bridge, on separate ROS domain 189.
- Timer start/finish/reset, elapsed time increase, frozen finish, invalid transition
  HTTP 400, missing-camera handling and live camera selection checked.
- Final CLI-to-UE status query and active camera flag checked.
- Compact top status panel and lower-right speed dial visually checked at
  1280x720 in both idle and running display states; the camera inset was off.
- These initial checks preceded the moving-flight check documented below.
  Packaged Blocks runtime acceptance was outstanding at that point; see the 2026-11-13 package check below.

Local evidence: `log/projectairsim/display-validation/` (`hud-gauge.png`,
`hud-gauge-running.png`, `control.json`, `lifecycle.json`, `final-status.json`,
UE and bridge logs).

Latest flight check: native AS2 Takeoff 2 m → GoTo +3 m at 1 m/s → hold 6 s →
Land/disarm passed. Ground-truth HUD speed median during GoTo was 1.001 m/s,
peak 1.113 m/s, hold final 0.037 m/s and landing final zero. Telemetry shutdown
returned null current speed. Evidence: `log/projectairsim/display-flight/`.
This was a separate game instance; user-visible dynamic review in the intended
window remains outstanding. Earlier actor-difference spikes and a failed
connection attempt are retained in that evidence directory. Manager automatic
telemetry ownership has unit coverage; the successful flight used a manually
started reader. The old camera and actor-speed validation above is historical.


## Packaged course runtime check (2026-11-13)

The Linux Development package now includes the reviewed two-stage Blocks course v7
and this HUD, including both Nimbus Mono fonts. The separate runtime map is
`/Game/BlocksPlannerCourse`; reviewed editor maps remain preserved.
The static scale-reference drone and stale original-building reflection captures
are editor-only in the runtime copy.

Actual packaged Vulkan rendering at 1280×720, 24 obstacle objects, one unarmed
SimpleFlight robot at the green start, real ground-truth speed forwarding,
and HUD start/finish/reset plus invalid-transition HTTP 400 were checked.
No flight, PX4, ROS/AS2 or planner run was performed for this package check.
The launcher supplies `-noraytracing` to avoid a UE 5.2 Vulkan extension/device
creation failure observed with NVIDIA 580.178.04; accepted SunSky/SSGI lighting remains.

Build, launch and interface instructions: [course package](../course/PACKAGE_README.md).
Artifacts and validation evidence: `log/projectairsim/package-course-v7/`.

## Display iteration (2026-11-14)

The source and offline v6 preview clarify timer and telemetry semantics while retaining
the existing layout. Review images are in `log/projectairsim/display-preview/v6/`,
including a stale-data case at both resolutions. This iteration has not yet been
compiled or checked in UE or a packaged application. The development backlog is
[场景客户端开发清单](../../../docs/场景客户端开发清单.md).

## JSON-RPC source increment (2026-11-15, not runtime-validated)

The plugin source adds POST `/aerohub/rpc` on the existing display HTTP port.
This is an independent JSON-RPC 2.0 request/response endpoint, not the native
ProjectAirSim NNG transport. Send an explicit request id; notifications and batch
requests are not supported by this initial increment.

Implemented methods: `SetTitle` with `params.text`, `ShowMessage` with
`params.text` and optional `duration_seconds` (default 3, zero persists),
`ClearMessage`, and `GetDisplayState`. Text is limited to 512 UTF-16 code
units; durations must be finite and between zero and 86400 seconds.
Status includes title and currently visible message. Titles and messages are
drawn before the main HUD panels; the gauge's duplicate status label is removed.

Example request (after rebuilding the plugin):
```json
{"jsonrpc":"2.0","id":1,"method":"SetTitle","params":{"text":"Experiment 1"}}
```

This increment is uncompiled. It currently truncates drawn title/message text;
wrapping, multilingual font coverage, game-thread dispatch verification,
per-vehicle state/timer APIs, view identity and Global are still pending.
Existing Python tests do not validate this new UE endpoint.

### Vehicle state source increment

SetRunState now accepts:
```json
{"jsonrpc":"2.0","id":2,"method":"SetRunState","params":{"vehicle_id":"Drone1","state":"running"}}
```
Allowed states are idle/running/finished. The top badge displays this explicit
state, or -- before one is supplied; timer events do not set it. Telemetry is
stored by vehicle, rejecting the reserved __global__ identity. Automatic
view binding, per-vehicle timers and reset isolation are not complete yet.
The plugin's initial text-only increment passed BuildPlugin; the state
increment is validated separately in the build-state log.
