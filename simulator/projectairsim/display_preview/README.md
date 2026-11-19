# HUD visual preview

Render deterministic review images without starting Unreal, ProjectAirSim,
PX4, ROS or a desktop session:

```bash
python3 simulator/projectairsim/display_preview/render.py \
  --config config/simulator/default.yaml \
  --output log/projectairsim/display-preview/v5
```

The command generates idle, running and finished views at 1280x720 and
1920x1080. It reads the same `display.vehicle` and `display.speed_max_mps`
settings used by the runtime HUD. The synthetic background is only for judging
contrast, scale and occlusion; it is not a simulation screenshot.

Keep each visual iteration in its own directory under
`log/projectairsim/display-preview/` so that designs can be compared without
rerendering or overwriting an accepted version. These generated review images
remain ignored by Git with the rest of `log/`.

This is a visual design harness, not a second runtime implementation. Accepted
styles must still be implemented and checked in the UE plugin. Runtime data,
animation, DPI/font differences and actual scene composition require a later UE
Play review.

The approved UE reference is `v5/running-1920x1080.png`. The runtime HUD
uses the same proportions, with 1280×720 selected as the current review viewport. Nimbus
Mono regular and bold are shipped with the plugin, including their license,
so the packaged application does not depend on system font installation.
Peak speed is observed since display startup or reset; it is not the scale
maximum. The preview uses fixed sample values, while UE uses upstream ground-truth kinematics
and display timer events.

The 2026-11-14 v6 source iteration adds explicit TIMER/WALL labels, SIM GT
speed attribution and an independent data freshness label. It also renders a
stale-data case with no current value, needle or colored fill. v6 images are
offline review material; the previously approved v5 remains the UE reference
until the updated plugin is built and reviewed in an actual UE window.

2026-11-15 当前运行版已完成紧凑主栏、车辆/相机/格式层级、WALL 可读性、
左右双标题及 ControlRun 生命周期。上文 v5/v6 为历史视觉参考；当前真值以
UE 插件及实际窗口为准。离线 renderer 仅覆盖部分 HUD 样例，不覆盖双标题、
实际多机相机和完整 RPC 状态。交付状态见 docs/场景客户端开发清单.md。
