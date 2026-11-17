# Aerohub Scene Control

独立 UE 5.2 Runtime 插件，负责街区仿真人物的播放启停；不修改无人机 HUD、
不解锁/控制无人机，也不是 Mission 或通用资源仲裁层。当前仅支持固定 `runner`。

## 运行前准备

工程启用本插件及 Sequencer；地图中恰好有一个带 `WestRunner` 标签的人物，
以及一个带 `WestRunnerSequence` 标签的 LevelSequenceActor。
序列设置 `auto_play=false`，保留 `loop_count=-1`；人物初始动画暂停。
本仓库街区通过 `simulator/projectairsim/weststreet/prepare_actor_control.py` 保存上述配置。

WorldSubsystem 在 Game/PIE 启动时暂停并求值至第 0 帧，冻结人物 SkeletalMesh 动画。
找不到目标或标签重复时返回 unavailable，不根据任意客户端名字搜索/控制车辆。

## RPC

默认地址：`http://127.0.0.1:19892/aerohub/scene/rpc`。
独立于 HUD 的 `:19891/aerohub/rpc` 与 ProjectAirSim 的 NNG 服务。
环境变量 `AEROHUB_SCENE_CONTROL_CONFIG` 可指向 `{"port":19892}` 配置。
UE HTTPServer 默认本地监听；本接口仅面向受信任本机调用，不对公网开放。

```json
{"jsonrpc":"2.0","id":1,"method":"ControlActor","params":{"actor_id":"runner","event":"start"}}
```

- `start`：首次启动或从暂停位置续跑，不复位。平滑启动：默认1秒内将序列播放速率从0升到1；原5m/s路线和动画同步变速。
  重复start不重新计时，暂停后再次start会重新加速，循环过起点不会重复加速。
- `stop`：暂停并保持当前位置、序列时间及动作姿态，重复调用不复位。
- `reset`：暂停并回到第 0 帧/起点。
- `GetActorState`：params 为 `{"actor_id":"runner"}`。

状态包含 available、state、time_seconds、duration_seconds、location_ue_cm、speed_mps，
以及play_rate、startup_ramp_seconds、starting（当前是否处于加速段）。
位置单位为 UE 厘米，速度由真实位置差分获得；它不进入无人机 HUD 的速度表。
路口处差分速度受曲率/采样时序影响，不将其声称为精确恒速证据。
非法方法、actor_id 或 event 不改变目标状态。

## 构建与安装

```bash
"$UE_ROOT/Engine/Build/BatchFiles/RunUAT.sh" BuildPlugin \
  -Plugin="$PWD/simulator/projectairsim/unreal/AerohubSceneControl/AerohubSceneControl.uplugin" \
  -Package="$PWD/log/projectairsim/scene-control-build/AerohubSceneControl" \
  -TargetPlatforms=Linux
```

将产物安装到目标工程的 `Plugins/AerohubSceneControl`，确认与引擎 BuildId 相同后
在 `.uproject` 启用。已存在的安装副本先核对差异，不盲目覆盖。新模块需要重启一次。
无需烘焙/打包地图即可使用现有 UE `-game` 预览。

## 已验证

2026-11-16，UE 5.2.1/Linux，BuildPlugin 成功；真实运行验证默认暂停、实际位移、
冻结、续跑、复位、重复 start/stop、非法目标/事件/方法拒绝均通过。
运行检查入口 `simulator/projectairsim/weststreet/validate_actor_control.py`。
实际飞行联调中，只有确认无人机起飞到约 3 米后才调用人物 start，落地后调用 stop。
证据位于 `log/projectairsim/weststreet-flight/`。不是 PX4/AS2 或真机验收。


## 平滑启动实现与验证

启动加速留在UE已有WorldSubsystem中：每帧按世界DeltaSeconds计算
`play_rate = a*a*(3-2*a)`，其中`a=clamp(elapsed/startup_ramp_seconds,0,1)`。
只调整已有LevelSequence的播放速率，不重写位置曲线，不逐帧远程发RPC，不增加线程。
默认1秒，速度及加速度在两端连续；停止/复位仍立即暂停。
配置文件`AEROHUB_SCENE_CONTROL_CONFIG`可设置`startup_ramp_seconds`（0–10秒），
0保留瞬时启动以做压力测试；修改配置后重启场景。客户端API和调用层次不变。
这不是新增走路/起跑动画，原跑步动画会随整条序列一起减速。

UE5.2.1/Linux真实场景已验证1秒渐增播放速率、实际巡航速度中位5.00001m/s、
重复start不重置、stop冻结、resume重新加速和reset回起点。
检查入口：`simulator/projectairsim/weststreet/validate_startup.py --output <证据文件>`，
先单独启动有该插件的场景。该检查不控制无人机，不替代持续追踪飞行验收。
