# WestStreet：PC 模拟算力板验证

运行链为工作站 ROS 任务容器 → PC 上的 onboard_computer 容器 → AS2 原生 Behavior / 控制器 / 平台 → flight_board 容器中的 PX4 SITL → 主机 ProjectAirSim。无人机状态由 PX4 提供。HUD 显示无人机，Global 可观察已配置的行人跟随相机。

复用固定镜像 `onboard_computer:v0.1.1`、`flight_board:v0.1.1` 和 `osrf/ros:humble-desktop-full-jammy`。无需重新打包场景；使用已安装插件的 UE5.2 工程以 `-game` 启动。

`run.py` 要求已有通过 PX4 地面检查的 manager profile，复用机器人传感器及磁场配置；另指定 UE 根目录、WestStreet uproject、工作站接口 overlay 和全新证据输出目录：

```bash
python3 deployment/airsim/demos/weststreet/run.py \
  --base-profile /path/to/verified/profile.json \
  --ue-root /path/to/UE5.2 \
  --project /path/to/WestStreet/PAirSim.uproject \
  --workstation-overlay /path/to/workstation \
  --output "$PWD/log/projectairsim/weststreet-framework/run-new" \
  --ground-only
```

overlay 内应存在 `ws/install/setup.bash`，由相同版本的 `as2_msgs`、其 `geographic_msgs` 依赖在工作站镜像内通过 colcon 构建。容器内构建位置应为 `/work/ws`，源仓库挂载为 `/repo`。AS2 来源为仓库锁定子模块；不要混用其他版本的消息定义。

移除 `--ground-only` 即执行飞行：重置行人并暂停 → 解锁 / offboard → AS2 Takeoff 3m → 确认实测高度 → 启动行人 → AS2 GoTo 沿初始朝向前飞 5m → 实测速度小于 0.25m/s 持续 2s → AS2 Land → 确认 disarm → 暂停行人。任务客户端不发送仿真运动指令。此短程测试不代表街区避障或无碰撞验收。

场景需包含 `/Game/Aerohub/WestStreetCircuit` 及 AerohubDisplay、AerohubSceneControl 插件。人物使用统一 `ProjectAirSimExtensionClient(scene_port=19892)` 的 `reset_actor/start_actor/stop_actor/get_actor_state`。RPC 失败会记录任务错误，不冒充飞行成功。

本入口使用 DDS domain 197、PX4 instance 30 / system 31，及端口 4572、14552、14592、18991、19889–19892；运行前保证这些资源空闲。容器名含本次时间戳。失败证据包括 `mission.log`、`onboard.log`、`px4.log`、`mission/result.json` 和 `cleanup.json`。任务构造前失败可能没有 result.json，应先读 mission.log。

清理前读取实际 PX4 解锁状态；无法确认 disarm 时保留仿真器和算力板执行链用于恢复。不要在无人机仍解锁时强制结束整个链路。日志保留每次运行的成功、失败及未完成状态。

## 2026-11-16 实测

`log/projectairsim/weststreet-framework/ground-02/` 地面检查通过。
`log/projectairsim/weststreet-framework/flight-04/` 完成框架短程飞行：
Takeoff、GoTo、Land 均返回成功；实测起飞高度 2.947m，前飞目标 5m，
停稳后端点误差 0.087m、速度 0.045m/s，最终 PX4 arming_state=1。
人物起飞后启动，最终在路线 19.189s 处暂停；本轮容器和场景已停止。
统一客户端 8 项测试通过。

降落后的 `set_offboard_mode(false)` 被原生平台拒绝（该平台不允许算力板切回 MANUAL）；
该返回记录为 release_error，实际 PX4 disarm 已另行核实，不以此服务返回判定着地/解锁。
首轮飞行因模式切换期间状态超时失败，使用 PX4 降落命令恢复，证据保留在 flight-01，
不计为框架验收成功；flight-02/03 在启动端口检查阶段失败。
