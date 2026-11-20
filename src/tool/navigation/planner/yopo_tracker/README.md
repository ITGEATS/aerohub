# YOPO-Tracker Planner

单机 RGB-D 跟踪 Planner。锁定的 YOPO-Tracker 模型及目标滤波器选出曲线，ROS 2 Humble Adapter 加上制动尾段，通过默认 `FollowTimedPath` 执行。原生 Takeoff 和 Land 由任务调用。

- [组件清单](../../../../../deployment/yopo_tracker_humble/component.yaml)记录依赖、资源所有权和阶段状态。
- [Adapter 运行说明](ros2_adapter/README.md)包括构建、启动、停止与回归入口。
- [设计](DESIGN.md)说明上游链路、坐标与时间契约。

## 验收状态

状态为 `partial`。先前的 AirSim + PX4 SITL 故障用例以原生 CUDA 模型连续执行两版追踪轨迹。暂停 RGB-D 发布后，飞行器沿已载入的制动尾段减速，最终 Action 返回 `COMPLETED`。悬停连续低速 1.011 仿真秒，末速约 0.022 m/s，原生 Land 和 PX4 解除武装通过。

镜像的 DDS 合成回归覆盖错时 RGB-D、位姿和速度配对以及真实 CUDA 推理。FollowTimedPath 接口回归覆盖连续性拒绝和滚动交接。复验命令见 [Adapter 运行说明](ros2_adapter/README.md#隔离验证)。

### Blocks 追人试飞与 RGB-D 帧龄

2026-11-20 在 Windows 有屏 AirSim 1.8.1 Blocks + Docker PX4 SITL 中复飞。使用地图原生行人路线、FrontCamera 原始 RGB-D（320 × 192，相机桥请求频率 15 Hz）、原生 CUDA 模型和默认 FollowTimedPath。PX4 读回 `SENS_GPS0_DELAY=200` ms。先用原生 Takeoff 并确认悬停，行人开始移动时启动追踪。连续追踪时间按第一条轨迹生效至锁定制动计算。全程用例要求持续追到人物路线终点、受控停车并完成 Land 和解除武装。

| 帧龄与执行参数 | 生效的追踪版数 / 时长 | 首次中断 | 飞行收尾 | 运行标识 |
| --- | --- | --- | --- | --- |
| 默认 300 ms，`max_speed=1.0` m/s | 3 版 / 2.9 s | `RGBD_TIMEOUT`。两次取图 RPC 耗时 233 和 188 ms，新配对到达时已触发断流保护 | 未走完路线。无新增碰撞，Land 和解除武装通过 | `raw-rgbd-20261120-155846` |
| 实验 500 ms，`max_speed=1.0` m/s | 17 版 / 16.6 s | `BRAKE_RESERVE`。第 17 版生效后图像持续配对，7 次处理均返回目标估计，但没有选出新轨迹进入制动计算 | 未走完路线。制动确认、Land 和解除武装通过，无新增碰撞 | `raw-rgbd-20261120-160226` |

帧龄从 RGB 与深度中较早的采集时间戳起算。Adapter 用它检查配对观测是否可用、计算结果能否提交，以及追踪期间是否断流。取图、定位配对等待、原生处理和制动尾段计算共享这个时间预算。另一次使用 300 ms 和 `max_speed=1.5` m/s 的试飞（`raw-rgbd-20261120-154847`）中，第三版的三个候选在提交检查时已达 381–432 ms。该轮原生处理耗时 119–243 ms，Adapter 的制动计算耗时 22–99 ms。原生回调内部尚未逐段计时。

500 ms 实验中有 6 版在 336–492 ms 帧龄时提交并生效。最后一次中断仍有新鲜图像和目标估计，问题留在原生选轨。目标估计持续输出本身无法证明每帧均获得了有效检测。执行端按第 17 版的预留制动尾段停车，第 18 版是最终静止覆盖。默认帧龄仍为 300 ms，500 ms 仅在独立实验镜像中使用。

两轮 `max_speed` 均作为 FollowTimedPath 控制参数传入。PX4 记录的追踪段水平速度峰值分别为 1.36 和 4.40 m/s。实时轨迹包含独立的速度参考，`max_speed` 不能作为实际水平速度上限的验收依据。

| 验收项 | 结论 |
| --- | --- |
| 真实传感器到原生 CUDA 选轨、Adapter 到 FollowTimedPath、PX4 实际运动 | 两轮均有实飞证据 |
| 图像断流后制动、停车及降落 | 已覆盖 |
| 人物全程持续跟踪、图像持续时的独立失跟用例、跟踪精度 | 待验收 |

复核时按原始 RGB-D stamp 对齐 `camera-timing.jsonl`、`tracker-timing.jsonl` 和提交事件，再用 `mission/result.json`、便于读取的 `timing-summary.json` 与 `speed-summary.json` 交叉核对。原始 PX4 时序保存在 `px4.ulg`。这些试飞来自迁入当前分支前的开发镜像，部分辅助容器使用本机工作区挂载，证据范围为 Windows SITL 开发试飞。

回归入口位于 [replay/](replay/)。试飞日志按运行标识保存在本地 `log/yopo-tracker/airsim-blocks-flight-01/`。
