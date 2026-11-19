# YOPO-Tracker Planner

单机 RGB-D 跟踪 Planner。锁定的 YOPO-Tracker 模型及目标滤波器选出曲线，ROS 2 Humble Adapter 加上制动尾段，通过默认 `FollowTimedPath` 执行。原生 Takeoff 和 Land 由任务调用。

- [组件清单](../../../../../deployment/yopo_tracker_humble/component.yaml)记录依赖、资源所有权和阶段状态。
- [Adapter 运行说明](ros2_adapter/README.md)包括构建、启动、停止与回归入口。
- [设计](DESIGN.md)说明上游链路、坐标与时间契约。

## 验收状态

状态为 `partial`。在先前验证基线上用 AirSim + PX4 SITL 运行原生 CUDA 模型时，移动目标驱动两版追踪轨迹实际生效。独立暂停 RGB-D 发布后，飞行器沿已载入的制动尾段减速，最终 Action 返回 `COMPLETED`。悬停连续低速 1.011 仿真秒，末速约 0.022 m/s，原生 Land 和 PX4 解除解锁通过。交付分支基于 Aerohub `dev`，镜像与接口回归和上述历史试飞分开核对。

三版连续追踪用例在交接前缺少可用的新轨迹，执行了上一版的制动尾段。最新一轮日志中的目标估计连续等步长外推，说明没有获得新的有效目标观测。长期持续追踪、相机正常时的目标丢失和跟踪精度待验收。

在当前开发基线上，复用本地 CUDA 环境构建的交付镜像以无宿主挂载方式启动正式节点。DDS 合成验证拒绝错时的 RGB-D、拒绝缺少同采样时刻的位姿或速度，并用配对成功的原始 stamp 进入真实 CUDA 推理。当前默认 FollowTimedPath 的两版交接回放通过。当前镜像的 AirSim 飞行留待复验。

回归入口位于 `replay/`。本地过程和保留证据归档在 `dev/yopo_tracker/`，日常构建输出保存在 `log/`。
