# FlowPilot 单机滚动规划

准备模型见 [Inference worker](../../../../basic_function/inference/models/flowpilot/README.md)。ROS 2 服务入口是 `python3 -m uav_inference.flowpilot_node`，规划器入口是 `ros2 run uav_flowpilot_planner flowpilot_planner`。两者设置相同 namespace 和 use_sim_time。服务的 endpoint 指向 worker /infer 所在主机的根 URL。

规划器订阅 flowpilot/depth、flowpilot/camera_info、self_localization/pose、self_localization/twist 和 flowpilot/goal。深度使用 DepthPlanar。固定系目标为 PoseStamped。速度历史用于局部二次拟合，给出加速度和 jerk。姿态采用与观测配对的 AS2 定位。camera_pitch_deg 按 base_link 下绕 Y 轴的右手旋转填写，必须与 optical_frame 的 TF 一致。图像和目标都在同一个无人机命名空间内。

输出沿用 FollowTimedPath 的接纳、修改、取消和停止反馈。适配器从观测时刻开始计算原曲线，对未来交接时刻重新采样，保留剩余时间域。点消息支持 P/V/A 与 yaw，jerk 保留在模型响应。默认到达条件沿用执行入口的 5 米半径。ROS 调用示例见仓库 deployment/airsim/demos/flowpilot_single，数值对照与未覆盖项见 [验证记录](../../../../../deployment/airsim/demos/flowpilot_single/validation.md)。

能力边界见 [能力卡片](../../../../../cards/inference/flowpilot.yaml)。字段、坐标与交接约定及分层复验入口维护在 [深度轨迹模型的观测对照](../../../../../skill/uav-platform-sensor-adaptation/references/flowpilot-depth.md)，模型准备流程归 [CUDA 模型迁移 Skill](../../../../../skill/model-migration-cuda/SKILL.md)；其中数值仅适用于固定提交 f29f1bf。

本规划器在 `uav_flowpilot_planner` 包内独立实现 FollowTimedPath 的接纳、修改、取消与停止生命周期：`lifecycle.py` 与 `core.py` 以 YOPO 参考规划器（MIT）为蓝本派生，保留来源与许可证声明；`node.py` 只负责 FlowPilot 专属的请求组装、曲线重建与事件字段。构建与运行不 import、不依赖 `uav_yopo_planner`，也不修改 YOPO 包的任何已跟踪文件。

## 观测、期限和交接

AS2 的权威 Pose/Twist 按同一采集时间配对，再匹配深度时间。加速度与 jerk 使用严格递增、有覆盖的速度历史拟合，深度重复或倒序帧丢弃，仿真时钟回退清空观测基线。ROS 服务将固定数组转换为 JSON 原生数值，整帧无有效深度时拒绝请求，稀疏坏点仍按基线填 20 m。

`command_speed_mps` 是模型的期望速度，缺省沿用 `max_speed`；后者保留执行配置语义。`initial_plan_timeout_s` 缺省 1 s，约束首个计划一直未就绪的情况。单次推理超过原有 `inference_timeout_s` 后立即请求停止并解除本地 ROS future 等待，worker 的 GPU 占用仍由现有 runtime 管理。

交接前用已接纳的执行参考检查新段首点 P/V/A 和 yaw，事件 `handoff` 记录误差和门限。位置、速度、加速度、航向门限参数分别为 `handoff_position_tolerance_m`、`handoff_velocity_tolerance_mps`、`handoff_acceleration_tolerance_mps2` 和 `handoff_yaw_tolerance_rad`，缺省 0.15 m、0.5 m/s、0.5 m/s²、0.15 rad。部署时与 FollowTimedPath 的实际门限一致核对。航向从旧参考接续，`max_yaw_rate_rad_s` 缺省 π/2 rad/s。平移曲线保持原模型表示，交接拒绝沿用已有失败计数及覆盖到期收口。首次提交检查新段首点与实测位置。

点时间先量化到 ROS 纳秒网格，再用于插值检查。`execution_sample()` 与 FollowTimedPath 的分段五次插值对照，`sample()` 用于生成原 Bernstein 曲线的采样值。回归覆盖首个推理失联、取消、坏帧、错误响应、连续性拒绝后的参考保留和恢复。跟踪和实际避障验证状态见验收记录。
