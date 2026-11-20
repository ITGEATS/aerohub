# uav_aerovla_planner

AeroVLA 迁移阶段 3：板端运行的 Planner ROS 2 包（Tool 层）。订阅两路相机图与
位姿，组装预分词 prompt，调用板端推理 service
`inference/aerovla/infer`（`uav_inference_interfaces/InferAeroVla`），解析离散
动作 bin 并发布时间化轨迹（`uav_planner_interfaces/TimedTrajectorySegment`）。

## 运行

```bash
ros2 run uav_aerovla_planner aerovla_planner
# 或（开发态，包在 PYTHONPATH 上）
python3 -m uav_aerovla_planner.planner_node
```

## 接口

- 订阅：`~/front`、`~/down`（`sensor_msgs/Image`，bgr8）、`~/pose`
  （`geometry_msgs/PoseStamped`，机体状态）、`~/goal`（`PoseStamped`，目标位置；
  入口范式沿用 yopo adapter 的 `goal` 话题 + `cancel` Trigger 服务）。
- 发布：`~/trajectory`（`TimedTrajectorySegment`）、`~/status`（JSON `String`）。
- 服务客户端：`~/cancel`（`std_srvs/Trigger`，停止并发布空轨迹）。
- 参数：`service_name`（默认 `inference/aerovla/infer`）、`target_position`
  （double[3]，GT 目标——oracle 依赖，卡片如实声明）、`obj_desc`（string，
  TravelUAV 目标描述）、`rate_hz`（默认 0.5，≈2 s 推理窗口）、`enabled`、
  `vehicle_id`、`max_input_skew_s`（0.1，同拍检查）、`inference_timeout_s`、
  `valid_from_lead_s`（0.05）、`max_consecutive_failures`（3，连续失败暂停循环）。

## 保真约定

数值公式、分支与 prompt 字符串逐字移植自上游（来源行号见模块 docstring）；
`policy.py`/`prompts.py` 零 ROS 依赖，主机可直接 pytest：

```bash
python -m pytest tests -q
```

依赖：`rclpy`、`sensor_msgs`、`geometry_msgs`、`std_msgs`、`std_srvs`、
`uav_inference_interfaces`、`uav_planner_interfaces`、numpy、scipy；
分词需 `transformers` 与 openvla-7b tokenizer（`AEROVLA_TOKENIZER_PATH` 可配）。
