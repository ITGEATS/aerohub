# uav_aerovla_planner

当前 Registry 入口为 `aerovla_goto` / `uav_aerovla_planner.goto_node`，
订阅观测并调用原生 GoTo/Land；配置和生命周期见下文 GoTo 章节。
`aerovla_planner` / `planner_node` 仅保留旧时间化轨迹对照入口。

## 历史轨迹入口（非当前 Registry 链路）

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
  TravelUAV 目标描述）、`prompt_table`（string，预分词 prompt 表路径；板端无
  `transformers`，正式运行必须提供，文件随 aerovla 组件部署到
  `/data/aerohub/lib/aerovla/prompt_ids_blocks.json`，表内 `obj_desc` 与本参数
  不一致时报错）、`rate_hz`（默认 0.5，≈2 s 推理窗口）、`enabled`、
  `vehicle_id`、`max_input_skew_s`（0.1，同拍检查）、`inference_timeout_s`、
  `valid_from_lead_s`（0.05）、`max_consecutive_failures`（3，连续失败暂停循环）。

## 保真约定

数值公式、分支与 prompt 字符串逐字移植自上游（来源行号见模块 docstring）；
`policy.py`/`prompts.py` 零 ROS 依赖，主机可直接 pytest：

```bash
cd src/tool/navigation/planner/aerovla/ros2_adapter
uv sync --locked
uv run --locked pytest -q
```

依赖：`rclpy`、`sensor_msgs`、`geometry_msgs`、`std_msgs`、`std_srvs`、
`uav_inference_interfaces`、`uav_planner_interfaces`、numpy、scipy。
input_ids 两条路径：`prompt_table` 预分词表（板端标准，表来自
`log/aerovla/stage1` 验证约定，见 `deployment/airsim/config/aerovla/`），
或 openvla-7b tokenizer（需 `transformers`，`AEROVLA_TOKENIZER_PATH` 可配）。

主机测试依赖由本目录 `pyproject.toml` / `uv.lock` 管理，复用本目录 `.venv`；ROS 节点仍在已有 Humble 环境运行。

坐标边界：输入姿态为 ROS ENU←FLU，策略使用 NED←FRD。完整姿态转换为
`R_ned_frd = W_enu_to_ned @ R_enu_flu @ diag(1,-1,-1)`；水平航向为
`yaw_ned = wrap(pi/2-yaw_enu)`。输出时间化段保留 NED，由框架统一转换为 Behavior 的 ENU。

## 原生 GoTo 正式 Registry 入口（闭环验证待完成）

显式运行 `python3 -m uav_aerovla_planner.goto_node`（或 `aerovla_goto`）可试用逐步 GoTo
执行；需要现有 ROS/AS2 overlay 和本包。沿用原 node 的 goal、RGB、pose、inference 参数，
另需 `twist_topic`（默认 `/drone0/self_localization/twist`）、`goto_action`（默认
`/drone0/GoToBehavior`）。以独占 UAV 的 namespace 运行，并在该 namespace 下发布 `goal`。

Registry 默认已切换到此入口，输出契约为 `BehaviorExecution`；本入口不改 Behavior。原生 GoTo 成功后继续等待实测 position/yaw/speed
收敛，执行后取得新观测才推理下一步。大转向保持“先转向，再垂直移动，丢弃水平位移”。
取消响应只表示请求，`status.stop_confirmed` 表示新鲜低速确认；超时保留 STOP_FAILED。
有效 LAND（显式标记或完整三档数值还原后的近零动作）直接调用原生 LandBehavior；
只有 Action 状态 SUCCEEDED 且 land_success=true 才报告任务成功。缺失动作数值按
INVALID_ACTION 计错，不再以默认零动作触发降落。

`goto_position_tolerance_m=0.2`、`goto_yaw_tolerance_rad=0.1`、
`goto_speed_tolerance_mps=0.15`、`goto_stable_s=0.3`、`goto_max_state_age_s=0.5`、
`goto_step_timeout_s=30`、`goto_stop_timeout_s=10` 为可调 harness 参数。

框架解析 Registry 的 `BehaviorExecution` 契约后，仅管理 Tool 生命周期，不创建轨迹 Gateway 或
FollowTimedPath 会话。支持 START、PAUSE、RESUME、CANCEL 与 GET_STATUS；暂停保留进程，
取消先请求 Tool 停止，收到服务接受及停止确认后再退出。未确认停止前禁止新任务或恢复。
节点只接受该契约的 `resolved_config`，拒绝旧轨迹 resolved 配置；外部 goal 不能覆盖框架任务。
任务状态使用 `/planner/tools/task_<task_id，连字符转下划线>/status`，配套 cancel/resume Trigger。
本入口尚未完成真实 NPU 与飞行验收；过程与限制见 `dev/aerovla-goto/README.md`。

在真实 Humble/AS2 环境补编译本包与两组接口后，可先执行
`python3 tests/check_ros_goto.py` 检查真实消息类型、参数和节点构造；不发送运动目标。

`python3 tests/check_native_goto.py` 启动真实原生 GoTo（脚本路径适配现有 onboard 镜像），
以合成 FLYING/TF/Twist 和控制模式服务 stub 联测。已验证：原地 yaw 目标提前成功时，
原生发出最终 yaw 参考，实际 Tool 继续等实测 yaw 稳定后才推进。
该检查没有真实控制器、推理或飞行；完整动力学与异常矩阵仍未完成。

框架不把目标邻近或降落请求当作任务成功，等待 Tool 的 Land 成功结果。
`land_action` 默认使用部署 namespace 下的 `/LandBehavior`；`land_timeout_s=60`，
`land_speed_mps=0` 表示使用 AS2 默认速度，可通过 Registry 参数调整后两项。
`python3 tests/check_land_runtime.py` 使用真实 ROS Land 消息与合成 Action server 验证派发及结果；
该检查不运行原生 Land 插件或物理仿真。


## 代码职责

`observation_node.py` 提供共同的 ROS 观测与推理循环：配置加载、图像/位姿快照、
坐标转换、方向提示、prompt、推理请求与失败计数。执行入口在共用状态初始化后
通过 `configure_execution()` 配置各自资源，再由共用层启动框架任务。

`goto_node.py` 和历史 `planner_node.py` 分别继承该共同层，二者互不继承。
前者解释动作并调用 GoTo/Land，后者独立保留轨迹消息、滚动保持和参考交接缓存。
GoTo 不声明 `roll_horizon_s`、`valid_from_lead_s`，也不创建 trajectory publisher。
共用层不依赖 GoTo/Land 或时间化轨迹消息；`policy.py`、`prompts.py` 和 `actions.py`
分别保留策略数学、提示词和正式入口的输出校验。
