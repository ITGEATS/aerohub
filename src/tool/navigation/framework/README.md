# Planner 执行框架

默认 `execution_backend:=follow_path`：算法 Adapter 输出 TimedTrajectorySegment，
框架负责校验和机群调用协调；逐机 FollowTimedPath Behavior 负责采样与执行。

`execution_backend:=legacy_setpoint` 显式启用旧 FleetSetpoint 输出桥。
CAT-ORA PC Demo 可显式选择 follow_path；旧板端及 Swarm Demo 仍选择 legacy。
禁止同机同时开启两条执行链。

- `uav_planner_framework/`：配置、生命周期、校验、完成判定与 Action 客户端。
- `registry/`、`examples/`：算法 manifest、任务与部署配置。
- [公共接口](../interfaces/)：ROS 消息、服务和 Action。
- [FollowPath 扩展](../../../behavior/follow_path/)：时间化轨迹执行。
- [旧输出桥](../../../behavior/legacy_setpoint/)：仅用于显式旧链路。

AS2 启动器通过 `AS2_EXECUTION_BACKEND=follow_path` 选择扩展节点；未设置时保留
既有 Demo 的启动行为。使用新模式前必须构建安装扩展和接口包。
迁移状态与验证边界见 [交接文档](../../../../skill/temp/pr-main-layout/history/migration.md)。

## 执行状态观察

新后端发布 `planner/execution_state`（std_msgs/String JSON），字段包含 task_id、
backend、status、failure_reason、prepared、stop_requested 与 stop_confirmed。
prepared 仅表示服务可发现；stop_confirmed 只有全机群停止响应成功及后续模式反馈
均满足时才整体置真，不能将规划服务 CANCEL 获准视为停止完成。
该话题用于应用观察，不替代 FollowTimedPath 的 Goal/Feedback/Result。


## 单步 Tool 的原生 Behavior 执行

Registry `output: {type: BehaviorExecution, interface_version: 1}` 表示 Tool 自行调用原生
Behavior。当前仅支持单机；AeroVLA 默认使用该契约和 `uav_aerovla_planner.goto_node`。
框架保留配置解析、进程启动、任务超时、状态及生命周期管理；不创建轨迹 Gateway、轨迹超时或
FollowTimedPath 会话。既有轨迹类组件继续使用 `TimedTrajectorySegment`。

Tool 使用 `/planner/tools/task_<UUID，连字符转下划线>` 下的 status JSON、cancel/resume
Trigger；status 包含 task_id、generation、state、phase、last_error、stop_confirmed。每0.5秒心跳。
框架需要当前取消请求被接受及匹配任务、更新 generation 的停止确认；暂停保留进程，恢复要求已确认停止，
取消/失败保留进程直到停止确认，确认前不接受新任务。心跳默认5秒失联失败，启动容差可通过
`runtime_startup_timeout` 放宽；失联不能视为安全停止。

`planner/execution_state` 在该模式下报告 backend=tool、stop_requested、逐机 stop_confirmed
、land_requested 和 land_success。AeroVLA Tool 自行调用原生 Land；
只有 Land Action 成功后 Tool 才报告 SUCCEEDED/land_success=true，框架据此完成任务并收口进程。
请求降落或靠近目标不能代替成功结果。
这不是跨框架实例的资源仲裁；单框架仍须独占其 UAV。

开发接口验证：在已有 Humble/AS2 overlay 补编译后，执行
`python3 tests/check_tool_runtime.py --ros-args -p registry_root:=<registry绝对路径> -p resolved_dir:=<构建目录下的测试resolved目录>`。
该检查启动真实 AeroVLA 子进程并发布合成定位，不运行推理或发送运动目标，不代替飞行验收。
