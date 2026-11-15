# AS2 Navigation Module

公共 Python 调用层只依赖 `uav_planner_interfaces/action/NavigateToPose`，不依赖具体
Planner。实现服务端放在对应 `src/planner/<planner>/` 下；目前 SUPER 提供首个实现。

## 加载与调用

先加载 ROS、AS2 和本包的 install overlay，并将安装的模块目录加入搜索路径：

```bash
export AS2_MODULES_PATH="$(ros2 pkg prefix uav_planner_python_api)/share/uav_planner_python_api/modules${AS2_MODULES_PATH:+:$AS2_MODULES_PATH}"
```

```python
import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase

rclpy.init()
drone = DroneInterfaceBase('drone0', use_sim_time=True)
drone.load_module('navigation')
try:
    # 此处假定平台已起飞，算法及执行组件已部署、就绪。
    accepted = drone.navigation.to_point(0., 4., 2., wait=False, timeout=60.)
    succeeded = drone.navigation.wait_to_result(timeout=75.)
    result = drone.navigation.result
    print(result.code, result.reason, result.stop_confirmed)
finally:
    drone.navigation.cancel()  # 活跃任务须确认停止，再销毁客户端。
    drone.shutdown()
    drone.destroy_node()
    rclpy.shutdown()
```

阻塞调用默认等待最终成功/失败并返回 bool；`wait=False` 返回值只代表接受。
`.feedback` 是最近反馈，`.done` 表示最终结果已到达，`.result` 和
`.result_status` 分别提供业务结果与 ROS Action 状态。尚无结果时返回 None。
`.cancel_async()` 只发送取消请求并返回其确认 Future，取消完成仍看最终结果；
没有活跃任务时返回 None。`.cancel()` 等待任务结束及停止确认；未确认则抛出异常。阻塞方法用于应用线程，
不能在该 DroneInterface executor 的回调中调用。

`drone.navigation(target, ...)` 接收 PoseStamped；`to_point(x,y,z,...)` 默认 frame
为 `<drone_id>/odom`，明确不约束终端朝向。`to_pose(target,...)` 默认要求完整朝向，
实现不支持时返回 UNSUPPORTED，不能静默降级。参考系不自动转换。

## 公共 Action

地址为 `<UAV namespace>/navigate_to_pose`。目标位置、朝向模式与可选约束分别表达：

- `target`：带 frame 的 PoseStamped；位置必须为有限值。
- `orientation_mode`：UNCONSTRAINED、KEEP_YAW、TARGET_YAW、TARGET_ORIENTATION。
  未约束朝向时四元数不构成约束，不能以默认四元数推断用户要求。
- `use_max_speed/max_speed`、`use_position_tolerance/position_tolerance`、
  `use_timeout/timeout`：仅启用时读取对应值；单位分别为 m/s、m、s，均须有限且为正。

反馈包括阶段、实际位姿、剩余距离及轨迹 revision。结果包括 success、code、reason、
最终位姿及 stop_confirmed。ROS Goal UUID 由标准 Action 提供；接口不指定算法。

首版没有抢占、暂停恢复或目标更新。同一客户端仍在运行时再次调用会抛出异常；
服务端忙时拒绝新 Goal。错误请求可由服务端接收后立即返回带原因的失败结果，且
不会产生运动。实现不得将不支持的显式要求当成成功。

## SUPER 当前能力

以 `navigation_action:=true` 启动 SUPER Adapter。只接纳 UNCONSTRAINED 与 KEEP_YAW；
现有执行保持接管航向，尚未实现目标 yaw/完整姿态约束。显式 max_speed 只允许与
部署值一致，不改变 SUPER 规划参数，也不保证动力学瞬时速度严格不超限。

默认目标容差沿用 Adapter 的 goal_tolerance（Demo 为 1.5m），实际速度小于 0.3m/s，
保持 2s 且 FollowTimedPath 成功。任务超时默认 300s，使用 ROS 时钟；客户端等待预算使用单调墙钟，可通过
`wait=False` 后的 `wait_to_result(timeout=...)` 单独设置。成功、取消和执行失败都复用
既有停止机制，停止等待预算 8s；STOP_UNCONFIRMED 表示不能确认停止，后续任务
仍受既有执行静止检查约束。此结果不等于降落或解除武装。

常见 code：SUCCEEDED、CANCELED、INVALID_GOAL、UNSUPPORTED、NOT_READY、
EXECUTION_FAILED、TIMEOUT、STOP_UNCONFIRMED、INTERNAL_ERROR。
诊断来源和失败细节保存在 reason 中。

## YOPO 对照实现

YOPO 复用同一 `NavigateToPose` 和上述 `drone.navigation`，以
`navigation_action:=true` 启动 `uav_yopo_planner`；具体封装位于
`src/planner/yopo/ros2_adapter/uav_yopo_planner/navigation.py`。两个 Planner
须择一部署到同一 UAV 的公共地址，当前没有运行时选择算法或跨进程控制权仲裁。

| 约束/行为 | SUPER | YOPO |
| --- | --- | --- |
| 不约束终点朝向 | 支持，保持接管 yaw | 支持，沿原参考轨迹调整 yaw |
| KEEP_YAW | 支持 | UNSUPPORTED |
| 目标 yaw / 完整朝向 | UNSUPPORTED | UNSUPPORTED |
| 到达条件 | 部署容差、低速持续 2s、执行完成 | 保留原始距离 < 5m 后停止流程 |
| 显式位置容差 | 使用请求值 | 仅接受 ≥5m，仍按原始 <5m 停止 |
| 显式 max_speed | 仅接受部署值 | 仅接受部署值 |
| 确认停止 | 既有执行客户端 quiescent，8s 墙钟预算 | 原 stop ACK、控制模式、新鲜状态、速度 <0.2m/s 持续1 ROS秒，10s墙钟预算 |

YOPO 的默认任务超时同为 300 ROS 秒。成功结果在原有 STOPPED 后才返回；
FAULT 返回 STOP_UNCONFIRMED，其他原始失败原因保存在 reason。
启用公共入口时，旧 `yopo/goal` 和 `yopo/cancel` 拒绝修改任务；未启用时保留原有行为。
SUPER 与 YOPO 工作站 Demo 均通过 `drone.navigation.to_point(...)` 进入公共 Action，
取消和结果读取也使用共享 Module。YOPO 的独立恢复命令可经标准 Action cancel 服务
取消此前客户端遗留的任务。Module 与 YOPO 的协议回归由
`deployment/airsim/demos/yopo_single/test_navigation_interface.py` 验证。


## 阶段性验收与后续细化

当前阶段已完成公共 Action / AS2 Module、SUPER 与 YOPO 各自适配，以及两套 Demo
正常导航入口统一。接口回归与实际仿真证据见各 Demo 验证记录。
YOPO 原始5m到达触发与最终停稳误差的区别仍保留；统一结果结构不代表两种算法
已经提供相同成功承诺。长程记录来自直接Action调用；补齐Module后的Demo已验证短程和取消。

下一阶段待办：

- [ ] 统一成功语义：目标范围、执行完成与最终停稳。
- [ ] 明确参数契约：默认值、单位、坐标系、约束和不支持时的处理。
- [ ] 明确公共就绪检查与Planner专属检查的归属。
- [ ] 提取共用任务管理：接纳、互斥、超时、取消、反馈与结果。
- [ ] 保留算法专属适配：目标转换、规划调用、轨迹生成与滚动更新。
- [ ] 使用SUPER、YOPO回归细化后的成功、取消和异常路径。
