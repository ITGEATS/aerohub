# AS2 Navigation Module

公共 Python 调用层使用共享的 `NavigateToPose`、`NavigateToSemanticGoal` Action 与
`UpdateNavigationTarget` Service。服务端由对应 Planner 实现，各 Planner 声明支持的操作。

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

同一 Module 的重复调用会更新当前任务目标，不抢占其他客户端的任务。到达策略见下节。

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
`src/tool/navigation/planner/yopo/ros2_adapter/uav_yopo_planner/navigation.py`。两个 Planner
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


## Card v1 静态对接

| 实现 | Tool Card | 部署清单 | 公共操作 |
| --- | --- | --- | --- |
| SUPER | [super-single.yaml](../../../../cards/planner/super-single.yaml) | [component.yaml](../../../../deployment/airsim/demos/super_forest/component.yaml) | `navigate` |
| YOPO | [yopo-single.yaml](../../../../cards/planner/yopo-single.yaml) | [component.yaml](../../../../deployment/airsim/demos/yopo_single/component.yaml) | `navigate` |

两张Card均使用v1 `type: tool`，操作引用
`uav_planner_interfaces/action/NavigateToPose` 与同一 `.action` 参数Schema。
标准Action取消属于该任务协议，不另行声明旧的取消Service。
`lifecycle: system_managed` 指进程启停权限，不限制调用方取消自己提交的任务。
Card不复制参数值、动态就绪/停稳阈值或具体Action地址；部署清单
`runtime.public_interfaces.navigate` 保存当前Demo的地址与Schema绑定。

YOPO保留已有Card ID，因公共操作契约从Topic/Service改为Action，将Card及组件版本
更新为0.2.0；SUPER新增Card及清单为0.1.0。
SUPER卡片复用已有里程计、导航障碍点、控制器状态类型；YOPO沿用既有类型，
本次未新增能力类型或硬件类型。SUPER内部地图仅声明为Demo表示，
不冒充独立ROG-Map能力或查询接口。

当前仍由部署显式选择其中一个实现；同一UAV公共地址不能同时启动两个提供者。
此处只是静态契约对齐，没有新增自动发现、参数Schema加载器、运行时选择或跨进程资源锁。
原有Planner约束差异继续以实现、组件说明与验证记录为准。

对齐检查日志在 `log/navigation-card-v1/`。两张目标Card和部署绑定检查通过；
全仓校验仍因DiffPhys、YOLO26n-seg、GCOPTER、Object Detection四张旧卡未迁移到v1而失败，
本次检查与修改前错误集合一致，不表示全仓卡片已全部通过。

## 持续导航与目标更新（2026-11-15）

```python
nav = drone.navigation
nav.to_point(5., 0., 2., wait=False, arrival_behavior="hold", target_timeout=60.)
nav.to_point(7., 0., 2., wait=False)   # 同一个 Action UUID，不取消原执行
nav.to_point(10., 0., 2., wait=False)
nav.wait_until_arrived(timeout=30.)  # 到达后仍保留导航任务
nav.cancel()                       # 确认停止后结束任务
```

- `arrival_behavior="complete"` 是默认值，到达且停止确认后结束；`"hold"` 则保留会话，
  通过既有执行层停止/悬停，目标改变后重新获取执行。HOLD 必须 `wait=False`，
  使用 `wait_until_arrived()` 等待当前目标；`wait_to_result()` 等待整个任务结束。
- 活跃任务中重复 `to_point` / `to_pose` 仅更新目的地。未提供的任务策略继承；显式改变
  朝向模式、速度、容差、任务超时、到达策略或目标流超时会报错，必须结束旧任务再创建。
- `timeout` 仍是从任务创建开始计时的总时限，默认 300 ROS 秒；更新不延期。
  `target_timeout` 是距最后一次接受目标的 ROS 秒数，过期执行停止并返回 `TARGET_TIMEOUT`。
  Python HOLD 默认 5 秒；直接 Action HOLD 必须显式启用有限正值。
  Mission 即使目标不动，也必须在期限内重发目标；相同位置只续期，不触发重新规划。
- 更新服务地址 `<UAV>/navigate_to_pose/update_target`，请求包含当前 Action `goal_id`、
  严格递增的 `target_revision`（初始为 1）、目标和朝向模式。UUID 用于任务关联，不是鉴权凭据。
  非当前 UUID、旧版本、非法目标、停止中更新会被拒绝。没有无限待处理目标队列。
- `accepted` 表示目的地已接受，不是轨迹已生效。反馈 `target_revision` 表示已接受目标版本，
  `revision` 仍表示执行轨迹版本，`arrived` 表示已确认到达并处于 HOLD。
  当前**没有目标版本到实际生效轨迹的确认映射**，不能据此声称更新已由飞控执行。
- 运动中更新保留已有可执行轨迹与导航 Action。到达保持后的恢复允许创建新的底层
  FollowTimedPath 执行会话，不能把已结束的轨迹当作仍可 modify。
  SUPER 旧最终轨迹先结束时也必须先停止确认再恢复；YOPO 丢弃过期目标的推理结果，
  并用推理时冻结的目标计算 yaw。快速目标更新若持续快于推理，仍可能耗尽覆盖并安全停止。
- 更新不是视觉跟踪/看向目标功能；两算法已有的朝向与到达差异仍保留，特别是 YOPO 的 5m 阈值。
  默认一轮完整任务结束后再次调用会创建新任务；停止未确认时禁止重新启动。

### 构建与验证边界

Action 类型已有变更，新增 `unique_identifier_msgs` 依赖；必须重新编译公共接口、
NavigationModule、SUPER/YOPO Adapter，以及受影响的 FollowPath 执行消费者并显式加载
同一 overlay。不能混用旧生成类型或用旧 Demo 飞行记录证明新功能。
本次证据放在 `log/navigation-target-update/`；接口测试使用真实 ROS 传输和可控执行/推理替身，
不等价于真实 GPU 算法或 AirSim + PX4 SITL 动力学验收。持续更新、保持恢复及失效停止的
真实仿真闭环仍需单独验证。

本轮检查记录与未完成矩阵见 [目标更新接口验证](VALIDATION-target-update-20261115.md)。


### System Tool初版的运动占用

SUPER托管NavigateToPose已接入 `uav_motion_resource`，与System Tool起降在同车
命名空间和同一motion目录下互斥。需重建并加载新增依赖，并按
[System部署](../../../../deployment/system/README.md)挂载持久化目录。
停止未确认或进程退出后保留占用记录。本次仅修改SUPER入口；YOPO与其他历史
控制入口尚未接入，前述“没有跨进程控制权仲裁”仍适用于这些未覆盖入口。
当前改动尚未完成SUPER接口回归及飞行验收，不提升已有验证等级。

## Semantic navigation operation

Navigation exposes multiple operations; planner alternatives need not implement all of them.
`to_pose` uses `NavigateToPose`; `to_semantic_goal` uses the separate shared
`NavigateToSemanticGoal` Action at `/<UAV>/navigate_to_semantic_goal`.
`ready` continues to mean pose-operation availability; `semantic_ready` checks
semantic-operation availability. No automatic fallback between operations is performed.

```python
drone.navigation.to_semantic_goal("approach the orange ball", timeout=60.0, wait=False)
# Later: navigation.feedback / done / result; navigation.cancel() confirms stopping.
```

`description` must be non-empty and `timeout` finite and positive (monotonic seconds).
The current SPF alternative has **no arrival detector**: budget exhaustion yields
`TIMEOUT`, `success=false`, after stopping; it never claims successful attainment.
`wait=True` therefore returns False on a confirmed timeout; inspect `result.code`,
`result.reason` and `result.stop_confirmed` to distinguish timeout/cancel/failure.
An unsupported operation has no available server and fails explicitly.
Rebuild/source `uav_planner_interfaces` and this package together when adopting the new Action.
