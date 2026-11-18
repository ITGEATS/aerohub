# YOPO 公共导航接口对照（2026-11-15）

本轮是已有 YOPO 的公共调用层适配，保留算法、模型、ROS 推理接口、参考曲线、
滚动 FollowTimedPath 和原有 stop gate；不宣称重新完成算法复现或真机验收。后续按用户要求补做的单次长程检查见文末。

## 范围与检查矩阵

公共 Action 与 Python Module 沿用 SUPER 实现，不新增 YOPO 专属上层接口。
新增服务端在 `src/tool/navigation/planner/yopo/ros2_adapter/uav_yopo_planner/navigation.py`，
`navigation_action` 默认关闭，Demo 显式开启。初版工作站 Demo 直接使用公共 ROS Action（后续已补齐 Module 调用，见文末）；
真实 AS2 `drone.load_module('navigation')` / `drone.navigation` 与 YOPO 的连接在接口测试中验证。
两种方式使用同一个 Action 服务端。

检查矩阵包括：外部工作区构建、原有曲线/采样数学与 ROS 生命周期回归、公共 Module/Action
的就绪拒绝、约束拒绝、连续任务、互斥、旧入口保护、取消、超时、执行失败和停止无法确认；
实际 CUDA + 有屏 AirSim/PX4 SITL 的地面就绪、森林短程成功、取消及输入中断。
工作进程退出、长程、动态障碍、多机、RoboPi、真机不在本次新增包装层飞行验收范围。
这些已有算法/系统能力仍以原验证文档为准，不因本轮通过而扩大声明。

## 构建和接口验证

- 复用 `onboard_computer:v0.1.0` / 工作站 Humble / `flight_board:v0.1.0`，没有重建共享镜像。
- 通过现有 `build.sh` 补编译 Planner、公共接口与推理接口；工作站
  `prepare-workstation` 增加编译 `uav_planner_interfaces`，不复制 ROS 二进制。
- `log/yopo-single/navigation-build.log`：4包构建通过。
- `log/yopo-single/navigation-ground-build.log`：2包工作站接口构建通过。
- `log/yopo-single/navigation-math-02.log`：原曲线与执行采样数学6项通过。
- `log/yopo-single/navigation-legacy-regression.log`：原 ROS 生命周期通过，包括
  rolling revision、拒绝不改参考、过期响应丢弃、单个在途推理、stop gate、时钟回退。
- `log/yopo-single/navigation-interface-final.log`：公共调用检查7组通过（含就绪拒绝、约束拒绝、连续到达、忙时/旁路/取消、超时、执行失败、停止失败）。
- `test_navigation_interface.py` 使用真实 ROS Action、实际 AS2 Module、生产 YOPO node，
  推理与执行端为确定性测试替身，不作为真实模型或飞行证据。

测试中发现 Action 回调与原 timer 并发时，连续任务的启动可能和上一任务的终态观察竞争。
最终实现把公共任务启动/校验安排到与原 node timer 相同的回调组，不修改原规划逻辑；
接纳新任务的预留位由锁保护。早期失败日志保留为 `navigation-interface*.log`。
早期还出现测试替身在单线程执行器中阻塞和缺少传感器就绪等待的问题，修正测试调度后再测。

## 实际仿真证据

实际输入为 AirSim DepthPlanar/CameraInfo、AS2 状态；真实独立 YOPO worker 使用 CUDA，
模型身份与镜像 ID、源码摘要保存在每轮 `source-manifest.json` / `images.json`。
本轮模型身份为 `09ec31094ed83e09702efc1facf18076564ebbe1afc164960c41036e16ba229f`，
`ros-reference-u8-inpaint-v1;velocity=2.0`。

初轮 `dev/yopo-single/evidence/navigation-sitl-01` 地面与短程成功；最终代码重新启动全新
`dev/yopo-single/evidence/navigation-sitl-final`，森林目标 `[0,15,2]` 返回
`SUCCEEDED / ARRIVED_5M / stop_confirmed=true`，降落、解除武装通过，无碰撞；
真实深度中心带15m内障碍像素204。该指标只说明场景中有障碍观测，不证明动态避障能力。

结果中的 `offboard_release_error` 是已解除武装后显式释放 offboard 被平台拒绝，
如实保留；飞行、停稳、Land 与 disarmed 均已通过。它不应被隐藏成“所有后处理无错误”。

实际取消用例 `dev/yopo-single/evidence/navigation-cancel/cancel/result.json`：至少3个滚动
revision 后取消，公共结果 `CANCELED / CANCELED / stop_confirmed=true`；
停稳、Land、disarmed通过，无碰撞，未误报到达。

实际输入中断用例 `dev/yopo-single/evidence/navigation-stale/stale/result.json`：仅暂停本轮
sensor容器，公共结果 `EXECUTION_FAILED / STALE_OR_INVALID_INPUT / stop_confirmed=true`；
停稳、Land、disarmed通过，无碰撞，未误报到达。故障注入与恢复记录在同轮证据中。
三个最终飞行用例汇总于 `dev/yopo-single/evidence/summary/navigation-validation-summary.json`。
本轮专属容器、场景和 CUDA worker 已回收，未停止其他既有容器。

## 行为边界

YOPO 原到达条件为距离 <5m，再走原停止确认；不承诺 SUPER 的1.5m到达精度。
只接受 UNCONSTRAINED；目标朝向和 KEEP_YAW 返回 UNSUPPORTED。
显式容差 <5m 返回 UNSUPPORTED；≥5m仍保留原5m条件。
显式速度仅允许等于部署值，不能将其解读为动力学瞬时硬限速。
FAULT 返回 STOP_UNCONFIRMED，不将 stop ACK 等同于停稳；取消和其他失败保留原始原因。

实际飞行后的适配层不负责 Land；工作站应用沿用原有独立恢复流程。

## 公共导航入口的长程 forest 复核

用户随后要求长程验证。本次先固定目标 `[-37.57,231.76,6]`（约235m）、
原部署速度2m/s、300s任务预算、原始5m到达与stop gate，先通过地面就绪检查，
再执行一次全新有屏forest实例。未修改算法、控制器、模型、接口或门槛。
预声明检查保存在 `dev/yopo-single/evidence/navigation-forest-long-01/long-test-contract.json`。

实际 CUDA YOPO 经公共 NavigateToPose、滚动 FollowTimedPath 和 PX4 SITL 完成：

| 检查 | 结果 |
| --- | --- |
| 公共导航结果 | SUCCEEDED / ARRIVED_5M / stop_confirmed=true |
| 真实深度、模型及CUDA身份 | 通过，见 source-manifest.json；起飞前中心带障碍像素204 |
| 导航至确认停稳 | 126.702 ROS秒，126.920墙钟秒 |
| 接受的轨迹 revision | 343 |
| 采样航迹累计长度（含停止段） | 243.100m |
| RUNNING速度中位 / 峰值 | 1.990 / 2.379m/s；部署速度不是瞬时硬限速 |
| 首次 STOPPING 距目标 | 4.960m |
| 观测最小距目标 | 3.867m |
| STOPPED时距目标 | 5.460m |
| 确认低速保持 | 1.041 ROS秒 |
| 碰撞 / Land / disarmed | 无碰撞 / 通过 / 通过 |

**到达与最终保持不能混淆**：YOPO按原始距离<5m触发停止，随后停止过程中位置有移动，
最终STOPPED位置误差5.460m。本次满足原始到达触发与停稳条件，
不证明最终悬停点在5m以内，更不证明SUPER式终点位置保持。
如果上层要求最终停稳误差≤5m，当前实现不能据本轮标为满足；本轮没有修改原算法来掩盖该差异。

该轮仍记录已解除武装后的 offboard释放拒绝，未隐去该后处理问题。
这是一次长程通过，不是三次重复长程验收，也不是物理无人机实飞。
证据目录 `dev/yopo-single/evidence/navigation-forest-long-01/` 含源码摘要、镜像、模型身份、
场景/传感器/计算/PX4日志、原始轨迹、状态、碰撞评分和结果；
`forest-long/summary.json` 为汇总，`forest-long/result.json` 为任务原始结果。
本轮专属场景、容器和CUDA worker已回收，其他原有容器保持运行。

## YOPO Demo 补齐 AS2 Module 调用

后续按用户要求，将工作站 Demo 的正常导航目标、取消和结果读取统一到
`DroneInterfaceBase('drone0', use_sim_time=True)` 加载的共享 `navigation` Module：
`to_point(..., wait=False)` → `cancel_async()` / `done` / `result`。
DroneInterface 自带执行器处理 Action 回调，Demo 继续独立处理观测和碰撞评分，
结束时关闭并销毁 DroneInterface。算法与公共服务端保持不变。
恢复专用入口仍可通过标准 Action cancel 服务取消此前客户端遗留的任务，
不将当前 Module 无句柄的 cancel 误当成跨客户端恢复。

`prepare-workstation` 补编译 AS2 Python API 和共享 Module，并从固定 onboard 消息定义
补生成 AlertEvent/BehaviorStatus；复用已锁定SHA的CPython3.10依赖清单，运行时设置
Python依赖路径和AS2_MODULES_PATH。不重建镜像。
`log/yopo-single/module-ground-build.log` 记录4包工作站构建通过。
`dev/yopo-single/evidence/module-demo-01` 的地面与实际CUDA森林短程任务通过，
日志确认从工作站安装目录加载 navigation Module；导航结果成功且确认停稳，
无碰撞，Land和disarmed通过。该轮仍如实记录已解除武装后的offboard释放拒绝。
此前长程记录使用直接Action调用；本次应用入口更改不重标为Module长程验收。

`dev/yopo-single/evidence/module-demo-cancel` 的独立地面检查和实际CUDA森林取消用例通过：
至少3个revision后由Demo调用Module的cancel_async，结果CANCELED且stop_confirmed=true，
无碰撞，Land和disarmed通过。两轮均生成原始result和summary，环境与worker已回收。
