# Navi 目标更新接口验证（2026-11-15）

## 范围与结论边界

本轮为现有 Navigation Tool 接口开发，不是 YOPOv2 算法复现。
目标是同一个 `NavigateToPose` 会话内更新目的地，并支持到达保持、恢复和目标流失效停止。
SUPER/YOPO 原算法、既有时间化执行语义及停止门不以测试替身结果替代验收。

**源码已接入；最终接口回归结果见下表。真实动态目标飞行验收未完成。**

## 环境与证据

- 固定本地镜像：`onboard_computer:v0.1.1`，没有重建镜像。
- 独立无网络临时容器；ROS Humble + `/opt/uav_execution_ws/install` 底层。
- 新 overlay：`log/navigation-target-update/install`，只供本轮测试，不替换正在运行的部署。
- 从仓库编译 9 个包：`uav_planner_interfaces`、`as2_msgs`、`as2_python_api`、
  `uav_inference_interfaces`、`uav_planner_python_api`、`uav_yopo_planner`、
  `uav_super_adapter`、`uav_planner_framework`、`mars_quadrotor_msgs`。
- Python 局部依赖复用 `log/super-navigation/workspace/python_deps`，由
  `deployment/airsim/demos/super_forest/navigation_dependencies.json` 声明。
- 生成接口、Python Module 及两套 Adapter 均使用本轮 overlay；未用旧生成 Action 类型跑新客户端。
- 原始证据：`log/navigation-target-update/{build,rebuild,super-interface,yopo-interface,unit-regression,cards}.log`。

首次使用 dependencies 精简镜像构建时缺少 `geographic_msgs`；转用已有完整固定镜像后构建通过，
没有安装系统依赖或修改共享镜像。一次 YOPO 复跑的测试目标在输入过期边缘提交，返回 `NOT_READY`，
记录保留在 `yopo-interface-attempt2-not-ready.log`。测试提交条件增加深度新鲜度余量，
运行时 0.3 秒新鲜度门未改变，不重试掩盖运行时拒绝。

## 接口检查

SUPER 测试入口：
`src/tool/navigation/planner/super/ros2_adapter/bridge/tests/navigation_interface.py`。
YOPO 测试入口：`deployment/airsim/demos/yopo_single/test_navigation_interface.py`。
它们使用真实 ROS Action/Service/DDS、真实 NavigationModule 和 Adapter，
但规划输入、推理、执行或位姿由测试替身提供，不产生飞行能力结论。

| 项目 | 结果 |
| --- | --- |
| 9 包构建 | 通过 |
| SUPER 协议回归（13 项） | 通过 |
| YOPO 协议回归（10 组） | 通过 |
| YOPO 原轨迹数学回归（6 项） | 通过 |
| SUPER 转换回归（8 项） | 通过 |
| 公共导航框架回归（29 项） | 通过 |
| SUPER/YOPO 两张 Card 及部署接口绑定 | 静态检查通过 |
| 全仓 Card 检查 | 未通过，57 个错误来自其他旧卡；未修改它们 |
| Python 语法、diff 空白检查 | 通过 |

覆盖：同一会话运行中更新且不关闭执行、目标 UUID/版本校验、非法目标不改变原目标、
客户端拒绝更新后保留已接受版本、HOLD 到达后不结束 Action、静止目标续期、目标改变恢复、
保持中取消、迟到更新拒绝、目标流超时、普通到点回归、约束拒绝、任务超时、执行失败与停止未确认。
YOPO 还检查旧目标推理结果直接丢弃；这不是 GPU 推理性能或快速目标流连续执行验证。

## 未完成的动力学验证矩阵

以下均**未执行**，不能标记动态目标导航已验收：

- SUPER / YOPO 各自真实算法的运行中移动目标更新，核对任务、目标和执行生效证据。
- 到达保持后恢复；目标不动时续期；保持中的取消与状态失效。
- 快速更新、乱序、取消与更新/推理竞争，确认不会失控或复活旧任务。
- 更新拒绝时保留有效轨迹；PVA/yaw 连续性和覆盖耗尽安全停止。
- 真实 GPU YOPO 推理、AirSim 真实传感器输入与 PX4 SITL 动力学闭环。
- 新 overlay 下普通一次性导航、任务超时和停止失败的动力学回归。

目标 revision 目前仅确认目的地接受，没有目标版本到飞控实际生效轨迹的独立确认映射。
这与轨迹 revision 是两种证据，不能混同。
