# SPF `to_semantic_goal` 契约验证（第一阶段历史记录）

> 本文是早期接口迁移的历史证据，包含已移除的 vlm_proxy 路径及当时未完成项。
> 当前调用方式见 README，最新真实链路状态见 [live_cloud_validation.md](live_cloud_validation.md)。

后续 Cloud Function 迁移已替换私有代理与测试入口；当前下层与最终回归见
[cloud_inference_validation.md](cloud_inference_validation.md)。以下保留第一阶段原范围/结果。

## 范围

这是 Navigation 调用层的功能开发，不是一次 SPF 算法复现。
SPF 作为 Navigation alternative 提供 `to_semantic_goal`，不提供 `to_pose`，
不新增 `guide_by_description`。本轮不迁移云端 Inference 插件，不修改算法的
反投影/转向逻辑，也不新增语义到达判定。

输入：`NavigateToSemanticGoal.Goal(description, timeout)`，timeout 为必填有限正数，
单位是单调墙钟秒数，覆盖推理等待、重试退避和运动执行。
正常预算耗尽：停止后返回 `TIMEOUT`、`success=false`，ROS Action 为 ABORTED；
取消：确认停止后返回 `CANCELED`；停止无法确认：`STOP_UNCONFIRMED`、FAULT，
不接受新任务。单段轨迹成功不是语义目标成功。
停止宽限为额外 10 秒；必须收到 stop 成功响应、待提交/执行的轨迹 Action 已结束，
且接收年龄小于 0.5 秒的三维低速样本（<0.2 m/s）持续至少 1 秒。

## 构建与 ROS 合成输入测试

复用本机已有 `onboard_computer:v0.1.1`，不改共享镜像；网络关闭，源码只读，
产物为 `log/spf/semantic-navigation/ros_ws` 的独立 overlay。
构建包：`uav_planner_interfaces`、`uav_spf_planner`、`uav_spf_vlm_proxy`、
`uav_planner_python_api`、`uav_follow_path_execution`。

测试入口：
`src/tool/navigation/planner/spf/ros2_adapter/tests/test_semantic_navigation_ros.py`。
使用真实 rclpy Action/Service/Topic、生成的共享消息、SPF 服务端和 VLM 查询客户端；
传感器、VLM 响应及 FollowTimedPath 执行端为测试替身。不使用云端、AirSim、PX4 或 GPU。

已执行的契约用例：

1. 空描述、非正/NaN/Inf timeout 被拒绝。
2. 推理等待中超时，无运动提交，返回 TIMEOUT；确认停止后可接收下一任务。
3. 推理等待中取消、拒绝并发任务，迟到的 VLM 结果不能提交运动。
4. 活跃轨迹中超时，调用停止并等待执行端收口。
5. 重试退避中超时，不等完整退避结束。
6. 陈旧速度样本不能确认停止，FAULT 后拒绝新任务。
7. 垂直运动不能被水平零速误判为停止。
8. stop 响应失败不能确认停止。
9. 单段/多段运动成功不会提前完成语义任务。
10. 轨迹 goal 接受晚于 timeout 时，仍须停止它才能交接。
11. 旧 `spf/goal` 不能替换 Action 任务，也不会在 busy 分支死锁。
12. 畸形模型输出返回执行失败，不会杀死工作线程，后续任务仍有 timeout 保护。

在已构建的容器环境复跑（先加载 ROS、AS2 和本次 overlay）：

```bash
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /evidence/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=187 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
python3 /repo/src/tool/navigation/planner/spf/ros2_adapter/tests/test_semantic_navigation_ros.py
python3 -m pytest -q -p no:cacheprovider \
  /repo/src/tool/navigation/planner/spf/ros2_adapter/tests/test_projector.py \
  /repo/src/tool/navigation/planner/spf/ros2_adapter/tests/test_trajectory.py \
  /repo/src/tool/navigation/planner/spf/vlm_proxy/tests
```

`/repo` 是只读仓库挂载，`/evidence` 是上述日志目录挂载；需选择未占用 ROS domain。
正常的开发/CI 环境也可直接在 source overlay 后运行对应仓库相对测试路径。

## 结果与未完成项

- 新增 ROS 契约测试：12 项通过。原始输出：
  `log/spf/semantic-navigation/semantic-ros-tests.txt`。
- 原有 23 项单测：21 通过、2 失败。失败均为 prompt 上游保真比较，当前工作区未初始化
  SPF upstream，缺少 `action_projector.py` 和 `vlm_client.py`；未跳过、删除或改写测试。
  输出：`log/spf/semantic-navigation/existing-unit-tests.txt`。
- 5 个包在固定镜像中补编译通过；FollowPath C++ timeline 测试 1/1 通过。
  构建输出 `log/spf/semantic-navigation/build.txt`，C++ 输出
  `log/spf/semantic-navigation/follow-path-tests.txt`；`installed-path.txt` 确认测试加载安装产物。
- 第一轮 ROS 测试暴露 stop future 在消费成功响应前被替换的问题；修正响应消费顺序后
  重跑。测试 teardown 也调整为先排空 executor，再关闭 ROS context。
- AS2 Python 调用函数已实现并编译；完整 DroneInterface 加载/调用未验证。镜像有
  AS2 Python 源码，但未安装其 Python 包及 `pydantic` 依赖。本轮没有安装或 mock
  这些依赖来宣称原生模块加载通过。
- 新 Demo 的 Action 任务调用、故障注入、timeout 门控及工作站 overlay 构建逻辑已更新，
  但未运行工作站容器或有屏 AirSim + PX4 SITL 闭环。现有飞行记录只验证旧 topic/轮数契约。
- 已知转向符号、VLM 图像时间戳过期判断问题仍待独立修复；这轮测试不证明规划方向或
  观测时间正确。不声明语义导航目标到达、复现成功、RoboPi 或真机通过。

旧 `validation.json` 原样保留历史运行证据，不能将其旧“轮数达标”作为当前 Action
契约通过的依据。Demo 的 `navigation_contract` PASS 仅表示预期终态与停止契约符合，
不是导航任务成功。
