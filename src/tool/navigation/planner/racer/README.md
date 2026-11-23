# RACER ROS 2 接入

本目录保存锁定 RACER-ROS2 Fork 的 gitlink 与 AeroHub 接入层。PC 双机已完成 ForestScene 固定区域原生完整探索，运行与验收见 [双机交付](../../../../../deployment/airsim/demos/racer_multi_flight/PC_DUAL.md)。以下单机入口提供的能力为单机原生探索 FSM → 在线 Adapter → FollowTimedPath → AS2/PX4；属于部分接入，不声明完整四机复现。

- upstream.lock.json：记录上游基线、AeroHub Fork、分支、gitlink 路径和锁定提交。原生 ROS 2 包声明和第三方代码头保留；上游基线没有根 LICENSE，不能把所有嵌入式求解器笼统声明为一个许可证。
- ros2_adapter/native/：Air-Suck/RACER-ROS2 Git 子模块，父仓 gitlink 锁定已经发布且包含 external execution、停止反馈和诊断修改的提交。构建直接使用该子模块。
- ros2_adapter/bridge/：相机时间配对与坐标转换、原生进程管理、B-spline 时间化轨迹转换、在线交接与停止确认。算法仍在原生包，桥不直接发布运动参考。
- bridge/bspline_evaluator.py：保留此前验收所用 AeroHub FALCON spline 计算实现（历史快照 664ff5e），实际文件以本目录内容及构建/运行清单 SHA256 锁定；不是新增外部运行依赖。

原生改动提供 future-start 计划、ACCEPTED/ACTIVE/STOPPED 等双向执行反馈和 operator-stop 退出。滚动更新保留旧曲线未来时刻的 PVA/yaw，没有将曲线简化为终点。原生视点选择、TSP/ACVRP 与局部 B-spline 优化仍由 RACER 完成。PC 构建从完整 Fork 编译 17 个原生包。双机监督器沿用 pillar 启动模块中的参数函数。原版四机柱场景、点云地图和 RViz 配置保留在锁定子模块中。

当前单机示例明确映射 RACER ID 1 到 Drone0，Drone1 只作为场景中的停放车辆。话题、机数、范围参数属于这个示例，不是通用迁移要求。原生无 frontier 时的 idle/返回策略与任务有界停止分别记录，不把 FINISH 当覆盖率证明。

构建、运行、停止和回退见 [部署入口](../../../../../deployment/airsim/demos/racer_single/README.md)。输入接口与选择限制见 [能力卡片](../../../../../cards/planner/racer-single.yaml)。固定共享镜像提供 AS2/平台与执行扩展，另行锁定；当前构建不重建或替换共享镜像。
