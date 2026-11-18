# ROG-Map 原生 ROS 2 Humble 复现记录

## 范围

- 源码：SUPER Fork 中锁定的 ROS 2 ROG-Map，提交 `148116eee43112b2bee242b2480d147a2c1f4e3d`
- 镜像：`aerohub/rog-map:ros2-humble@sha256:8801b77bad61fc194ad1bab9700db3b15da711da982748da339b5e13c92d89aa`
- 环境：`onboard_computer:v0.1.1` + `flight_board:v0.1.0` + 有屏 AirSim 1.8.1，`DISPLAY=:1`
- 使用 Skill：`uav-platform-sensor-adaptation`、`uav-ros-adaptation`、`uav-reproducible-build`、`uav-component-deployment`、`uav-sim-e2e-validation`

原生 ROS 2 ROG-Map、输入转换、查询 Adapter 和验证程序编入自包含 CPU 镜像；运行时无源码、workspace、overlay 或 ROS 1 挂载。

## 关键处理

- 输入为测量时刻注册的 FLOAT32 XYZ 点云及同一时刻的 LiDAR TF，不能只改 `frame_id` 代替数值变换。
- 对外分别提供占据、膨胀占据、ESDF 切片、状态和 voxel/line 查询；点快照不是持久地图交换格式。
- 输入陈旧时 readiness 变为 false 且查询拒绝；算法重启先暴露空 epoch，再在新输入后恢复，不能复用旧地图假装连续。
- 真实下游 consumer 必须完成查询，不能以节点存活或 RViz 可见代替地图能力验收。

## 最终矩阵

- `log/airsim/rog-map-normal-20261118/result.json`：通过，269 个真实点云被接受，地图随 2.39 m 运动持续更新，占据、膨胀和 ESDF 均非空，下游查询通过，最终 Land/disarm。
- `log/airsim/rog-map-sensor-fault-20261118/result.json`：通过，约 1.02 s 检出陈旧输入，查询拒绝并在输入恢复后重新就绪。
- `log/airsim/rog-map-algorithm-fault-20261118/result.json`：通过，约 0.62 s 检出服务不可用，重启后观察到空 epoch 并恢复新地图。

AirSim 点云没有 intensity、ring 或逐点时间；物理板与真机尚未验证。
