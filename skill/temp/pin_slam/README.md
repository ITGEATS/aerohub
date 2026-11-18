# PIN-SLAM 原生 ROS 2 Humble 复现记录

## 范围

- 源码：`src/basic_function/state_estimator/pin_slam/upstream@e2b7fbf328e5fac1d638e217d70a69cbe9f30a52`
- 镜像：`aerohub/pin-slam:ros2-humble@sha256:a276960113485a124ed5fc95ba47d7f457a0ae0d73cd1d9c74c1179bf130145b`
- 环境：`onboard_computer:v0.1.1` + `flight_board:v0.1.0` + 有屏 AirSim 1.8.1，`DISPLAY=:1`
- 使用 Skill：`uav-platform-sensor-adaptation`、`uav-ros-adaptation`、`uav-reproducible-build`、`as2-extension-integration`、`uav-sim-e2e-validation`

上游 Python 核心由原生 rclpy Adapter 驱动，完整锁定依赖、算法、Adapter 和 AS2 插件编入 GPU 镜像。运行时无源码、workspace、overlay、ROS 1 或 Proxy 挂载。

## 关键处理

- 保留 LiDAR 采样时间，明确 AirSim 瞬时扫描不支持 deskew；非仿真平台必须重新测量 `sensor_from_base`。
- PIN-SLAM 作为全局校正源，PX4 local odometry 保留高频控制状态；按测量时间配对，不用当前时间给旧估计盖章。
- 状态不连续、非单调时间、陈旧或未配对输出不会更新校正；重启产生新的隐式地图 epoch。
- neural points 只作为有界 RViz 视图，不代表完整隐式模型，也不能用于碰撞查询。

## 最终矩阵

- `log/pin_slam/airsim-shadow-final-20261118/gate-report.json`：通过，position RMSE 0.149 m，最大输出间隔 0.306 s。
- `log/pin_slam/airsim-authority-final-20261118/gate-report.json`：通过，position RMSE 0.210 m，canonical arrival gap 0.040 s。
- `log/pin_slam/airsim-sensor-fault-final-20261118/result.json`：LiDAR 失效约 1.84 s 检出，Land/disarm 通过。
- `log/pin_slam/airsim-algorithm-fault-final-20261118/result.json`：算法退出约 2.06 s 检出，Land/disarm 通过。

配置诊断和失败调参轮保留在 `log/pin_slam/`，不作为成功证据。物理板、真机和 PX4 local estimator 失效尚未验证。
