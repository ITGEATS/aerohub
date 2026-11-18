# FAST-LIVO2 原生 ROS 2 Humble 复现记录

## 范围

- 源码：`src/basic_function/state_estimator/fast_livo2/ros2_upstream@a713004f0ba0624c8fb80d85c7047fe62523c6fb`
- 镜像：`aerohub/fast-livo2:ros2-humble@sha256:a97bfa6cc0464a6b3bc3596e92bf03f7ba0c94e72fa7df3097bae8a614613a62`
- 环境：`onboard_computer:v0.1.1` + `flight_board:v0.1.0` + 有屏 AirSim 1.8.1，`DISPLAY=:1`
- 使用 Skill：`uav-platform-sensor-adaptation`、`uav-ros-adaptation`、`uav-reproducible-build`、`as2-extension-integration`、`uav-sim-e2e-validation`

采用锁定的原生 ROS 2 实现；FAST-LIVO2、依赖、Adapter 和 AS2 插件全部编入自包含镜像。删除活动 ROS 1 Dockerfile、Proxy、Compose 和源码挂载入口。该锁定实现为 CPU/OpenMP 路径，没有 CUDA 算法路径。

## 关键处理

- AirSim IMU 从 FRD 转为 FLU；RGB 使用 optical frame；LiDAR 明确为瞬时 XYZ 扫描，不伪造 ring、强度或逐点时间。
- 候选状态经坐标变换进入 canonical `odom`；注册点保留算法世界表示并通过实际 TF 关联。
- PX4 使用 `flight_board:v0.1.0` 内置固件；状态消息与 `onboard_computer:v0.1.1` 链路重新验证，不挂载仓库 PX4。
- shadow 精度门通过后才选择 FAST-LIVO2 为唯一 AS2 状态/TF 权威；传感器和算法故障均触发悬停、降落和解除武装。

## 最终矩阵

- `log/airsim/fast-livo2-shadow-20261118/gate-report.json`：通过，空中三维误差 0.151 m，水平误差 0.119 m，注册点非空。
- `log/airsim/fast-livo2-authority-20261118/gate-report.json`：通过，到点误差 0.097 m，最终真值误差 0.044 m。
- `log/airsim/fast-livo2-sensor-fault-20261118/result.json`：通过，约 8.65 s 检出，Land/disarm 完成。
- `log/airsim/fast-livo2-algorithm-fault-20261118/result.json`：通过，约 9.13 s 检出，Land/disarm 完成。

AirSim 不提供 LiDAR 逐点时间，因此不声明旋转扫描 deskew 等价；物理板、真机和真实数据集尚未验证。
