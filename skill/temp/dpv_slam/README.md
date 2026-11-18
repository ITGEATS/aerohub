# DPV-SLAM 原生 ROS 2 Humble 复现记录

## 范围

- 源码：`src/basic_function/state_estimator/dpv_slam/upstream@0ac95b656d1fda91c271d2a106460d19ad966fc7`
- 镜像：`aerohub/dpv-slam:ros2-humble@sha256:5341db3f2f08f7d5376bddcf7288d7b843900dcdbf0cfbc9e1d4cae1b618c783`
- 环境：`onboard_computer:v0.1.1` + `flight_board:v0.1.0` + 有屏 AirSim 1.8.1，`DISPLAY=:1`
- 使用 Skill：`uav-ros-adaptation`、`uav-reproducible-build`、`as2-extension-integration`、`uav-sim-e2e-validation`、`uav-capability-cards`

DPVO 算法、CUDA 扩展、原生 Humble Adapter 和 AS2 插件均编入自包含镜像。运行时不挂载源码、workspace 或 overlay，不运行 ROS 1、Proxy 或 bridge。

## 关键处理

- 保留被移除帧的原始观测时间，并按 Adapter 报告的 source stamp 配对真值。
- 固定上游评测随机种子 `1234`，保留官方 `BACKEND_THRESH=18` 和回环能力。
- 区分低频视觉全局校正与高频 canonical 状态：视觉输出允许不均匀关键帧间隔，控制状态仍按 0.1 s freshness 验证。
- 单目尺度只在满足运动基线和拟合门后冻结；稀疏点是视觉地标，不是占据地图。
- 故障用例先完成可观测运动，再停止传感器或算法容器，验证 canonical 状态保持、Land、disarm 与资源清理。

## 最终矩阵

- `log/dpv_slam/airsim-shadow-24patch-20261118/gate-report.json`：通过，position RMSE 0.162 m，候选 10.35 Hz，完整覆盖。
- `log/dpv_slam/airsim-authority-24patch-20261118/gate-report.json`：通过，position RMSE 0.246 m，canonical arrival gap 0.032 s。
- `log/dpv_slam/airsim-sensor-fault-24patch-20261118/result.json`：RGB 失效约 1.90 s 检出，Land/disarm 通过。
- `log/dpv_slam/airsim-algorithm-fault-24patch-20261118/result.json`：算法退出约 1.89 s 检出，Land/disarm 通过。
- 对应 `cleanup.json` 均确认容器清理和端口释放。

历史门槛失败与补丁诊断镜像保留在 `log/dpv_slam/`，不作为成功证据。物理板、真机和 PX4 local estimator 失效尚未验证。
