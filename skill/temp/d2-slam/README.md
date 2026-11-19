# D2SLAM 原生 ROS 2 Humble 迁移与复现记录

## 最终范围与版本

- 父仓库分支：`feat/d2-slam-ros2-adaptation`
- 父仓库 worktree：`/home/snh/code/aerohub-worktrees/feat-d2-slam-ros2-adaptation`
- D2SLAM Fork：`src/basic_function/state_estimator/d2_slam/upstream`，分支 `feat/ros2-humble`
- `swarm_msgs` Fork：`src/basic_function/state_estimator/d2_slam/dependencies/swarm_msgs`，分支 `feat/ros2-humble`
- 迁移基线：D2SLAM `f5ea552bf1198de1332c78a3915948cd350546ad`；`swarm_msgs` `7dfbe609e8150f8bda5ac1a5a57b3a929f0a499b`
- 已发布迁移提交：D2SLAM `d9264eb4bb1a6973759d82c8651381cacc3bbaaa`；`swarm_msgs` `4b1b5aa2d6281a6b16d6d692a81f305d5f700b28`
- 运行基础：`onboard_computer:v0.1.1`；飞控为 `flight_board:v0.1.0` 内置 PX4 SITL
- 自包含镜像：`aerohub/d2-slam:ros2-humble-native`，镜像 ID `sha256:baf21bd2cc05ee01a28dabc63ccede00512a9b88f6b62a669bfaf94769f530fd`
- 多机拓扑：`deployment/swarm/configs/d2_slam.yaml` 的三机 `pc_simulation`
- 有屏验收：`DISPLAY=:1`、`AIRSIM_RENDER_MODE=windowed`

本轮完成的是算法源码、消息包、构建系统、launch、参数、时间和通信接口的原生 Humble 迁移。旧的 ROS 1 Proxy、Unix socket 桥、catkin workspace、CUDA 锁文件和运行时源码/overlay 挂载已从活动部署删除；它们不能再作为 PC 复现或板端支持方案。

## 使用与迭代的 Skill

- `ros-to-humble`
- `uav-architecture-analysis`
- `uav-platform-sensor-adaptation`
- `uav-ros-adaptation`
- `as2-extension-integration`
- `uav-reproducible-build`
- `uav-component-deployment`
- `uav-capability-cards`
- `uav-sim-e2e-validation`

本轮依据实际迁移补充了 ROS 1 必须源码迁移、交付镜像不得携带或挂载 ROS 1 runtime、多速率状态源选择、CPU 线程配额和验证器不得制造断流等规则。没有创建新的 Skill。

## 最终运行链

1. 每架无人机的 ROS 2 Sensor Adapter 发布独立 RGB、`DepthPerspective` 和 IMU。
2. 原生 ROS 2 D2VINS、D2PGO 和 D2Comm 在各自 compute 容器内运行，保留原生多机 peer 通信。
3. AeroHub Adapter 把 `32FC1` 米制深度转换为 D2SLAM 的 `16UC1` 毫米契约，并将原生输出转换为 AeroHub ROS 2 接口。
4. Adapter 订阅 D2VINS 的高频 `imu_propagation`，发布 `/droneN/d2_slam/odometry`；稀疏地标和协同状态分别发布。
5. authority 模式下，AS2 插件把 D2SLAM 用作全局校正，PX4 local odometry 继续提供高频局部位姿和速度。状态能力与地标能力共享同一进程组和启停边界。

## 构建与运行边界

- `deployment/d2_slam/Dockerfile` 在构建阶段复制锁定 Fork 源码，编译消息、算法、Adapter、AS2 插件和验证程序。
- 运行容器不挂载仓库、源码 workspace、build/install overlay、catkin devel space 或宿主依赖目录；只挂载生成配置和证据目录。
- 镜像内不存在 `/opt/ros/noetic`、catkin devel space、ROS 1 Proxy 或外部 overlay。
- 最终 AirSim RGB-D 配置使用官方可用的 `onnxruntime-cpu` 路径，不要求 CUDA。AirSim 本身仍使用主机 GPU 渲染，不能将渲染进程误记为算法 GPU 进程。
- robot1 有 14 个物理 CPU 核。三架无人机每个 D2VINS 使用 3 个 ONNX intra-op 线程；每实例 4 个线程会造成多实例争用，因此线程数通过 `D2_ONNX_INTRA_OP_THREADS` 显式配置。

## 关键问题与修复

### 低频优化输出被误当作控制状态

D2VINS 的 `odometry` 随优化关键帧发布，约 0.5 s 的间隔属于其语义，不能作为控制状态 freshness 的依据。查阅原生下游使用后，Adapter 改为订阅官方高频 IMU 传播输出 `imu_propagation`。最终 shadow 和 authority 均约 49 Hz，最大 source-stamp gap 不超过 0.045 s。迁移时不能只按消息类型选择名字相似的话题，必须核对原项目实际消费的状态流。

### 前端队列锁与丢帧

前端曾在执行 ONNX 推理时持有队列锁，生产回调又使用 `try_lock`，导致图像在高负载时被持续丢弃。修复为取出任务后释放锁再推理，回调正常入队；处理线程和回环线程只在空闲时休眠。最终有界队列容量为 3，图像轮询为 20 Hz。队列加大不是无限缓存，仍需同时观察输入时间戳、消费计数和处理延迟。

### AirSim 多相机同时激活崩溃

三机相机 RPC 同时激活会触发 AirSim 相机服务崩溃。编排器按车辆错峰激活传感器，待前一实例稳定后再启动下一实例。此修复属于平台启动时序，不改变算法数据或验收门槛。

### CPU 线程超额分配

三个 ONNX 实例各用 4 个 intra-op 线程时，虽然单实例可运行，但并发后出现输出间隔抖动。按物理核数给进程、ROS executor、AirSim 和系统留出余量后改为每实例 3 线程，三机输出稳定。线程数应根据并发实例和整机预算测量，不能把单实例最优值复制到全部实例。

### 验收窗口与观察器干扰

轨迹精度和 freshness 只统计起飞后到降落前的 airborne 窗口，初始化与落地阶段仍保留为 readiness 和安全证据，但不混入飞行指标。高带宽 ROS 回调与 AirSim truth RPC、截图和统计分线程，避免单线程证据节点自己制造 0.2--0.3 s 假断流。记录 source stamp 与本机 monotonic arrival，分别判断源流连续性和观察器实时性。

### 传感器与表示语义

AirSim 使用 `DepthPerspective`，不是 `DepthPlanar`。前视相机与 D2SLAM 外参必须一致，避免机臂和桨叶占据近场。`/droneN/d2_slam/landmarks` 是协同稀疏视觉地标，只能用于 RViz 和地标消费者，不能声明为占据图、ESDF、稠密重建或碰撞安全空间。

## 最终验证证据

证据位于当前 worktree 的被忽略目录 `log/d2-slam/`。所有下列 `result.json` 的顶层 `passed` 均为 `true`：

- `ground-imu-propagation-final-20261118/result.json`：原生 ROS 2 图、三机真实 RGB/深度/IMU、约 47 Hz 状态、非空地标、双 peer 和清理通过。
- `airsim-shadow-imu-propagation-final-20261118/result.json`：三机 49.68--49.73 Hz，最大 gap 0.042--0.045 s，position RMSE 为 0.085 / 0.135 / 0.093 m；有屏窗口、降落和清理通过。
- `airsim-authority-imu-propagation-final-20261118/result.json`：三机 48.86--48.94 Hz，最大 gap 0.045 s，position RMSE 为 0.091 / 0.144 / 0.088 m；AS2 报告 `plugin_name=d2_slam`，同镜像 shadow gate、canonical 输出、降落和清理通过。
- `airsim-sensor-fault-imu-propagation-final-20261118/result.json`：传感器与候选失效在 1.92 s 检出，随后安全降落并清理。
- `airsim-algorithm-fault-imu-propagation-final-20261118/result.json`：D2VINS 被终止后在 1.99 s 检出候选失效，随后安全降落并清理。
- `airsim-communication-fault-imu-propagation-final-20261118/result.json`：本机状态继续输出时，双 peer 丢失在 6.47 s 检出，随后安全降落并清理。

低频关键帧输出、队列争用及线程超额配置造成的失败轮次保留在同一证据根目录，不得删除或用最终成功结果覆盖。

## 交付边界

- 已验收的是三台 PC compute 容器，复用了 `deployment/swarm/`；尚未验证 `hybrid_board` 或物理飞行。
- D2SLAM 全局校正失效时冻结最后有效校正，并依靠新鲜 PX4 local state 完成受控降落；PX4 local estimator 自身失效不在本轮结论内。
- 稀疏地标不提供规划碰撞语义。
- 两个 Fork 已先发布，父仓库 gitlink 和组件清单锁定上述可远端获取的提交。
