# D2SLAM 复现过程记录

## 最终范围与版本
- 开发分支：`feat/perception-d2-slam`
- 上游源码：`src/basic_function/state_estimator/d2_slam/upstream`
- 上游基线：`4c33871083e4ae97b97973eb5bd88dc6735359d7`
- AeroHub Fork：`Air-Suck/D2SLAM` 的 `aerohub` 分支，提交 `f5ea552bf1198de1332c78a3915948cd350546ad`
- 依赖：`swarm_msgs@7dfbe609`、`ceres_solver@488a28fa`、`opengv@5dda2a3d`
- 拓扑：`deployment/swarm/configs/d2_slam.yaml` 的三机 `pc_simulation`
- 仿真链路：有屏 AirSim 1.8.1 + `flight_board:v0.1.0` 内置 PX4 SITL + `onboard_computer:v0.1.0`
- 验收显示：`DISPLAY=:1`，`AIRSIM_RENDER_MODE=windowed`
- 算法运行：每机一个真实 D2VINS GPU 进程，不使用专用镜像

## 使用与回顾的 Skill

- `uav-architecture-analysis`
- `uav-platform-sensor-adaptation`
- `uav-ros-adaptation`
- `as2-extension-integration`
- `uav-reproducible-build`
- `uav-component-deployment`
- `uav-capability-cards`
- `uav-sim-e2e-validation`

其中平台传感器、ROS 适配、AS2 接入、可复现构建和端到端验证 Skill 已依据本轮新增经验更新。架构、部署和能力卡 Skill 的现有职责与模板已覆盖本轮做法，经回顾无需增加 D2SLAM 专属规则。

## 最终接入

1. D2SLAM 保留原生 ROS 1 实现；ROS 2 Humble Adapter 与 ROS 1 Proxy 通过 Unix-domain socket 双向传输必要数据，不使用通用 `ros1_bridge`。
2. 每架无人机消费独立 RGB、AirSim `DepthPerspective` 和 IMU。ROS 2 深度为 `32FC1` 米，Proxy 转换为原生路径所需 `16UC1` 毫米且保持深度位宽。
3. 相机从机体原点前移 0.4 m，并同步修改 D2SLAM `body_T_cam0`。原配置会看到大量本机机臂和桨叶，使前端跟踪相对相机静止的近场结构。
4. 自身状态与稀疏地标对外分成两项能力，但共享同一 D2SLAM 进程与启停边界。
5. AS2 插件把 D2SLAM 作为 `map -> odom` 全局校正；PX4 local odometry 提供高频 `odom -> base_link` 和 body twist。
6. 上游必须修改的 Ceres ownership、深度位宽、landmark stamp 和协同证据接口进入可追溯 Fork；AeroHub Adapter、Proxy、插件、编排和配置留在主仓库。

## 构建与运行要点

- 固定 onboard 镜像中的 catkin 工具链会把绝对工作区路径写入产物。构建和运行时同时把外部工作区挂载到 `/workspace` 与原主机绝对路径，不能只挂 `/workspace`。
- ROS 1/CUDA 依赖使用 `deployment/d2_slam/locks/` 锁定；ROS 2 Adapter 和 AS2 插件在外部 overlay 补编译。
- flight-board Docker 内的 PX4 是唯一飞控来源。诊断消息使用与该固件 schema 对齐的外部 `px4_msgs` overlay，不使用仓库 PX4 源码。
- Bridge 使用有界队列并记录 `input_drop`、`output_drop` 和 `reconnect`；最终通过现场三机均为 `input_drop=0`、`output_drop=0`、`reconnect=1`。

## 关键失败与修复

### 视觉估计几乎不移动

最初真实轨迹约 4.6 m，而候选仅移动约 0.2 m。保存的前视图证明机臂和桨叶占据大面积近场，视觉前端优先跟踪相对相机静止的机体结构。相机前移 0.4 m，并同步外参后，三机 shadow 运动与精度门限通过。不能用遮罩或放宽 ATE 代替物理传感器修复。

### 深度输入和地标时间

AirSim 场景最初错误使用 `DepthPlanar`，改为 D2SLAM 配置所需的 `DepthPerspective`。上游通用图像 helper 会把 `mono16` 降为 8-bit，深度回调改为显式保留 `16UC1`。landmark PointCloud 原本使用零时间戳，改为最近估计 odometry stamp。

### Ceres double free

`ConsensusSolver` 的最后一步让 Ceres 接管由 `shared_ptr` 持有的 cost/loss/manifold，reset 时发生二次释放。所有 consensus step 统一设为 `DO_NOT_TAKE_OWNERSHIP` 后真实多机运行稳定。

### authority 被误判为断流

最初 canonical 最大间隔约 1.2 s，其中包含起飞前初始化空档；限定到实际飞行窗口后仍约 0.2--0.3 s。进一步发现单线程验证器每 0.2 s 串行执行三次 AirSim truth RPC，期间无法处理 ROS 回调，SensorDataQoS 队列覆盖造成假断流。将 ROS executor 放到独立线程，并同时记录 wall arrival 与 source header gap 后，authority 三机最大 arrival gap 为 0.040--0.048 s，低于预声明 0.1 s 门限。门限未放宽。

### 启动环境

非交互 shell 未配置 AirSim 资源路径，最终通过本机 `AIRSIM_BINARY` 显式指定；PX4 message overlay 同样由本地环境变量提供。两者属于个人机器配置，不写入共享文件。ROS setup 脚本不兼容 Bash nounset，插件 smoke 在 source 环境时不启用 `set -u`。

## 最终验证证据

证据根目录为 `$D2_SLAM_WORKSPACE`：

- `shadow-fork-f5ea552-final-04/result.json`：通过。三机 position RMSE 均低于 1.5 m，真实输入、GPU、landmark、双 peer、俯视相机、着陆和清理均通过。
- `authority-fork-f5ea552-final-04/result.json`：通过。AS2 报告 `plugin_name=d2_slam`，canonical gap 低于 0.1 s，三机精度与协同通过。
- `sensor-fault-fork-f5ea552-final-04/result.json`：通过。停止 `drone1` sensor adapter 后检测传感器和候选失效并安全降落。
- `algorithm-fault-fork-f5ea552-final-04/result.json`：通过。强制退出 `drone1` D2VINS 后检测候选失效并安全降落。
- `communication-fault-fork-f5ea552-final-04/result.json`：通过。暂停 `drone0`、`drone2` D2VINS 后，`drone1` 在本机候选继续输出时检测远端帧失效，再安全降落。
- `plugin-smoke-final-02/output.log`：通过。非恒等全局校正与高频 local pose/twist 正确组合，1 s 内收到 5852 个 local 更新（合成接口负载，仅证明插件语义）。
- `build-fork-f5ea552-clean.log`：从干净 Fork 提交完整重编译通过。

组件清单通过 `log/d2-slam/accepted/<场景>/` 指向上述通过现场的精简副本；完整现场保留在外部工作区。所有重跑均使用新的证据目录，`authority-final-02` 的精度失败和 `shadow-final-03` 的候选间隔失败也保留用于复核。最终通过现场确认无任务容器或服务线程残留。

## 交付边界

- D2SLAM landmark 是稀疏视觉地标，不是占据图、ESDF、稠密重建或安全空间。
- 已验收三台 PC 模拟 compute container；未验收 `hybrid_board` 或物理飞行。
- D2SLAM 全局校正失效时保持最后有效校正并依靠 PX4 local state 降落；PX4 local estimator 自身失效不在本轮结论内。
- 上游 Fork 提交、父仓库 gitlink、文档和 Card 必须指向远端可获取的同一版本后，才能完成最终提交与推送。
