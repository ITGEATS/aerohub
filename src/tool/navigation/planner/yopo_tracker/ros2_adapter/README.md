# YOPO-Tracker ROS 2 Humble Adapter

上游使用 ROS 2 Jazzy 和 Python 3.12。迁移目标是项目固定 onboard-computer 镜像的 ROS 2 Humble 和 Python 3.10。镜像构建从上游 Git 提交 `d313f8610195dc986caceff7a3616f63d7e6f31f` 导出，保留 MIT 许可证，校验 `epoch50.pth`，并用 Humble 的 rosidl 构建上游 `quadrotor_msgs`。NumPy 1.21.5、SciPy 1.8.0 和 OpenCV 4.5.4 来自固定基础镜像。镜像内的独立环境按 `uv.lock` 安装 Torch/CUDA 与 YAML 依赖。

## 构建

在仓库根目录执行。构建机需要 Git、tar、Docker 和所需依赖源的访问权限。

```bash
python deployment/yopo_tracker_humble/build.py
```

有本地上游仓库时，传入 `--upstream <git-repo>` 可从本地 Git 对象导出同一提交。依赖缓存可用时，可加 `--offline` 禁止构建步骤联网。镜像在构建阶段生成 `quadrotor_msgs` 和当前仓库 `uav_planner_interfaces` 类型支持，包含上游源码、模型权重、`native/` 原生调用与 `bridge/` 实时 Adapter。运行只加载镜像内产物。

本机已有兼容 CUDA worker 时，可构建共享其镜像层的本地验收镜像。以下命令使用本轮验证过的 worker。将本地上游路径传给 `--upstream` 可复用已有 Git 对象。

```bash
python deployment/yopo_tracker_humble/build.py \
  --uv-cache-image elastic-stage2-worker:ultralytics8.4.210-cu124 \
  --reuse-worker --tag uav/yopo-tracker:humble-native-local
```

此模式复用 worker 的 `/opt/vision`，只导出上游源码、权重、Humble 类型支持和启动代码等应用层。首次构建仍需要锁定依赖和接口产物，已有构建缓存时可直接复用。

## 原生节点契约

容器传入 `--native-diagnostics` 可启动原生诊断节点。该入口保留上游的 measured 起点模式、时间步长和候选生成策略。话题在 `yopo_net` 的节点私有命名空间内，可通过 ROS remapping 配置。实时执行接入见[适配设计](../DESIGN.md)。

| 节点私有接口 | 消息 | 语义 |
| --- | --- | --- |
| `~/odom` | `nav_msgs/Odometry` | 上游就近使用最新到达的自身状态，速度由原生节点按世界系线速度读取。订阅为 BEST_EFFORT |
| `~/rgb`、`~/depth` | `sensor_msgs/Image` | RELIABLE，队列深度为 1。按原始采集 stamp 进行原生近似同步。颜色为 RGB8/BGR8，深度为 32FC1 米或 16UC1 毫米 |
| `~/respawn` | `std_msgs/Bool` | true 时复位原生会话和目标滤波器 |
| `~/diagnostics/trajs_visual` | `visualization_msgs/MarkerArray` | 供离线核对的原生候选轨迹诊断输出，固定世界系，header 是发布时刻 |
| `~/diagnostics/position_command` | `quadrotor_msgs/PositionCommand` | 上游原生参考诊断，用于核对生成类型支持及无目标时的输出行为 |

## 隔离验证

验证脚本启动镜像内的原生节点，等待 DDS 发现，以 ROS 消息发送合成里程计和 RGB-D。每组候选从真实订阅端返回并校验后，脚本才发送下一对图像。脚本检查世界系和起点、候选与采样点数量、数值有限性，并检查错时图像及空场景的输出。该验证使用真实 CUDA 推理、合成传感器与候选诊断，不依赖 AirSim。

```bash
docker run --rm --network none --gpus all -e ROS_LOCALHOST_ONLY=1 \
  --entrypoint bash uav/yopo-tracker:humble-native -c '
    source /opt/ros/humble/setup.bash &&
    source /opt/yopo-tracker/install/local_setup.bash &&
    cd /opt/yopo-tracker &&
    exec "$YOPO_TRACKER_PYTHON" -u -m ros2_adapter.native.verify_ros
  '
```

跨发行版数值基线还需要同一固定输入在上游 Jazzy 环境重放。AirSim 实时输入及执行的验收状态见 [Planner README](../README.md#验收状态)。

验证共享 CUDA 层的本地镜像时，把上述镜像标签换成构建时选择的本地标签。4 对同步输入各返回 15 条候选，每条 20 个世界系样点。错时图像未产生候选，空场景未产生位置指令。

### 默认 FollowTimedPath 接口复验

复用本地镜像的 AS2 和 Python 环境。启动 `--network none` 的临时容器，将 `src/tool/navigation/interfaces/`、`src/behavior/follow_path/` 和本组件的 `replay/`、`ros2_adapter/` 复制到容器内，并保留相对于仓库根目录的路径。在容器中执行 [verify_action.sh](../replay/verify_action.sh)，脚本构建临时 overlay、启动默认 Behavior 和模拟控制器。两段轨迹使用上游 `set_trajectory` 从合成边界生成。验证覆盖 Goal、Modify、拒绝、切换、PVA/yaw 采样和取消。

同一脚本在隔离容器内可加 `--selected-fixture`，并在启动容器时提供 `--gpus all`。该模式先用真实 GPU 模型回放空场景，再用固定候选张量验证原生 EKF 和选轨到真实 Behavior 的交接。第二段接续点来自 Behavior 已接纳的首段，验收使用匹配 revision 的反馈和实际参考样本。

`--selected <sqlite3-bag>` 使用实际模型回放指定录制。录制包含 `/drone_0/rgb`、`/drone_0/depth` 和 `/drone_0/odom`。图像必须按采集时间可配对，里程计必须使用固定 `drone0/odom` ENU 和原生节点需要的世界系速度，首段位置须与模拟控制器初始位置连续。验证保留录制的观测时间，并在推理前单独确定执行时刻。该入口最多核对两段，不连接实时相机或飞控。

### Windows AirSim 固定参考试飞的 PX4 启动

使用 [start_sitl_px4.ps1](../replay/start_sitl_px4.ps1) 启动试飞的单机飞控容器。它使用本地 flight-board 镜像、ROS 域 161 和 PX4 实例 96。传入本轮已有的 Docker 网络、容器名和供 PX4 连接 TCP 中继的 settings 文件。需自定义固件 ULog 话题时传入 `-LoggerTopicsFile`。相机、时钟和 AS2 由各自入口启动。

```powershell
& ./src/tool/navigation/planner/yopo_tracker/replay/start_sitl_px4.ps1 -SettingsFile $px4Settings -Network $sitlNetwork -ContainerName $vehicleContainer
docker exec $vehicleContainer /opt/px4-gazebo/bin/px4-param --instance 96 show SENS_GPS0_DELAY
```

飞行前从 PX4 读回的值应为 `200` ms。该参数在 PX4 启动时加载，与本场景的 AirSim GPS 延迟一致。

### 实时 ROS 2 Adapter

[正式节点](bridge/live_node.py)是运行镜像的默认入口。使用包含同一版 `uav_planner_interfaces` 的 AS2 执行端。容器接入部署已有的 DDS 网络和 ROS 域，同域运行时钟、相机、定位及执行节点。启动时提供对应场景的制动配置。

```bash
docker run --rm --gpus all --network "$sitl_network" -e ROS_DOMAIN_ID="$ros_domain" \
  uav/yopo-tracker:humble-native --brake-calibration /opt/yopo-tracker/brake-calibration.sitl.json
```

接线沿用 `drone0` 的 FrontCamera RGB-D、CameraInfo、AS2 位姿和速度、TF 及 `/clock`。相机桥配置 RELIABLE，维持采集时间戳。任务使用原生 AS2 起飞并进入悬停后，调用 `/yopo_tracker_live/start`。匹配 RGB-D 与定位后，单独的工作线程调用锁定模型及其目标 EKF。Adapter 通过同一个 FollowTimedPath Action 会话接管，以已接纳轨迹的未来 PVA/yaw 生成接续点。Action feedback 区分接纳和实际生效。

每个追踪版本提交前即附上制动与静止覆盖。制动沿原曲线逐渐降低推进速度，按制动时刻的同一 PVA/yaw 交接，并从剩余曲线留出实测过冲余量。短覆盖或不可行候选会被跳过。校准 JSON 需要 `observed_terminal_overrun_m`、`reference_acceleration_m_s2` 和 `reference_jerk_m_s3`。前向 SITL 对照使用 0.675 m、3 m/s² 和 8 m/s³，适用于本次仿真配置。剩余曲线长度和采样的动力学限制用于选择制动方案，避障与实际停车效果需在目标场景验证。

[SITL 制动配置](../../../../../../deployment/yopo_tracker_humble/brake-calibration.sitl.json)已随镜像打包。

当配对观测断流、原生跟踪退出或调用 `/yopo_tracker_live/stop` 时，当前会话停止接受新的追踪更新。已接纳的制动尾段继续执行，满足原连续低速条件后以最终 revision 退出到悬停。`/yopo_tracker_live/status` 给出断流或停机原因、提交和生效 revision、低速确认和结果。新任务需再次调用 start。

```bash
ros2 topic echo /yopo_tracker_live/status
ros2 service call /yopo_tracker_live/start std_srvs/srv/Trigger '{}'
ros2 service call /yopo_tracker_live/stop std_srvs/srv/Trigger '{}'
```

先开启状态订阅，再在另一个终端发起任务。stop 的响应只确认请求已锁定。通过匹配任务 ID 的 `finished` 事件及实测低速判断是否可继续 Land。更新或回滚时，等待降落和解除解锁，恢复对应源码与同版 overlay，并重新初始化目标估计。

[移动目标与相机中断验证脚本](../replay/sitl_tracking_brake.py) 操作隔离 AirSim 场景、原生 Takeoff 和 Land，并在指定数量的追踪 revision 生效后创建 `pause-camera` 文件，由宿主只暂停独立相机容器。两版追踪并暂停相机的用例已通过。三版连续追踪用例在第三条候选生成前进入制动，后续持续跟踪与实时图像仍有输入时的目标丢失用例待验收。
