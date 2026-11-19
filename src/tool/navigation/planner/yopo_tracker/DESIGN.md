# YOPO-Tracker 规划器适配设计

保留 YOPO-Tracker 原生的 RGB-D 联合推理、目标关联和轨迹选择。通过默认 `FollowTimedPath` 执行带制动尾段的曲线。当前 [验收状态](README.md#验收状态) 包括移动目标两版滚动轨迹、相机断流停车和待验收的持续跟踪。

## 1. 两条链路对照

| 环节 | YOPO-Tracker 上游默认链路 | 本项目选定的链路 |
| --- | --- | --- |
| 输入 | RGB、深度、自身里程计；从自身运动状态构造模型输入 | 保留 RGB-D 与自身状态输入，记录图像采集时刻、姿态与相机外参 |
| 目标与轨迹 | 网络为每个候选同时预测目标位置、终端位置/速度/加速度（PVA）、代价、置信度和时长；内置目标滤波器参与筛选 | 保留候选索引与原生关联顺序，由 YOPO-Tracker 自己选轨迹 |
| 新轨迹起点 | 默认 `plan_from_reference=false`，起点位置和速度来自实际里程计，加速度来自当前参考 | 选择上游已有的 `plan_from_reference=true`。滚动段在指定交接时刻从已接纳的参考曲线取起点 PVA |
| 执行 | 原生节点按内部时钟采样五次曲线，发布含 PVA、yaw、yaw_dot 和状态标志的 `PositionCommand`，由 `NetworkControl` 消费 | Adapter 提交带绝对生效时刻的 `TimedPath`，由默认 Behavior 采样，经过 AS2 控制器与平台进入 PX4 |
| 下一轮 | 新一帧观测触发原生规划更新 | 新观测驱动下一轮规划；Behavior 对上一段的接纳结果决定后续参考从哪条曲线衔接 |

任务中只有一个跟踪规划 Tool，即 YOPO-Tracker。Mission 发起和取消任务，起飞、降落由各自的 Behavior 承担，跟踪期间由默认 FollowTimedPath 独占运动参考。

上游基线为 `d313f8610195dc986caceff7a3616f63d7e6f31f`，目标项目设计基线为 `44ce9af`。可在上游依次对照 `callback_images`、`NMS`、`set_trajectory` 和 `control_pub`。[锁定版本的上游源码](https://github.com/TJU-Aerial-Robotics/YOPO-Tracker/blob/d313f8610195dc986caceff7a3616f63d7e6f31f/YOPO/test_yopo_ros.py)

实施分为两步。先在 ROS 2 Humble 中运行上游原生节点，保留默认 measured 模式，用隔离 DDS 回送合成输入并核对真实候选。平台和执行适配阶段再启用表中的参考轨迹模式，衔接 AirSim 和默认 FollowTimedPath。[Humble 迁移基线](ros2_adapter/README.md)

## 2. Planner 与执行器的交接

1. **导出选中的曲线。** Adapter 需要取得这一段实际采用的起点、终点、原始时长及同一时间线上的 yaw。图像采集时刻用于解释目标观测，轨迹生效时刻用于解释参考 PVA。上游逐点发布的 `PositionCommand` 本身没有携带整段曲线的未来生效时刻。
2. **首次提交。** 在固定的车辆 ENU 坐标系下生成 `TimedPath`，填写任务标识、版本号、生成时刻和生效时刻。每个点保留相对时间、PVA 和 yaw。首段通过 `FollowTimedPath` Goal 提交。[TimedPath 消息](../../interfaces/msg/TimedPath.msg) · [轨迹点字段](../../interfaces/msg/TimedTrajectoryPoint.msg)
3. **滚动更新。** 从已接纳时间线在交接时刻采样，构造下一段，通过 Modify 提交。接纳后按原定时刻生效。拒绝时保留上一段及其原覆盖终点。沿用默认执行器的位置、速度、加速度和偏航连续性检查。[默认执行器时间线](../../../../behavior/follow_path/include/uav_follow_path_execution/timeline.hpp)
4. **取消与结束。** 取消时让该任务的在途规划结果失效。Adapter 等待执行器返回终止结果；Mission 单独确认飞行器状态，再交给降落 Behavior。执行端返回的 revision 用于核对终止的是哪一版轨迹。[Action 定义](../../interfaces/action/FollowTimedPath.action)

首版对照范围是可见目标的连续跟踪和取消。`TimedTrajectoryPoint` 仅提供 yaw 角；显式 yaw_dot 前馈、上游悬停状态标志和搜索行为的执行效果仍需分别验证。本设计没有新增门限，也没有选择 HFV 执行包。

## 3. 已有证据与待实现部分

| 检查项 | 当前证据 |
| --- | --- |
| 原生模型输入与推理 | CUDA 回放 8 对合成空场景 RGB-D 帧，完成 8 次推理，确认目标 0 次。覆盖无目标输入。[原生回放入口](replay/native_replay.py) |
| ROS 2 Humble 原生接入 | 通过隔离容器中的真实 DDS 验证。4 对同步输入各得到 15 条候选轨迹，错时图像无候选，空场景无位置指令。[Humble 迁移验证](ros2_adapter/README.md) |
| 五次曲线与 yaw | 固定曲线的 151 个时刻通过 PVA 数值对照；合成 yaw 样本覆盖角度跨界。[数值检查](replay/check_curve.py) |
| 默认 Behavior | 在隔离 Humble 容器内补编译默认执行器。用合成边界状态调用上游 `set_trajectory` 生成两段五次曲线，提交真实 Goal 与滚动 Modify。验证了不连续更新和重复版本拒绝、按生效时刻采样三轴 PVA 与跨 ±π 的 yaw、两版 Action 反馈及取消后轨迹输出静默。控制器为模拟节点。[Action 检查](replay/check_action.py) · [复验入口](replay/verify_action.sh) |
| 选轨到执行的隔离回放 | 已实现有限 rosbag 回放入口。固定候选张量经过原生 EKF、候选选择和五次曲线生成后，实际提交 Goal 与 Modify。下一段在已接纳曲线上取交接 PVA 和 yaw。真实 CUDA 空场景未提交轨迹。[隔离交接验证](replay/verify_selected.py) |
| 有屏 AirSim 地面输入 | 飞控未解锁时采集 79 组 RGB、DepthPlanar 与 AS2 位姿/速度，最大 RGB-D 时差 3.001 ms，最大位姿配对时差 12.001 ms。真实 CUDA 模型处理全部 79 组，其中 43 组有已确认目标状态。采集阶段没有导航执行任务。[地面采集入口](replay/collect_airsim_ground.py) |
| 真实模型正向执行回放 | 同一份 AirSim 录制由真实 CUDA 模型完成原生选轨。默认 Behavior 实际接纳 Goal 和 Modify，输出 75 和 79 个三轴 PVA/yaw 参考样本，取消后轨迹停止输出。模拟控制器在独立 executor 上发布状态，隔离回放与 AirSim 实例没有执行连接。[正向回放入口](replay/verify_selected.py) |
| EKF 步长对照 | 相机设为 15 Hz，79 对样本按仿真时间测得 14.994 Hz。相同输入按上游 `image_fps=30` 和对照值 `15` 回放，两组各生成 18 次规划，已确认帧数分别为 43 和 33。静止序列缺少目标状态误差的对照，保留上游配置。[频率对照入口](replay/compare_frame_rate.py) |
| 默认执行器 SITL 试飞 | 使用与场景目标无关的固定五次曲线。AS2 起飞后先执行 Goal，再滚动到 Modify。两版均收到生效反馈，前视两路图像时间戳持续推进，PX4 里程计在东向实际移动约 0.257 m。取消返回 `CANCELED`，随后停稳、降落和上锁。[固定参考试飞入口](replay/sitl_fixed_reference.py) |
| 实时模型选轨与执行 | 7 米静置场景的悬停输入共 108 帧，2 帧超过原生置信门限，EKF 确认 0 帧，因此无轨迹提交。5 米静置场景完成两次实时轨迹 Goal、Modify 和生效验证。第三次飞行记录 93、30 条运动参考，第二版使用第一版生效后采集的新图像。取消返回 `CANCELED`，停稳条件最长连续满足约 0.432 个仿真秒，未达既定的 1 个仿真秒，随后原生 Land 和上锁通过。[实时试飞入口](replay/sitl_live_tracker.py) |
| 固定参考取消对照 | 在约 1.076 m/s 时取消固定轨迹，现有悬停交接再次出现速度反复换向，最长仅约 0.453 仿真秒符合停稳条件。PX4 固件在取消后进入速度模式并收到 AS2 发出的回拉指令。Land 和上锁通过。[同速取消诊断入口](replay/sitl_cancel_diagnostics.py) |
| 制动轨迹对照 | 经现有 Modify 接入归零速度参考，末段完成时飞行器仍在运动。延后完成并保持归零速度参考的复验中，实测低速连续约 0.540 仿真秒，原停稳判据仍未通过。制动轨迹目前用于 SITL 验收。[制动诊断入口](replay/sitl_cancel_diagnostics.py) |
| 前向实测制动 | 用辅助深度相机核实前向走廊，实测初速约 1.084 m/s 时按实际运动选制动段。预计停止点约 1.98 m，已超过固定段 1.8 m 的覆盖，实际最远位移约 2.41 m。低速连续约 0.528 仿真秒，停稳未通过，两轮均未监测到新接触并完成正常降落。制动余量需在跟踪规划时提前保留。[前向制动诊断入口](replay/sitl_cancel_diagnostics.py) |

## 4. AirSim 接入设计

首个部署面向单机 AirSim + PX4 SITL。模拟器产生图像，项目现有传感器桥负责 AirSim RPC 和 ROS 发布；算力容器运行 YOPO-Tracker、AS2 与默认 FollowTimedPath；工作站 Mission 管理任务。飞行器的位姿、速度来自 AS2 自定位。AirSim 场景中的目标真值只供仿真评分使用。四方进程可在一台 PC 上运行。[普通 YOPO 算力侧编排](../../../../../deployment/airsim/demos/yopo_single/compute.py) · [SUPER 单机部署](../../../../../deployment/airsim/demos/super_forest/README.md)

### 4.1 从相机到规划器

单机验证使用仿真车辆 `PX4`、ROS 命名空间 `drone0` 和前视相机 `FrontCamera`。在该相机的 `CaptureSettings` 中同时声明 Scene（ImageType 0）和 DepthPlanar（ImageType 1），使用同一光心、安装姿态、图像尺寸和水平视场。相机桥在同一次批量 RPC 中请求两路图像，并按各自的采集时间发布图像与内参。[桥的请求和发布实现](../../../../../simulator/airsim/sensors/uav_as2_sensor_adapter/node.py)

下表是实时 Adapter 的 ROS 接线。话题均相对于 `/drone0/`，时钟例外。

| 来源 | 话题和类型 | Tracker 输入端的处理 |
| --- | --- | --- |
| AirSim Scene | `sensor_measurements/FrontCamera/rgb/image`，`sensor_msgs/Image` | 桥输出 `rgb8`；沿用上游预处理转为训练使用的 BGR 顺序，归一化一次 |
| AirSim DepthPlanar | `sensor_measurements/FrontCamera/depth_planar/image`，`sensor_msgs/Image` | 使用以米计的 `32FC1` 视轴深度，与彩色图逐像素配准；保留原始采集时间 |
| 相机桥 | 上述两路各自的 `camera_info`，`sensor_msgs/CameraInfo`；光学系为 `drone0/FrontCamera_optical` | 校验两路尺寸、内参、frame 与静态 TF；处理图像时遵守编码、行步长和字节序 |
| AS2 自定位 | `self_localization/pose`、`self_localization/twist`，`PoseStamped`、`TwistStamped` | 根据图像采集时刻选取对应姿态；按速度消息声明的参考系转换线速度。须核对速度的参考点。首段按上游初始化参考加速度，随后从已接纳曲线取参考加速度 |
| 仿真时钟 | `/clock`，`rosgraph_msgs/Clock` | 图像、定位、规划与 Behavior 使用同一仿真时基；暂停期间不得用墙钟推进轨迹。回拨使原会话时间线失效 |

Tracker 的 RGB 与深度均使用 320×192、水平 90°。桥用 `(W−1)/2, (H−1)/2` 作主点，上游配置用 `W/2, H/2`，且单一水平 FOV 和宽高比决定实际垂直视场。几何精度验收应逐点核对中心和边缘像素的投影。[桥的 CameraInfo 实现](../../../../../simulator/airsim/sensors/uav_as2_sensor_adapter/node.py) · [上游投影配置](https://github.com/TJU-Aerial-Robotics/YOPO-Tracker/blob/d313f8610195dc986caceff7a3616f63d7e6f31f/YOPO/config/config.py)

试飞相机桥以 15 Hz 请求图像，并显式设置 `camera_reliable=true`。按原始采集时间对齐 RGB-D 和定位。Tracker 的 EKF 以 `image_fps` 配置固定预测步长，验证时应同时记录实际参与推理的帧间隔。[Tracker 原生输入与滤波时钟](https://github.com/TJU-Aerial-Robotics/YOPO-Tracker/blob/d313f8610195dc986caceff7a3616f63d7e6f31f/YOPO/test_yopo_ros.py)

### 4.2 一帧数据如何形成一段参考

算力侧为配对成功的 RGB-D 保存两个采集时刻、配对姿态和相机外参；同一帧的模型结果始终引用这份快照。内置滤波器使用由**观测时相机位姿**解释的目标位置。轨迹候选的相对位移则在固定 ENU 中叠加到**计划交接时刻的参考起点**；终端速度与加速度只旋转。两个原点必须分别保留。目标估计与候选选择继续使用上游实现，公共 Tracking Tool、普通 YOPO 的导航目标和 SUPER 地图均不作为此 Planner 的输入。

单机实时入口位于 [ros2_adapter](ros2_adapter/README.md)。输入侧对齐图像与定位，工作线程调用锁定的原生 RGB-D 模型和目标 EKF。Adapter 在上游选中曲线处冻结边界状态、时长和 yaw，由一个客户端提交 FollowTimedPath。模型及预处理随当前迁移镜像运行。独立 Inference 服务接入仍属于后续设计。回放与实时入口复用轨迹编码、模型加载和 yaw 采样。

## 5. 默认 FollowTimedPath 接入设计

选择 `uav_follow_path_execution` 作为唯一执行包。普通 YOPO 已经示范单机固定 ENU 下直接构造 Goal；SUPER 则示范如何把规划器已经选定的多项式交给执行层。采用前者的单机接口和后者的曲线交付边界。SUPER 使用的机群客户端会把 fleet NED 转成车辆 ENU；Tracker 在 §4 已生成车辆 ENU 曲线，应直接构造单机 Goal，以免二次交换 x/y、翻转 z 和 yaw。[普通 YOPO 提交实现](../yopo/ros2_adapter/uav_yopo_planner/node.py) · [SUPER 曲线桥](../super/ros2_adapter/bridge/uav_super_adapter/node.py) · [机群坐标转换](../../framework/uav_planner_framework/follow_path_client.py)

| 交接阶段 | 客户端与 Behavior 之间的契约 |
| --- | --- |
| 准备 | 启动时核实 `/drone0/follow_timed_path` 由默认包提供。首段要满足 Behavior 的飞行状态、定位新鲜度和起点连续性检查。Mission 负责仿真起飞与稳定，Planner 不发布运动参考 |
| 首段 Goal | 创建 `FollowTimedPath.Goal.path`。填入 `interface_version=1`、本会话 `task_id`、`vehicle_id=drone0`、`frame_id=drone0/odom`、首个正整数 revision、生成与未来生效时刻、部署允许的 `max_speed`。滚动段 `is_final=false`；完整 PVA/yaw 使用 `valid_fields=7`。相对时间从零严格递增到原段末尾 |
| 点列与 yaw | 从同一组五次曲线采样位置和一、二阶导数。对本次冻结的观测和已有参考 yaw 生成带时间的角度样本，包含首尾；下一次观测经下一版更新生效。上游随控制周期动态计算 yaw，此处的分段冻结需要单独对照。可复用现有 `timed_points` 编码函数 |
| 滚动 Modify | 通过 `/drone0/follow_timed_path/_behavior/modify` 提交下一版，沿用任务、车辆、frame 和字段集合。交接时刻严格晚于上一版起点且在其覆盖范围内。先从旧曲线在该时刻求 PVA/yaw，再解新曲线；每个会话同一时刻只保留一笔待确认提交 |
| 接纳、反馈与终止 | 服务返回 accepted 后才登记已接纳版本。用匹配任务和版本的 Action feedback 核对 `elapsed`、`waiting_for_start`；检查实际执行另看运动参考。明确拒绝保留旧段，应答归属不明则停止会话。取消通过当前 Goal handle，收取终态和 reason，旧推理结果按会话代次丢弃 |

默认执行器会对过期的非终段、失去定位或控制模式变化给出不同终止原因。保留原始 reason；例如 `CONTROL_MODE_LOST` 需对照带时间戳的 `controller/info` 才能区分实际接管与交接时的旧反馈。首版不改公共 Behavior 的模式判断或连续性参数。[默认 Behavior 实现](../../../../behavior/follow_path/src/follow_path_extended.cpp) · [现有点列编码](replay/timed_points.py)

## 6. 仿真验收顺序

| 顺序 | 输入与观察位置 | 通过判据 |
| --- | --- | --- |
| 1. 静止采集 | 在 AirSim 地面态采集同一相机的 RGB、DepthPlanar、两组内参、TF 历史、定位和时钟 | 地面配对与真实模型消费已验证。相机桥输出内参 fx=fy=160，主点为 (159.5, 95.5)，保留与模型像素约定的差异以便后续几何对照 |
| 2. 同输入回放 | 用同一段有可见仿真目标的录制数据分别运行锁定上游和拟建 Adapter，固定初始化状态和参考模式 | 对应帧能对上关联结果、候选编号、预测时长和世界系终端 PVA；首个分歧可定位。空场景记录单独验证无有效目标 |
| 3. 执行接口 | 在模拟控制器上向真实默认 Behavior 提交同一次回放实际选出的两段参考，包含一次滚动更新，然后取消 | 已用 AirSim 地面录制通过真实模型正向回放，两版待生效轨迹均按指定时刻切换，实际输出匹配原生曲线，取消后轨迹处于静默状态 |
| 4. AirSim + PX4 SITL 执行 | 通过默认 Behavior 执行固定参考，观察飞控位移、两版反馈、取消与降落 | 已通过固定参考试飞 |
| 5. 实时仿真整链 | 移动目标驱动真实模型与滚动执行，在两版确认后中断 RGB-D | 两版均实际生效，断流后执行制动、确认悬停和降落。三版持续跟踪及相机正常时的目标丢失待验收 |

固定参考的制动对照可通过 [SITL 诊断入口](replay/sitl_cancel_diagnostics.py) 选择 `reserved-brake`。它在下一拍无法连续交接到可行制动段时，向同一个 FollowTimedPath 会话提交本次的制动段。前向试验实际位移峰值 1.597 m，落在 1.8 m 试验范围内，低于 0.2 m/s 最长持续 0.777 仿真秒，原停车判据要求连续 1 秒。补齐 200 ms GPS 延迟后，固定参考试飞通过制动低速验收，悬停复核受剩余仿真时长限制。

实时入口把制动段附在每条提交曲线后。制动沿选中的三维曲线降低推进速度，首尾保持 PVA 连续，并在同一路径内留出所配置的过冲余量。下一版沿用当前已接纳曲线的制动起点作为固定交接时刻。没有及时接纳的更新时，默认执行器继续执行已载入的制动段。持续新鲜低速后，Adapter 提交最终版，从同一 Action 会话退出到悬停。
