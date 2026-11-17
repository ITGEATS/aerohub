# AirSim + PX4 SITL 核查要点

适用于当前项目的平台接入。以下是核查入口与已知边界，不是已经全部跑通的部署流程。

## 数据来源分开核实

| 通路 | 核查重点 |
| --- | --- |
| AirSim → PX4 SITL → AS2 平台适配 | 飞控收到的仿真测量与输出的估计状态要分开；PX4 内部有某 uORB 话题不代表已通过 DDS/MAVLink 暴露，须核对目标版本配置 |
| AirSim RPC → 传感器桥 → ROS | 图像/CameraInfo、激光点云等可独立提供给算法，不要求绕经 PX4；IMU 若直接使用 RPC，需明确其与 PX4 所用 IMU 的实例、噪声和时间关系 |
| 算法输出 → AS2 状态插件/感知接口 | 位姿与速度不等于地图；插件发布正常不代表输入正确或 PX4 已融合外部定位 |

AS2 的 IMU topic 名不能证明数据未经处理；检查角速度、加速度、orientation 的实际来源。算法需要 IMU 测量时，不以 PX4 已估计姿态替代。PX4 外部里程计回灌是另一个有方向的接口，需要单独配置和验证；它不是传感器桥自动附带的功能。共享 IMU 时还需考虑估计相关性，不能将两路结果视为独立测量随意融合。

当前入口：

- `simulator/airsim/config/`：场景、传感器与资源配置。
- `simulator/airsim/sensors/`：RPC 与 ROS 传感器桥。
- `deployment/flight_board/`：PX4 SITL、消息及传感器/时钟桥环境。
- `src/basic_function/perception/navigation_inputs/`：导航坐标注册；不是通用雷达逐点去畸变。
- `src/basic_function/state_estimator/fast_livo2/`：算法、原生插件和现有输出桥；检查是否另有输入桥，不推定输出桥能反向使用。
- `src/basic_function/state_estimator/pin_slam/`：复用现有标准 ROS 2 LiDAR 话题的算法 Adapter 与状态插件；不要再增加第二个 AirSim RPC 读取器。

先确认 scene 二进制、AirSim/PX4 源码版本与实际配置一致。源码中存在功能不证明下载的预编译场景包含它。检查仿真时间与 PX4 时间如何映射、暂停/reset 后是否清空旧数据，以及各车辆 TF/话题是否隔离。

## FAST-LIVO2 案例的已验证边界

项目使用统一 AirSim RPC ROS 2 传感器桥提供 IMU、RGB 图像和 PointCloud2，再由 FAST-LIVO2 Adapter 转为锁定算法接口。AirSim 1.8.1 没有逐点采样时间，适配明确使用 instantaneous scan，不按扫描角速度伪造时间；仿真推导属性不能描述成真实硬件回传值。

坐标约定已用平台数据验证：AirSim IMU 和 LiDAR 的局部 FRD 转换为 FLU，相机保留光学坐标；LiDAR 到前视相机的矩阵为 `x_cam=-y_lidar, y_cam=-z_lidar, z_cam=x_lidar`。车辆沿 AirSim 世界 `+Y` 平移 1 m 的点云对照中，原始 FRD 补偿的中位配准误差为 `0.285 m`，错误横轴翻转为 `0.551 m`，因此不得为了修正算法轨迹而再次翻转横轴。

实际有屏 shadow 飞行证明三种测量进入真实算法，配准点云由 ROS 2 Perception 消费，状态误差通过预设 `0.5 m` 门限；随后 FAST-LIVO2 作为 AS2 状态/TF 权威完成闭环飞行，并在传感器中断和算法退出后受控降落。PX4 的航向 preflight 偶尔无法稳定，必须在启动上层链前用实际状态消息拒绝该轮运行，不能仅依赖启动日志或关闭检查。

AirSim 射线循环没有可供算法恢复的逐点时间，不能将其直接等同真实旋转雷达。后续诊断应优先保存可重放的原始三传感器记录，分开评估扫描模型、场景可观性、内外参与估计器参数；不要用合成字段、真值状态或放宽门限替代缺失语义。

PIN-SLAM 的当前 AirSim 接入直接消费标准 FLU XYZ PointCloud2。因为该帧是瞬时射线结果且没有逐点时间，配置必须保持 `deskew: false`，也不生成 ring、强度或时间字段。适配器在进入 GPU 前拒绝错误 frame、非单调时间、空帧和非单值 FLOAT32 XYZ；这些接口检查通过仍不代表神经地图或定位精度通过。

## 大图像跨容器传输与状态就绪

640×480 RGB8/32FC1 分别约0.92/1.23 MB。配对失败时，同时记录同一
`simGetImages` 批次的源stamp与消费者实际收到的stamp；源同步正常不能证明
DDS消费者收到完整配对。保持既定同步门限，先区分采样偏差与传输丢帧。

当前 Humble 固定镜像预置 UDP 配置；核查实际容器环境中的
`FASTRTPS_DEFAULT_PROFILES_FILE` 和 `FASTDDS_BUILTIN_TRANSPORTS`，显式使用
仓库 `deployment/airsim/config/dds/fastdds_udp.xml`。与 XRCE Agent 通信时，
现有 host 网络 Demo 使用 `ROS_LOCALHOST_ONLY=0`；逐项检查 PX4里程计发布端、
AS2里程计消息和历史TF，服务发现或周期发布的PlatformInfo不足以证明状态已就绪。

YOLO地面接入在 best-effort 下出现RGB/depth分别丢失；仅尝试共享IPC和builtin
传输没有解决。共享UDP配置、相机桥 `camera_reliable:=true` 与检测端同名参数
同时启用后，实际640×480、5 Hz输入通过10帧配对检查（最大偏差9 ms）。
这是当前版本的地面短时证据；其他负载与远端网络需重新测量。Reliable发布端
配best-effort订阅端不能当作已经验证的可靠链路。

新建消费者先等待实际深度stamp处的历史TF，再请求检测；保存每次返回的
reason及逐目标position_error。晚到TF可能超过算法现有等待预算，即使返回后
能够查询，也不能抹去该次三维测量失败。RGB与2D检测接通、三维测量有效、
检测精度和飞行闭环分别验收。可复验入口与当前边界见
`deployment/object_detection/airsim/README.md`。


## 原始颜色与静态RGB-D评分

AirSim Scene非压缩字节顺序须以实际二进制核对。可在传感器桥启动前同一次RPC
读取raw Scene与压缩PNG，按PNG解码后的BGR逐像素比较，再比较通道交换结果。
2026-11-15当前场景raw BGR与PNG解码MAE为0，通道交换约18.7，YOLO专属入口
因此显式配置 `source_color_order:=bgr`，共享桥仍输出rgb8。
源码描述、消息encoding与能检出目标三者均不能替代原始颜色核对；其他场景版本重测。

静态近距目标评分可用独立场景碰撞射线定位可见表面，保留原RGB、实例掩码、
深度、内参、实际相机pose及坐标换算。Actor原点不等于表面；碰撞几何也可能与
渲染表面不同，须报告该限制。采样时刻TF往返残差只能检查代数一致性，不能
充当独立的绝对定位精度。近目标有效且同帧远目标无有效深度时，应逐对象验收，
按公共契约保留PARTIAL，而非要求每个检出对象都有三维坐标。

输入就绪同时核查匹配stamp的深度、两路CameraInfo和历史TF。诊断间歇TF晚到时，
分别记录状态源/TF接收wall时刻、ROS clock、请求stamp与投影起止时本地最新TF；
在原等待预算内检验恢复和持续缺失拒绝，后续连续通过不抵消已有失败。


若独立best-effort观察者的TF/里程计新stamp持续推进，而算法Buffer停在旧stamp，
优先核对该消费者的动态TF QoS与调度。Humble TransformListener默认动态TF
reliable/depth100；YOLO地面对照中其Buffer落后537–735ms，独立观察者仅0–21ms。
该Tool改用sensor-data best-effort/depth5后，相同100次调用TF超时由4降为0，
仍有2次深度配对失败。仅调整动态TF，静态TF保持可靠transient-local，Buffer
继续按采样stamp查询和原预算等待；此对照不是长期网络可靠性或所有TF故障保证。
