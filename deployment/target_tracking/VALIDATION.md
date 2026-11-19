# Target Tracking 验证记录（2026-11-15）

总体状态：`partial`，完整复现未完成。已交付只观测 Tool 和工作站 Mission 样例；Mountains 实际观察验收失败，不含导航执行或飞行通过结论。
锁定上游 YOPO-Tracker `d313f8610195dc986caceff7a3616f63d7e6f31f`，MIT；
原始 `target_ekf.py` 与许可证原样安装，并以 `cmp` 检查安装内容。

| 适用项 | 结果与证据 |
| --- | --- |
| 固定共享镜像构建 | 五包通过：Inference、Object Detection、Tracking 接口及 Tracking Adapter/API；未重建镜像 |
| 干净外部工作区 | `log/yopov2-tracking/clean-build.log`，五包从零构建并调用真实 EKF |
| 最终核心回归 | 6项通过，200步固定 dt 上游对照 atol=1e-10 / rtol=1e-9；变步长 A/B、确认/失跟/重捕获、错误拒绝、外推不改滤波状态 |
| 最终真实 ROS/DDS | 5项通过，无 skip：有限墙钟超时、反馈、单会话、取消、晚回复世代隔离、无效 Goal、过期/未来/无三维/空检测、ROS clock reset；`log/yopov2-tracking/final-tests.log` |
| AS2 Python Module | 真实 DroneInterface 加载与调用；实现抽象 `__call__`，销毁幂等；与当前锁定 AS2 宿主实际运行 |
| 真实静态平台输入与消费 | 历史 ground-02 完整通过四次有效 tracked、取消、前后 disarmed 与清理；仅代表该轮对应源码 |
| 最终平台回归 | `airsim-ground-05` 通过：实际 cuda:0、四个不同 capture/采样时刻、AS2 Module 消费、标准取消、重复 destroy、首尾 disarmed、清理完成；命令退出0 |
| 移动目标精度/身份/遮挡 | 未完成；未声明位置/速度/协方差校准或身份保持精度 |
| 真实平台故障/重启/reset 恢复 | 未完成；合成 DDS 故障通过不能提升为平台故障通过 |
| 跟随 Mission/导航/飞行/真机 | Mission 已实现并真实运行，Mountains 观察失败；导航执行、飞行及真机未验收，Tool 不执行运动 |

最终合成构建使用已有 `onboard_computer:v0.1.1`（`c8756265a76d`），
真实地面 Harness 复用已有 `flight_board:v0.1.0` 与 `onboard_computer:v0.1.0`，
挂载匹配的检测 overlay 和本次 Tracking overlay；不是新镜像或跨物理设备验收。
实际镜像、配置、源码 SHA、DDS domain、平台日志和反馈在各轮 evidence 目录。

原始证据统一位于 `log/yopov2-tracking/`，过程记录位于
`skill/temp/yopov2-tracking/PROCESS.md`。日志不随 Git 分发，验收状态以本摘要为准。

失败与修正：

- `build-first.log`：colcon 拒绝绝对 `data_files`；改用指向锁定子模块的相对路径后通过。
- `build-ros.log`：两项测试夹具用了 int 填 float64 字段；修正类型后运行全部 DDS 测试。
- `airsim-ground-01/`：真实 CUDA 和四次跟踪业务断言通过，但重复销毁 ActionClient，
  整轮清理失败；不能判为完整通过。`ground-02` 完整通过。
- `clean-build-path-failure.log`：可选输出相对路径被 Docker 当作 volume 名；规范成绝对挂载后通过。
- `build-clock-jump.log`、`airsim-ground-03/`：目标 Humble 的 JumpThreshold 要求显式
  `min_forward=None`，首次遗漏；修正后全部11项测试通过。保留该平台失败轮。
- `airsim-ground-04/`：12秒窗口中只收到两次瞬时 tracked，四次门槛失败；
  最后 capture_id=59，不代表没有观测。进一步的 `feedback-status-red.log` 真实 DDS
  回归复现了状态只保持一帧的问题。修正为最新观测成功且仍有效时保持 tracked，
  缺测/请求失败后才在有效期内 predicted。`final-tests.log` 回归通过。
  门槛仍要求四个不同 capture 与采样时间，禁止用重复反馈计数替代独立观测。

Clock reset 使用 ROS 跳变回调及每轮时钟检查；未来/过期/旧世代回复仍拒绝。
没有为平台通过而放宽0.5秒有效期、观测年龄或重置门限。

最终轮证据：`log/yopov2-tracking/airsim-ground-05/probe/result.json`，
`passed=true`、`tracked_count=4`、`cleanup_complete=true`；对应配置、源码 SHA、
实际镜像与角色日志均在同轮目录。`source-manifest.json`、`environment.json` 和
`evidence-sha256.json` 保存最终源码、固定镜像及证据校验值。
该轮为静态地面输入与实际消费通过，不提升为移动目标完整复现或飞行通过。
有屏入口以 windowed 参数启动且图形会话可访问，但本次没有保存场景截图/录像。
所有自建场景/容器已回收，本次自启 YOLO worker 已停止；原有容器保留。

## Mountains 移动皮卡观察（2026-11-15）

**整轮未通过，完整复现仍为 partial。** 新增本仓库
[专用入口](airsim/mountains/README.md)，使用外部 Shipping 二进制与三份路线的
只读来源，manager 独立 UserDir 中复制路线。没有修改外部场景或课程仓库，
没有运行课程飞行 task，也未重建共享镜像。实际链路为本项目真实 RGB-D →
YOLO26n-seg CUDA → DetectSnapshot → Tracking → AS2 Python Module → 实际 Mission。
工作站与算力进程在一个固定镜像容器共置；AS2、PX4、clock 和传感器另有容器，
不代表跨物理设备部署已验收。

预声明门槛：12 s 完整 Mission；>=4个不同 capture ID 和 observation stamp 作为
有效 tracked 状态被 Mission 的 follow_point 消费；目标估计最大两两位移 >1 m；
几何/距离有限，保持自身高度误差 <=1e-9 m，水平间距公式误差 <=1e-6 m。
原0.5 s预测有效期及1 s Mission新鲜度不变。失跟、提前退出或缺证据均判失败。
真实 Actor pose 仅作独立运动评分，不输入检测、滤波器或 Mission。

| 项目 | 实测与结论 |
| --- | --- |
| 真实场景与车队 | 三车可用，simStartConvoy 实际驱动车辆；run08独立皮卡位移8.711m，不能代表跟踪成功 |
| 地面视点 | 最终 NED出生点(0,9,0)、Yaw -90°，无运行中无人机传送；原始相机图像清楚显示皮卡。run07尝试的碰撞截断地面传送保留为失败 |
| 相机/时间/坐标 | 显式FRD外参(0.3,0,-0.3)、640×480 RGB/DepthPlanar、90°；真实采样TF查询、registered depth。实际自身Pose为earth，Detect/Tracking/Mission统一earth，不改标签冒充转换 |
| GPU与颜色 | 真实cuda:0，worker实际处理推理请求；run12 raw与PNG BGR MAE1.294，交换通道10.378。连续两次采集存在曝光/渲染差异，不能称逐像素原子对照 |
| 类别 | run09原始推理记录确认truck；car类别轮未形成有效跟踪。truck不等于黄色身份或跨目标关联已验收 |
| Mission修正 | observe-only不加载navigation；公开spin_rate=1000提高AS2一次一回调的处理能力；发TrackObject前在既定初始预算内等待新鲜自身Pose；记录实际目标采样、own、range与followpoint |
| run10数量/运动子项 | 8个不同tracked采样、20个followpoint，估计位移1.026m、几何有限/高度保持；随后观测过期、Mission提前取消，整轮失败，不能缩短12s窗口获得通过 |
| 最终run13 | 1个不同tracked采样/capture、1个followpoint、估计位移0m；独立皮卡位移0.990m。导航模块未加载、motion_requested=false；跟踪有效性/新鲜度联合保护提前退出，Mission退出1，整轮失败 |
| 最终几何子项 | 唯一followpoint有限、保持自身实测高度、水平间距公式通过；数量和位移不足，不作为跟随成功 |
| 最终安全采样 | 46条PlatformInfo全为disarmed；probe窗口PX4命令观察中飞行/模式/解锁命令数0；Action图未提供本项目NavigateToPose，无导航执行标志。命令订阅不覆盖其启动前的AS2初始化窗口 |
| 停车和清理 | probe与宿主分别验证三车连续5次速度<=0.1m/s，非仅ACK；Tracking取消完成；本轮容器、场景、观察相机进程及自启worker回收；两原有容器ID/启动时刻未变 |
| 有屏证据 | run12/run13 desktop.png为实际场景窗口，ground-camera.png为算法前视相机；未录制视频 |
| 检查 | 六项Mission纯逻辑测试、Python语法与git diff --check通过；不替代失败的真实Mission验收 |

证据根：`log/yopov2-tracking/mountains-observe-01` 至 `-13`，全部失败保留。
最终主要文件：`mountains-observe-13/checks.json`、`sources.json`、`images.json`、
`probe/result.json`、`probe/mission/mission.jsonl`、`probe/projection-calls.json`、
`probe/inference-calls.json`、`probe/platform.json`、`probe/px4-commands.json`、
`probe/convoy.json`、`host-convoy-stop.json`、`cleanup.json`、`desktop.png`。
run10仅作较好子项证据，不能替代run13或完整12秒验收。

前置失败也保留：01/03宿主缺msgpack，02端口TIME_WAIT检查，04宿主pydantic串扰，
05对象pose RPC参数数量，06计算镜像缺msgpack，07frame/地形，08错误类别，
09自身Pose新鲜度，11原始bytes证据解码，12自身Pose发现未完成。
最终run13的退出文字合并多个有效性/新鲜度谓词，未逐谓词记录，
不能断言该轮具体是检测断流、未来stamp、状态过期或传输延迟导致；
run10的过期状态有独立日志证据。下一步应记录失败时目标/自身/clock的同一快照和
逐谓词结果，再定位连续观察失效，不放宽门槛。移动目标身份、精度、遮挡、
平台故障恢复、导航、飞行和真机均未完成。

过程：`skill/temp/yopov2-tracking/MOUNTAINS-OBSERVE.md`。
已回顾并更新正式 `uav-platform-sensor-adaptation` 与 `uav-sim-e2e-validation`：
实际frame/自身Pose就绪、AS2回调处理能力、Python依赖隔离、移动观察矩阵及实测停车。
未创建新Skill。无commit或push。


## Mountains 原生起飞后悬停观察接管审计（2026-11-15）

**FAILED / partial，未再飞行。** 已读取正式component、平台传感器、AS2接入与
端到端验证Skill及其契约/平台参考。此前`hover-01`状态话题缺失，`hover-02`和
`hover-observe-01`构造阶段clients命名冲突；保留旧证据，不重判通过。
`hover-observe-02`已修复命名并加载PX4 v4 Python消息overlay，实际进入原生起飞。

| 验收项 | 证据与判定 |
| --- | --- |
| 原生Takeoff/PX4 | AS2接受相对3m、0.5m/s；PX4 Armed/Takeoff detected。Takeoff阶段408个样本，最高earth z=2.5434265m，目标z=3.000816m；约40s后取消，无成功结果 |
| 悬停 | 未达3m±0.3m，未进入悬停验收；失败 |
| 静态RGB-D/4次有效目标 | 地面与失败RGB-D/相机pose保留，地面图可见皮卡但上部明显遮挡；未开始Tracking/Mission，0次已确认观测，未完成 |
| CUDA | cuda:0 worker就绪；本轮无检测调用证据，不算真实算法观察通过 |
| 车队移动12s | 静态门未通过，车队未启动，移动观察未执行 |
| 停车 | host-convoy-stop.json确认三车连续5次速度0m/s，通过 |
| 原生Land | Takeoff取消后AS2进入EMERGENCY，Land因EMERGENCY→LAND被拒，失败 |
| 地面解除武装 | PX4日志记录failsafe/RTL、Landing detected、Disarmed by landing；独立host-recovery/flight-events.json的disarmed_ground事件含fresh platform/mode/status/land，确认已落地解除武装 |
| 清理 | 旧cleanup及接管runtime-final.json均无所属容器/场景/relay，原两个容器相同ID/StartedAt仍运行 |

爬升约2.52m停滞的根因未定。High Accelerometer Bias告警与飞行同时出现，不能
据此建立因果；未调整控制参数、延长40s预算、改变3m目标或放宽任何门槛。
失败截图显示近处结构遮挡，尚不能认定遮挡物身份或飞行停滞原因。

记录器缺陷已用真实固定镜像、真实PX4 TrajectorySetpoint回调、禁用网络的容器
复现：float32不能JSON序列化，旧flight-events和最终摘要缺失。修正数组元素转
Python float，新增真实回调回归；probe标准输出/错误保存，结果先于节点销毁落盘。
回归1项和Mission安全测试7项通过、0跳过；该代码版本尚无新飞行验证，不能将旧轮次
改判成功。未使用物理设备、导航、AirSim飞行/传送API，未写外部场景/课程源码。

原始证据：`log/yopov2-tracking/mountains-hover-observe-02/`（保持原样）。
接管派生审计：`log/yopov2-tracking/mountains-hover-audit-20261115/`，含audit.json、
旧证据SHA256、测试摘要和最终运行时清单。正式Skill已回顾：component补充真实消息
记录器检查；e2e补充起飞取消→原生Land拒绝与独立PX4落地的区分。平台传感器和AS2
接入已有遮挡/实际输入、发布权、真实宿主验证规则，本次无新增已验证机制，无需改写。

## Mountains 侧视原生悬停观察（2026-11-16）

**FAILED / partial。** 专属侧面初态修正了首次护栏碰撞，四次原生Takeoff3m/hover/
Land/landed-disarmed通过；两次完整观察共101次真实RGB-D CUDA调用均无受支持车辆框，
有效静态跟踪0/4，车队未启动，12s移动观察未执行。全类别诊断实际把车辆区域判为
microwave0.75–0.84，没有降低0.35阈值或重映射类别。物理相机与实际采样TF核对通过。
三车实测停车连续5次、所属资源清理及两个既有容器身份保持均通过。
完整证据与复验见[侧视报告](airsim/mountains/VALIDATION-side-view-20261116.md)。
