# Mountains 地面移动目标观察验证

本仓库入口复用固定镜像与已有 Detect → Tracking → Mission 链路。
仅借鉴课程的 Shipping 场景启动及车队 RPC；不运行课程 task、飞行或模型编排。
本入口没有导航执行选项；Mission 仅观察时也不加载 navigation Module。
实际验收状态见 [验证记录](../../VALIDATION.md)。

在已有图形会话的终端设置本机 `MOUNTAINS_PACKAGE`，指向包含 `demo/`、`Engine/`
的场景包根目录。外部包、settings、路线和课程仓库始终只读。本仓库
`profile.json` 不包含本机二进制路径，由 `--scene-package` 生成忽略 Git 的运行配置。
manager 的独立 UserDir 中复制三份 `*Convoy3Route01.sav`，不修改原件。

前置条件：已有 CUDA YOLO26n-seg 独立环境与模型、检测外部工作区
`log/object-detection-tool/workspace`、Tracking 外部工作区
`log/yopov2-tracking/build`，以及固定镜像 `flight_board:v0.1.0`、
`onboard_computer:v0.1.0`。准备方式沿用 [Tracking 部署](../../README.md)。
宿主运行 Python 必须能导入 msgpack；可显式加载已有兼容依赖目录
`HOST_RPC_DEPS`，不要将含旧 pydantic 的宿主 overlay 传入模型 worker。
入口只复制 msgpack 到本轮 probe 依赖目录；worker 使用自己的独立环境。

```bash
PYTHONPATH="$HOST_RPC_DEPS${PYTHONPATH:+:$PYTHONPATH}" PYTHONDONTWRITEBYTECODE=1 \
python3 deployment/target_tracking/airsim/mountains/run.py \
  --scene-package "$MOUNTAINS_PACKAGE" \
  --output log/yopov2-tracking/mountains-observe-new
```

入口检查专用端口空闲并选择独立 DDS domain；复用就绪的 CUDA worker，或在端口空闲时
启动并最终回收本次 worker。不会停止其他容器或接管其他场景。工作站 Mission
在独立进程中运行，通过 ROS 调用本轮算力服务；两者在同一固定镜像容器中共置，
不是跨物理设备验证。AS2 平台及定位在另一容器，PX4/clock/传感器各有独立容器。

场景出生点 NED `(0,9,0)`、Yaw `-90°` 为本包道路地面测试配置，须用实际
`initial-camera.png`、`ground-camera.png` 核验皮卡可见。没有运行中无人机位姿传送。
显式 FrontCamera 外参 FRD `(0.3,0,-0.3)`，水平 RGB/DepthPlanar 640×480、90°。
桥以实际采样 stamp 发布图像、CameraInfo 和相机 TF；检测查询该 stamp 的历史 TF。
实际 AS2 自身 Pose 为 `earth`，检测输出、Tracking 和 Mission 显式保持同系。
相机 raw 颜色与压缩 PNG 的核查、真实深度、相机 pose 和桌面截图进入证据。

使用本场景 `simStartConvoy` 启动三车，`simStopConvoy` 停车，
`simGetConvoyStatus` 检查实际速度。独立 Actor pose 仅记录运动评分，
不作为检测、跟踪或 Mission 输入。当前模型把此皮卡检出为 `truck`，
此类别不保证颜色、外观身份或多目标关联正确。

运行前 `checks.json` 固定：Mission 12 s；至少4个不同 capture ID 和采样 stamp
以有效 tracked 状态被 Mission 消费；估计位置最大两两位移 >1 m；跟随几何有限，
自身高度误差 <=1e-9 m，水平间距公式误差 <=1e-6 m。失跟会提前取消并判失败，
不因数量/位移子项通过就算整轮通过。保持原0.5 s预测有效期和1 s Mission新鲜度门槛。

证据包括 `probe/mission/mission.jsonl`、summary、原始推理类别与框、三维投影、
PlatformInfo、Pose、PX4命令观察、车队状态/独立位姿、连接图、源码 SHA 和镜像 ID。
probe 退出先取消 Tracking、测量三车连续5次速度 <=0.1 m/s；宿主清理再独立检查停车，
然后回收本次容器、场景、观察相机进程与自启 worker，并核查既有容器身份未变。
任何未通过项保留失败；此入口不代表移动目标精度、故障恢复、导航或飞行验收。


## 原生起飞后悬停观察

`run_hover.py` 仅用于明确授权的模拟飞行，参数与地面入口相同，另需外部
`log/yopov2-tracking/px4-diagnostic-overlay/install` 提供与实际 PX4 v4 状态匹配的
Python消息类型；来源与哈希保存在运行证据，固定AS2镜像不替换。
先原生AS2 Takeoff，验证3m悬停和至少4个不同采集的静态目标，之后才启动车队完整
观察12s。失败时停车、取消观察、尝试原生Land并确认PX4 landed与平台/PX4解除
武装；无法确认时保留飞行支撑运行时。门槛见每轮`checks.json`。

2026-11-15 `mountains-hover-observe-02` **失败**：原生Takeoff接受3m/0.5m/s请求，
实际起飞但最高记录约2.543m，约40s后取消；未进入悬停或检测观察阶段。
AS2进入EMERGENCY后拒绝原生Land；PX4记录failsafe/RTL、落地和解除武装，独立
恢复事件确认fresh landed/disarmed。恢复到地面不能算原生Land通过。车队未启动，
宿主连续5次实测三车速度均为0。所属资源已清理，既有两个容器身份和StartedAt未变。
接管后仅诊断和修复记录器，没有再次飞行或改变门槛，旧轮次完整保留。

PX4数组直接转list保留NumPy float32，使旧JSON写盘抛异常；真实消息回调隔离
回归已复现并修复。当前入口保存`probe.log`，节点销毁前保存结果并逐项记录销毁
异常。1项真实ROS消息测试及7项Mission测试通过，修复尚未重新飞行验收。
审计索引：`log/yopov2-tracking/mountains-hover-audit-20261115/audit.json`。

## 2026-11-15 起飞根因修复与当前验收

已实测定位旧起飞失败：`settings.json` 的 Y=9 出生点位于 `02Armored-V` 车底。
诊断轮197个起飞物理样本报告该车碰撞；实际只上升约0.35m，估计高度却漂移到
约2.5m，PX4仍收到约0.485m/s向上参考。原生position起飞插件正确使用当前z+3m，
不是绝对高度/dz用错、超时过短或控制器不再给爬升参考。

飞行入口改用独立 `hover_settings.json`：Y=6避开车底，Pitch=-20使皮卡完整入镜。
地面入口继续使用原 `settings.json`。原3m/0.5m/s、40s Action、悬停与检测门槛不变，
固定镜像不变，不修改上游。`run.py --flight-only` 提供独立原生起飞/悬停/降落门，
先通过后再运行 `run_hover.py`。`--scene-audit-only` 是不解锁的几何辅助检查；
可见性射线没有检出本次动态车辆物理碰撞，不能用它单独证明净空。

最终相机配置的独立飞行通过，实际相对悬停高度2.906–3.075m，原生Takeoff、
悬停、Land及landed/disarmed全部通过。完整观察重试的飞行阶段也通过，
但静态目标门失败：51次真实CUDA调用返回空框。保存的实际输入经同一CUDA后端
回放，truck最高约0.017，低于既定0.35阈值；最高类别为train约0.535。
不修改类别映射或降低阈值。车队没有启动，12s移动观察未执行，总体验收仍未通过。
两次完整观察失败后均完成停车实测、原生Land、解除武装和所属资源清理。

[验证报告](VALIDATION-hover-root-20261115.md) 区分已解决的起飞问题与未通过的检测门。

## 2026-11-16 专属侧视实验

`--settings-file deployment/target_tracking/airsim/mountains/side_settings.json`
选择X2.6/Y0/Yaw180°侧视地面初态，实际相机FRD(.8,0,−.6)、Pitch−55°。
该配置独立于现有默认地面/悬停入口；先`--flight-only`通过，再使用
`--hover-observe --mission-category car`，每轮使用新输出目录。
支持的任务类别仅car/truck/bus，参数不改变模型类别映射或身份规则。

实际四次原生飞行通过，两次完整观察共101次真实CUDA均无受支持框；
全类别诊断将车辆区域判为microwave，并未重映射为车辆。静态0/4，车队未启动，
12s移动观察未执行。实际标定/采样TF核对、停车、Land/disarmed和清理通过。
详细矩阵、失败及复验见[侧视验证](VALIDATION-side-view-20261116.md)。
`diagnose_classes.py`只对保存的真实RGB输入做同阈值全类别CUDA回放，不向Tracking送入诊断类别。
