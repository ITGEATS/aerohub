# WestStreet：PC 模拟算力板验证

当前维护入口：[`launch_before_plaza.py`](launch_before_plaza.py)，任务实现为
[`demo_before_plaza.py`](demo_before_plaza.py)。使用方法、side 会话修复及验收边界见
[恢复版说明](README-before-plaza.md)。`demo.py` 仅委托到同一实现。

恢复版在 Demo 内组合共享 `TrackObject` 和 Navigation `to_point()`。其相对位置、
短时失跟冻结与目标点更新属于本场景的跟随策略。Elastic 的滚动时间化轨迹由独立的
[Following Tool 入口](../../../../src/tool/navigation/planner/elastic_tracker/README.md)提供。
若以该入口验收 WestStreet，Mission 只需管理场景、起降和 Following Goal；
同一任务中的运动更新由所选 Following 实现持有。

恢复版使用专用 `log/weststreet-before-plaza/runtime/`，请保留该目录。
此前失败的柱区航点、空中转场和预置分支已移除；不再提供对应启动参数。
下面的 System 短程入口与历史实验说明不代表当前追踪默认配置。

运行链为工作站 ROS 任务容器 → PC 上的 onboard_computer 容器 → AS2 原生 Behavior / 控制器 / 平台 → flight_board 容器中的 PX4 SITL → 主机 ProjectAirSim。无人机状态由 PX4 提供。HUD 显示无人机，Global 可观察已配置的行人跟随相机。

复用固定镜像 `onboard_computer:v0.1.1`、`flight_board:v0.1.1` 和 `osrf/ros:humble-desktop-full-jammy`。无需重新打包场景；使用已安装插件的 UE5.2 工程以 `-game` 启动。

`run.py` 要求已有通过 PX4 地面检查的 manager profile，复用机器人传感器及磁场配置；另指定 UE 根目录、WestStreet uproject、工作站接口 overlay 和全新证据输出目录：

```bash
python3 deployment/airsim/demos/weststreet/run.py \
  --base-profile /path/to/verified/profile.json \
  --ue-root /path/to/UE5.2 \
  --project /path/to/WestStreet/PAirSim.uproject \
  --workstation-overlay /path/to/workstation \
  --output "$PWD/log/projectairsim/weststreet-framework/run-new" \
  --ground-only
```

overlay 内应存在 `ws/install/setup.bash`，由相同版本的 `as2_msgs`、其 `geographic_msgs` 依赖在工作站镜像内通过 colcon 构建。容器内构建位置应为 `/work/ws`，源仓库挂载为 `/repo`。AS2 来源为仓库锁定子模块；不要混用其他版本的消息定义。

移除 `--ground-only` 即执行飞行：重置行人并暂停 → 解锁 / offboard → AS2 Takeoff 3m → 确认实测高度 → 启动行人 → AS2 GoTo 沿初始朝向前飞 5m → 实测速度小于 0.25m/s 持续 2s → AS2 Land → 确认 disarm → 暂停行人。任务客户端不发送仿真运动指令。此短程测试不代表街区避障或无碰撞验收。

场景需包含 `/Game/Aerohub/WestStreetCircuit` 及 AerohubDisplay、AerohubSceneControl 插件。人物使用统一 `ProjectAirSimExtensionClient(scene_port=19892)` 的 `reset_actor/start_actor/stop_actor/get_actor_state`。RPC 失败会记录任务错误，不冒充飞行成功。

本入口使用 DDS domain 197、PX4 instance 30 / system 31，及端口 4572、14552、14592、18991、19889–19892；运行前保证这些资源空闲。容器名含本次时间戳。失败证据包括 `mission.log`、`onboard.log`、`px4.log`、`mission/result.json` 和 `cleanup.json`。任务构造前失败可能没有 result.json，应先读 mission.log。

清理前读取实际 PX4 解锁状态；无法确认 disarm 时保留仿真器和算力板执行链用于恢复。不要在无人机仍解锁时强制结束整个链路。日志保留每次运行的成功、失败及未完成状态。

## 2026-11-16 实测

`log/projectairsim/weststreet-framework/ground-02/` 地面检查通过。
`log/projectairsim/weststreet-framework/flight-04/` 完成框架短程飞行：
Takeoff、GoTo、Land 均返回成功；实测起飞高度 2.947m，前飞目标 5m，
停稳后端点误差 0.087m、速度 0.045m/s，最终 PX4 arming_state=1。
人物起飞后启动，最终在路线 19.189s 处暂停；本轮容器和场景已停止。
统一客户端 8 项测试通过。

降落后的 `set_offboard_mode(false)` 被原生平台拒绝（该平台不允许算力板切回 MANUAL）；
该返回记录为 release_error，实际 PX4 disarm 已另行核实，不以此服务返回判定着地/解锁。
首轮飞行因模式切换期间状态超时失败，使用 PX4 降落命令恢复，证据保留在 flight-01，
不计为框架验收成功；flight-02/03 在启动端口检查阶段失败。

## 跑步人物感知：YOLO 与显式测距方式

`run.py --perception-only` 使用真实前视 RGB + planar depth，算力板容器运行
YOLO ROS Adapter 和 DetectSnapshot，主机独立 CUDA worker 使用已有 YOLO26n-seg。
无人机保持 disarmed；人物暂停采样后短暂跑步，再通过统一客户端暂停。
在上面的完整运行命令中增加 `--perception-only --query-workspace /path/to/query-workspace`，使用新的输出目录。

默认 `--range-method depth` 保留现有掩码深度算法及所有同步和深度范围限制。
切换尺寸先验时显式增加 `--range-method height --target-height 1.7`；1.7 m 是本例
声明的名义人体高度假设，尚未测定为 Nathan 跑姿的真实可见高度，不从 query 推导。
生产检测节点对应参数为 `position_method:=known_height_vertical_v1` 和
`target_height_m:=1.7`；默认 `position_method:=backend_default` 仍使用原后端策略。
height 模式只用于 person、完整且足够大的框、近似直立的前/侧视人体。
它使用实际 RGB 内参与采样时刻 TF，考虑相机俯仰，拒绝截断框和不适用的俯视几何。
它估计垂直中心，深度方法测得的是表面点；二者不是同一种三维测量。
`position_method` 区分来源；兼容字段 `depth_m` 在 height 模式表示估计 optical Z，
`depth_header` 保持空、`valid_depth_pixels=0`，不伪造深度测量。
这是本项目的 pinhole 尺寸先验实现，不声明复现 Elastic-Tracker。

本入口用只读源码路径显式覆盖检测 Python 包，复用 query overlay 的匹配生成接口；
不会修改原工作区共享 install。普通部署应将新 Python 消费者安装到新的局部 overlay。
人物控制仍是独立仿真 RPC，无人机运动仍应走 AS2/PX4。

### 2026-11-16 感知阶段证据

- `log/projectairsim/weststreet-tracking/perception-03/`：暂停和跑步均识别人，三维因
  CameraInfo 不可用失败，保留原证据。原生 CameraInfo 只在初始化发布，桥上游是
  best-effort；已修复归一化桥的订阅 QoS，并按解析后的原生零畸变 pinhole 配置发布
  timeless 静态标定，保留图像采样 stamp，核对实际图像尺寸及收到的原生标定。
- `perception-04/`：真实 CUDA + 原生 RGB-D，暂停 6/6、跑步 6/6 都检出 person；
  暂停 5/6、跑步 2/6 有有效深度位置。失败包括 50ms 同步门和原掩码深度算法
  10m 上限，未放宽。该轮旧版 `success=true` 仅代表静态识别/测距门，不能提升为
  动态测距通过；当前脚本已分别要求静态、跑步阶段的识别和定位门。
- `height-replay-01/`：身高先验几何 5 项单元测试通过；真实 GPU 检测结果的离线
  几何回放可计算两个远处失败样本约 13.9/14.1m。回放 TF 来源是原配对 depth stamp，
  与 RGB 有不超过50ms偏差，因此不是实时 height 模式的 capture-time TF 验收。
- height 模式实时 ROS 已在下述 height-live-02 验证；目标 EKF 消费、飞行追逐与异常恢复均未验证。最终无人机
  disarm、本轮容器和 worker 停止。不能把这些地面结果当成持续跟随或避障通过。

离线重算（不启动模型、场景或飞行）：

```bash
src/basic_function/inference/models/yolo26n_seg/.venv/bin/python \
  deployment/airsim/demos/weststreet/height_replay.py \
  --input log/projectairsim/weststreet-tracking/perception-04/perception/result.json \
  --height 1.7 --output log/projectairsim/weststreet-tracking/height-replay-new/result.json
```

### 身高估距与独立仿真真值核对

`--perception-only --range-method height --target-height 1.7` 会运行实时身高估距，
并通过 profile 的 `client_python` 独立启动只读真值记录器 `range_truth.py`。
检测与估距不接收人物真值。评分按最高置信度 person 选择目标，不按真值挑框。

比较口径为相机到人物的水平距离：人物 RPC 是根位置，而估距是身体中心，
直接比较三维距离会混入高度口径差异。真值相机位置来自原生无人机
GetGroundTruthKinematics 加本入口固定的 FRD 外参 `[0.6,0,-0.8]` 米；
UE 厘米位置转为原生 NED 米。估计距离由同一 RGB stamp 的输出位置减相机 TF 得到。
评分脚本专用于本入口当前外参；修改相机时须同步修改或重新配置评分器。

独立记录器用 GetSimTime 前后夹取人物/无人机 RPC，以区间中点标记并在相邻
真值间插值；超过150ms间隔、状态转换和缺少前后真值的图像拒绝评分。
这不是原子化的图像同步真值。`height-live-02` 相邻记录间隔约35–60ms，
RPC区间最多57ms；5m/s时该时间区间对应最多约0.29m位移，不能声称厘米级精度。

2026-11-16 `log/projectairsim/weststreet-tracking/height-live-02/` 实测：

| 阶段 | 唯一有效帧 | 真值水平距离 | 平均绝对误差 | 相对绝对误差中位数 | 最大相对绝对误差 |
| --- | --- | --- | --- | --- | --- |
| 暂停 | 6 | 7.40m | 0.63m | 8.8% | 9.1% |
| 跑步 | 6 | 7.71–13.79m | 0.86m | 8.2% | 11.1% |

名义身高1.7m，未根据本批真值校准。预先写入评分代码的简单检查门为每阶段
至少4个不同RGB帧、相对绝对误差中位数≤20%、最大≤30%，两阶段均通过。
这是约1.5秒跑步片段的地面测距检查，尚未验证整条路线、遮挡或无人机追踪。
`perception/result.json` 保存实时检测/TF证据，`truth.jsonl` 保存独立真值，
`range-check.json` 保存逐帧误差。模型与本轮容器已停止，PX4未解锁。
首次height-live-01因参数初始化遗漏未进入观测；已修复。
第二轮实测完成后因容器root创建的perception目录不可由宿主写入评分而退出；
评分路径改为宿主创建的运行根目录，已对原始数据重新评分，无需重复采集。

```bash
python3 deployment/airsim/demos/weststreet/range_truth.py \
  --score log/projectairsim/weststreet-tracking/height-live-02/truth.jsonl \
  --evidence log/projectairsim/weststreet-tracking/height-live-02/perception/result.json
```

### Tool 任务 demo（2026-11-16，闭环验收未完成）

新增 `demo.py`：工作站仅调用 `drone.system`、`drone.target_tracking`、
`drone.navigation`。`mission.py` 是之前的原生 Behavior 验证入口；本 demo
使用独立入口。算法、模型、平台准备、飞行执行均不放入任务脚本。

顺序：Tool 就绪 → System 起飞3m → System 查询实际高度与坐标系 →
Tracking 识别 person → 一个 Navigation HOLD 会话，每0.25s更新跟随位置 →
取消导航并确认停止 → 取消Tracking → System降落并确认disarmed/idle。
任务窗口默认20s，初始人物到无人机方向固定8m偏移，保持起飞后实际高度。
这是固定世界方向的跟随位置策略，尚非沿人物路线后方的跟随，也不保证可见性。
目标观测超过0.6s、无效、有效期届满、坐标系变化或任务异常即停止导航。
未确认停止则不交接到降落，保留算力执行供恢复，不把该状态判为成功。

当前 `follow=False`；保持接管yaw。Tracking 已拒绝 `follow=True`。朝向应由跟随
实现内部协调，不能另起运动会话与本demo导航争用资源。转弯后持续看人仍有缺口。

构建任务客户端（固定镜像、独立工作区、无网络、不发起飞行）。
`WESTSTREET_CLIENT_WORKSPACE` 指向运行配置中的 `client_workspace`。

```bash
bash deployment/airsim/demos/weststreet/build_demo_overlay.sh "$WESTSTREET_CLIENT_WORKSPACE"
```

运行前须在算力侧部署匹配版本 System、SUPER Navigation Action、Tracking、
YOLO检测与身高估距，提供真实相机/CameraInfo/TF、定位、SUPER激光/地图和时钟。
System与SUPER共用持久 `AEROHUB_MOTION_LOCK_DIR`。
工作站使用既有ROS/AS2底层，将本次客户端install挂载进来后运行：

```bash
export DEMO_CLIENT_OVERLAY=/path/in/workstation/client-workspace/install
bash deployment/airsim/demos/weststreet/run_demo.sh --output /evidence/tool-demo
```

该命令假定四方部署已就绪；旧 `run.py` 尚未提供这组Tools的完整启动编排（新联合入口见下节），
不能直接运行旧launcher就宣称启动了新demo。场景fixture独立读取
`events.json` 的 `airborne_ready` 再启动人物；任务不导入仿真RPC。
保存 `targets.jsonl`、`events.json`、`result.json`。success只表示任务窗口与
清理流程完成，不代表视觉误差、避障、整圈路线或动力学指标通过。

当前缺口：

- Navigation yaw链：SUPER原生已有yaw多项式，但Adapter未转交；原生FSM
  忽略距自身<0.1m的目标，需提供明确的原地朝向执行路径，不能只开配置。
- 朝向与跟随位置应由 Following 实现在同一运动会话内协调；Tracking 仅提供观测，原转向契约已移除。本 Demo 现有 Mission 编排不代表已提供通用 Following 入口。
- 四方联合启动、起飞后人物启停fixture、真实GPU视觉与SUPER移动闭环验收待完成。
  暂未发现需新增检测/测距Function或新增FollowPath Behavior的证据。

本轮9项任务策略/生命周期替身测试通过：正常持续更新、丢失目标停止后降落、
停止未确认不交接、过期/未来观测/坐标系不符/NaN拒绝等。
运行 `python3 -m unittest discover -s deployment/airsim/demos/weststreet/tests -v`。
这组测试不是ROS飞行验收；System的7项真实ROS替身测试见其VALIDATION。

客户端构建结果：7包构建完成，实际 `DroneInterfaceBase` 加载 System、Navigation、
Tracking 三个模块成功，未发送运动请求。AS2客户端依赖的Pydantic从已有锁定wheel
缓存按哈希校验后解压到独立 `python_deps`，不更换共享镜像。
证据为 `log/weststreet-demo/client-build.log`、`mission-tests.log`、
`system-build.log` 和 `system-ros-tests.log`。

### 真实四方 Tool 联合入口

`run_tool_demo.py` 是宿主部署/场景 fixture，`demo.py` 仍只调用公开 Tools。
先按 [运行配置](README-before-plaza.md#启动) 指定基础场景与已安装工作区，
然后运行以下命令。

```bash
python3 deployment/airsim/demos/weststreet/run_tool_demo.py \
  --system-only --output log/weststreet-demo/system-new
python3 deployment/airsim/demos/weststreet/run_tool_demo.py \
  --duration 5 --output log/weststreet-demo/follow-new
```

当前入口按已准备的本地场景/overlay运行；使用前核对脚本本地路径与现有依赖。
System-only不启动模型。联合模式在算力侧分别启动AS2、System/视觉/Tracking与SUPER；
宿主fixture等target_acquired后才启动人物，任务无RPC。地图注册等待未解锁状态稳定，
虚拟地面按ROGMap实际膨胀语义配置，不改变障碍膨胀半径。失败轮保持证据；物理
上锁未确认时保留仿真与算力恢复，不能直接结束模拟器。

真实起降 `log/weststreet-demo/live-system-04/` 已通过（最大爬升2.9605m、
起降Tool结果均成功、实际PX4上锁、资源已清理）。移动视觉追踪仍在验收，不能
以此起降结果替代。结束录包后用 `evaluate_tool_demo.py <运行目录>` 离线审计，
在具备ROS消息包的既有容器里执行，不读取仍在写入的sqlite库。

2026-11-16真实运行结果见 [Tool Demo 验证](VALIDATION-tool-demo.md)：已真实起飞、
GPU检测并启动人物，SUPER/FollowTimedPath实际消费；移动追踪失效后安全降落。
最新RGB-only配置改善了输入间隔，但跑姿导致尺寸估距跳变并被原EKF关联拒绝。
**5秒追踪未通过，不能据此宣称人物追逐或整圈通过。**

### 水平云台与单路采集

联合入口现使用原生 `lock-roll=true, lock-pitch=true, lock-yaw=false` 云台，
相机水平、yaw随机头；真实安装角归零，保留原相机偏移。实际只开启一条FrontCamera
RGB，320×240/FOV90、原生capture-interval=1/15s；Chase、depth及其他capture关闭。
显示器/Global观察相机不是传感器capture，不计入这条采集链。

传感器桥从原生actual_pose姿态历史计算模拟云台相对机体的变换，匹配原生零长度
SpringArm连同相机杆臂一起稳定的行为；该姿态只用于传感器安装TF，不注入飞控或
替代导航定位。按RGB采样stamp插值，超过100ms姿态间隔或250ms等待则丢帧，
不使用最新姿态或静态TF伪装。暂只支持该双轴锁定模式。

`log/weststreet-demo/live-gimbal-01/gimbal-audit.json`：实际唯一采集约13.77Hz，
345个动态TF、无静态camera TF，机体倾斜37.5deg；用独立PX4/AS2机体姿态合成的
水平残差最大2.34deg，不是原子渲染姿态精度证明。云台/采集配置已实际运行，
但本轮仍因YOLO后续漏检而停止，未通过5秒追踪。System安全降落并上锁。
离线审计入口 `audit_gimbal.py <运行目录>`；只分析停止录制后的bag。

### 视觉目标驱动云台指向

联合入口对FrontCamera启用平台绑定`aerohub-pointing`：Tracking公开TargetState流
→ 平台CameraPointing → 已部署原生桥RawRequest/SetPose。任务demo仍只调用Tools，
不使用Actor真值/LookAtObject，也不与Navigation争用无人机运动会话。

云台稳定roll，yaw/pitch按视觉目标三维位置指向人体中心；因此“对人”模式可有
有意的俯角，不再要求光轴始终世界水平。安装SpringArm仍抵消机体roll/pitch，
仅相机转动；不自动旋转机头。5Hz上限、2deg死区、pitch[-35,20]/yaw±100deg，
每次角度增量有界；输入无效或过期时停止提交新转动，不跟踪场景Actor。

原生SetPose确认前后以GetSimTime夹取姿态切换区间；图像stamp落入区间或姿态
结果不确定时拒绝转发，区间后才使用确认的新外参。RPC失败/时钟回退使该指向链
关闭图像输出并要求重启，避免旧TF配新画面。原生API在平台桥内部，非Mission旁路。

`live-pointing-01`已确认7次转动，320×240远处人物约14px高，连续漏检退出。
`live-pointing-02`保留单路/15Hz、恢复640×480，实际6次转动无RPC错误；最高置信
人物框中心偏差中位数为宽度0.25%、高度3.53%。实际更新窗口水平移动9.76m、
182条执行参考，但仍因关联失效未完成5s，物理速度峰值9.40m/s；不能称满足
速度约束或追踪验收。两轮均System降落上锁并清理，详情见原始result/evaluation。
当前默认图像规格为640×480单路RGB、原生15Hz，深度/其他camera capture仍关闭。

`audit_pointing.py`在停止录制后审计RPC确认和检测居中，`pointing-regression.log`
保存原Tracking29项回归；平台桥内`test_camera_pointing_ros.py`验证几何与切换隔离。
读者应将API接通、框居中、测距质量、连续跟踪、速度/避障与完整路线分别判定。

### 测距跳变的平均/过滤

WestStreet显式启用Tracking前处理`position_filter:=robust_linear_v1`，通用默认none。
最多5帧/0.8s，稳健局部速度将观测对齐到当前采样时刻后做残差限幅平均；3m孤立
跳点按无观测处理。无检测不产生新位置，多候选不混合；>0.5s间隔清空历史，
会话/坐标保护不变。因真实有效观测有时仅2–5Hz，初版0.4s窗口始终预热，已保留
该失败记录并增加稀疏采样回归后修正。历史窗口不是目标有效期，失跟仍0.5s。

真实`live-filter-02`录包确认14次处理，其中8次平滑、4次预热、2次多候选旁路。
平滑修正中位0.178m、最大0.885m，是相对原测量的修正量，非真值精度提升。
窗口内实际位移1.399m、98条轨迹参考，峰值3.942m/s；仍因36.960→37.479秒
的0.519s有效观测间隔失跟，未通过5秒任务。System降落上锁、资源清理通过。
原始位置与过滤值在`filter-observations.json`及ROS诊断话题中，采样stamp不改写。
`audit_filter.py <运行目录>`离线核查实际消费，不读取正在写入的sqlite录包。

回归36项通过（含6项滤波与稀疏采样、11项Core、5+4项真实ROS、10项facing），
不等价于连续追踪通过。下一步需解决有效观测间隔，不用增加平均强度伪造连续性。

### 双阶段最新buffer

联合入口启用检测stream和Tracking stream：唯一相机回调→容量1最新图像→原单worker
→容量1最新三维结果→Tracking；快照接口仍保留但不在任务高频链上。慢消费者覆盖
旧待处理值，不阻塞采集、不积压追旧帧；保留原始stamp/失效保护及世代隔离。

`live-buffer-01`已真实运行但未通过5秒追踪：推理前输入stamp出现0.666s间隔，
单次推理约60–80ms，System安全降落上锁并清理。后续需区分原生采集、ROS传输与
云台切换隔离，不通过扩容buffer/补旧值或增大失效阈值改判。2项采集callback、
3项真实DDS流消费及9项原快照回归通过；初次快照测试环境缺模块路径的失败已保留。

分段耗时与首次失跟逐帧去向见 [TIMING.md](TIMING.md)。该轮仅加打点，未改保护门。

### 图像 DDS 接收缓冲：已验证的部署前提

640×480 RGB真实对照发现，宿主约208 KiB接收上限会截断DDS请求的4 MiB缓冲，
实际socket仅416 KiB，导致桥与检测器两段UDP接收溢出。只提高上限后，两段传输
缺帧归零，检测入口由4.25恢复到12.64 Hz；该轮5秒任务与独立录包阶段门通过。
这不是推理慢或采集线程降到1.5 Hz。完整环线及可靠性矩阵仍未验收。

部署前按[公共DDS配置说明](../../config/dds/README.md#图像传输的宿主接收缓冲上限)
检查宿主上限、实际SO_RCVBUF和逐socket drop；配置值不等于实际值。
诊断临时修改已经恢复。联合入口在启动图像链之前检查宿主`rmem_max>=4194304`，
不足时明确拒绝启动，并在`predeclared.json`记录实际值；不会自动调整宿主上限。


### 人物平滑启动

AerohubSceneControl现在默认在1秒内把原路线播放速率从0平滑升到1，达到5m/s。
仍调用同一个`start_actor()`，不需要任务脚本逐帧控制人物。重复start不重置加速；
stop/reset立即暂停，resume重新加速，路线循环时不重复加速。地图资产无需重烘焙。
这是场景运动基线调整，不改变Tracking关联或失跟门；原瞬时启动可通过场景控制
配置`startup_ramp_seconds=0`保留为压力用例。详细配置及实测见
[AerohubSceneControl](../../../../simulator/projectairsim/unreal/AerohubSceneControl/README.md#平滑启动实现与验证)。

平滑启动后的独立20秒复测尚未通过；最新轮实际跟随15.62m、有效跟踪约5.20秒，
随后测距/过滤造成有效观测中断并按原门降落。多候选帧现在旁路关联但不清空仍在
窗口内的单候选平滑历史，避免短暂额外候选引起重新预热和反向趋势；候选不加入
平均，过期规则不变。40项回归通过；详见[验证记录](VALIDATION-tool-demo.md)。

### 当前策略：可控测距误差降权更新

当前联合入口显式选择`association_policy:=soft_innovation_v1`与`position_filter:=none`，
替代上面的历史预过滤实验。创新残差降权、单帧位置/速度修正限幅由同一个EKF完成，
避免前置趋势把连续测量挡在更新之外。配置与边界见
[Tracking软更新说明](../../../../src/tool/target_tracking/README.md#有噪声观测的软更新)。
原始时间戳、无效几何、歧义候选和真实0.5秒缺测保护保留。

43项回归通过，包括两段真实失败观测回放。首轮独立20秒实测被Navigation执行更新
拒绝提前中断，未完成追踪验收；不把回放通过当作20秒飞行通过。详情见验证记录。

最新`live-nav-gap-fixed-20s-01`已通过无额外启动延迟的20秒阶段任务：实际跟随
90.26m、997条执行参考，4次真实软更新，正常停止并降落解除武装。原Navigation拒绝
已定位为旧final与新计划执行时间不重叠仍发modify；已补停止确认后重新接管。
该轮未自然触发覆盖空洞，分支验证以原失败时间回归为准；不宣称完整环线/可靠性。
诊断选项`--runner-start-delay <秒>`仅用于放大静止终点交接窗口，默认0，记录在manifest；
使用额外延迟的轮次不能冒充同等时长的移动目标验证。

Adapter已补齐旧执行成功完成后的自动路由：外层任务有效时，新目标无需调用者
重新发起Navigation Goal，内部确认停止后提交新执行Action；未变目标只续期。
最新`live-completed-route-20s-01`原20秒任务通过，实际跟随90.14m、991条执行参考，
正常降落解除武装。路由协议及验证边界见[验证记录](VALIDATION-tool-demo.md)。

### 第一视角与保留窗口

`run_tool_demo.py --keep-scene --viewer-wait 20 --observe-runner` 在任务前登记
现有 FrontCamera RGB 组件为 HUD 观察视角。点击游戏窗口后，Tab 选择无人机
第一视角，G 切换全局；不需要再启用 Chase、Depth 或额外 streaming/capture。
`SetView` 返回布尔确认，实际视角状态通过 `GetDisplayState` 查询；
`viewer-ready.json` 记录 `camera=FrontCamera`、`view_available=true`。

`--keep-scene` 仅在物理解除武装确认后保留场景、PX4、传感器桥接和 HUD，
停止任务、导航、Tools 和算法进程。`retained-runtime.json` 记录所属资源；
此模式的评估应报告“执行服务已清理，观察场景保留”，不声称全部服务停止。
`--observe-runner` 记录独立真值供事后评分，不反馈任务或 Tracking。
首个路口约在人物启动后21秒，20秒直线测试不能证明转弯追踪。

### 机头绑定水平相机（待实飞观察）

当前入口起飞2m，相机原生FRD安装位置`[0.6,0,0.8]`：机体下方0.8m，
名义相机高度约1.2m。保留roll/pitch稳定、yaw跟随机体，关闭
`aerohub-pointing`，不再独立SetPose追人。仅此配置调整不包含机头追人
控制，因此原固定机头任务过弯后可能出视野，不作为转弯跟踪修复。

### YOLO同帧调试小窗

使用ProjectAirSim SDK Python环境，依赖官方ImageDisplay及其OpenCV显示环境：

```bash
python deployment/airsim/demos/weststreet/yolo_viewer.py <run_dir>
python deployment/airsim/demos/weststreet/yolo_viewer.py <run_dir> --replay --start 30
```

复用仓库ImageWindows与官方ImageDisplay(with_bounding_box=True)，仅显示
实际推理输入及同帧YOLO框。适配器按stamp_ns匹配记录，将xyxy转换为官方
bbox2d的center/size，并转换RGB/BGR与行stride。annotations仅来自模型预测，
不是仿真真值。不新增采集、推理或原生topic订阅，不修改上游。

窗口名称区分LIVE/REPLAY；LIVE为最新已完成结果，不是当前原生视口。
当前显示适配层绘制1像素细框、类别、置信度、框高、采样时间及推理往返耗时，
官方ImageDisplay仅负责窗口显示，不重复画框；仍无过期提示。无新结果或回放结束保留最后
一帧，不能据此判断当前仍检测成功。Esc关闭。

`--target-yaw`现同时开启部署与Demo调用：任务通过System Tool获得自身位置，
向Navigation.to_pose下达位置与纯yaw朝向。默认关闭，旧任务调用不变。
新代码真实35秒预算已执行但在首弯后失跟，尚未通过转弯验收；问题是
原生末端yaw约束的参考响应过慢，详见VALIDATION-tool-demo.md。

### 连续yaw实验入口

`run_tool_demo.py --continuous-yaw`同时开启Adapter和任务的CONTINUOUS_YAW，
与`--target-yaw`互斥。位置仍用SUPER；FollowTimedPath在同一输出中
限速更新朝向，不另启转向Behavior。接口变化后需重建导航执行overlay及
工作站接口overlay。原默认关闭，不自动改变历史场景。
首轮真实任务仍失跟，不能标为转弯修复；详见VALIDATION-tool-demo.md。

### 小窗与任务生命周期

使用`run_tool_demo.py --yolo-window`由fixture持有显示进程；正常结束、
异常和中断均关闭该窗口，即使`--keep-scene`也不保留YOLO窗口。
显示进程处理SIGINT/SIGTERM并关闭官方显示子进程；另带5秒无新结果
自动退出。独立调试时可用`yolo_viewer.py --exit-on-idle 5`，回放仍可
手动关闭，不自动把回放结束当成实时过期。实际idle/SIGTERM测试均无子进程残留。

### 有界短时恢复实验

`--recovery-window 1`显式把Tracking预测有效期从默认0.5秒延至1秒，
默认配置不变。目标原观测stamp不重盖，输入capture新鲜度仍0.5秒，
软关联距离门和单帧速度修正限幅仍使用原0.5秒模型预算。
任务观察年龄大于0.3秒时标recovery，冻结最后位置目标但继续更新朝向；
恢复新测量后恢复位置更新。冻结目标不代表物理速度立刻为零，也不保证
不加速，减速和连续性仍归Planner/Behavior。超过声明有效期仍失败收口。
与UE温和转弯路线分别验收，不能混用两个变量宣称算法单独改善。

### 独立圆弧降速路线

`--ue-map /Game/Aerohub/WestStreetSmoothTurn`显式选择新地图，原
WestStreetCircuit可显式选择；新动作序列WestSmoothTurnMotion，全长约
110.8秒。主街角采用4m圆弧、弯中2.75m/s、直路5m/s、加减速1.25m/s²，
原障碍段保留。生成与检查脚本在simulator/projectairsim/weststreet/，
既有资产拒绝覆盖；地图与sequence是外部UE资产，不随Git分发。

实际首弯半径约3.999m、速度2.750m/s已通过观察记录验证，原地图/序列
哈希未变。但单独采用新路线、原0.5秒窗口的35秒任务仍未通过：无人机
先撞路灯，随后失跟。人物胶囊通行不代表无人机净空；目前不能宣称两项
改动已完成联合验收。详细结果见VALIDATION-tool-demo.md。


实验参数 `--predictive-observation` 默认关闭：7候选点、1.5秒预测的
简化模型，经既有Navigation执行，不提供地图可见性或轨迹可行性保证。
65秒真实测试约50秒在第二弯裁框失跟，未通过；不能作为持续循环配置。
详见 VALIDATION-tool-demo.md。


### 当前追踪场景基线恢复

按用户要求，`run_tool_demo.py`默认地图恢复为
`/Game/Aerohub/WestStreetShiftRight2mLate4m`：第二弯半径4m、弯道2.75m/s、
直线5m/s，首弯右移2m与延后4m保留。现有UE资产可直接使用，无需
重生成/重烘焙。10m/2m/s旧地图仅保留外部历史资产，不再提供生成或启动入口。
候选观察点策略默认关闭。此操作只恢复下一轮运行入口，没有启动新飞行，
不改变旧失败结果。

下一步跟随策略使用Tracking已有TargetState.position/velocity及其header时间、
observation_stamp和valid_until；速度大小由水平velocity求模，方向由向量归一化，
低速方向不可靠。现有EKF已输出p/v，不应在Demo重复差分建立第二套估计器；
其动态精度与响应仍需评估。暂不新增a估计，但执行层速度/加速度约束仍保留。


p/v实验入口：`--tangent-follow --tangent-lead 1 --continuous-yaw
--recovery-window 1 --ue-map /Game/Aerohub/WestStreetShiftRight2mLate4m`。
前看默认仍0.35秒，1秒为显式候选；两种实验跟随模式不能同时开启。
YOLO保持0.35、无ROI。该候选一次65秒任务通过，但240秒任务在约
78.8秒因广场柱墩遮挡失效，两圈尚未通过。位置前看不是速度前馈，
距离允许浮动，不能声称严格保持8米。详见VALIDATION-tool-demo.md。
