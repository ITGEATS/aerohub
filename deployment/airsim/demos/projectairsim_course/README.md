# ProjectAirSim 赛道单轮测试

使用新 HUD 赛道包、原版 flight_board PX4 和现有 AS2 执行层。
PX4、磁场和传感器配置仍来自 `log/projectairsim/package-course-wmm/profile.json`。
每次执行命令，停止上一轮已上锁的飞控，重启场景和 SITL，从起点飞一轮。
不在飞行中热换算法。启动器使用同一个文件锁，避免两个终端同时重置同一场景。

## 本机快速运行

在仓库根目录执行。YOPO 使用原生速度缩放，目标速度 5 m/s，不施加额外 2 m/s² 约束：

```bash
python3 deployment/airsim/demos/projectairsim_course/run_yopo.py
# 降速单轮，模型与执行端同步设定
python3 deployment/airsim/demos/projectairsim_course/run_yopo.py --speed 3
```

SUPER 保留 5 m/s、2 m/s² 配置：

```bash
python3 deployment/airsim/demos/projectairsim_course/run_super.py
```

历史本机入口 `log/projectairsim/manual-wmm5/run.py` 不随仓库分发；新部署使用上述正式入口。
每次命令只启动一轮，不自动重试。飞行中 Ctrl+C 请求结束任务并等待降落恢复。
若飞控仍解锁，入口拒绝重置。画面由运行者观察，录屏使用已有桌面录制工具。
启动、起飞、航线、停止、降落都需时间；端口释放时会明确打印等待信息。

YOPO、DiffPhys 只检查启动和输入、不解锁时，加 `--prepare-only`。这不是飞行成功证明。

## 共用与算法专属部分

| 项目 | 共用约定 |
| --- | --- |
| 场景 | 同一 120 m、24 根柱子的 Unreal 赛道；独立包提供几何与 HUD |
| 初始位姿 | 场景 NED [151.5, 0, -1.2896818542480468]，yaw 0 |
| 状态权威 | PX4 / AS2；真值不作为 Planner 控制输入 |
| 任务原点 | 每轮在地面测量；目标相对该原点 ENU [0, 120, 2]；DiffPhys 为 [0, 120, 3] |
| 起降 | 原生 Takeoff/Land Behavior，经 AS2 → PX4；不调用模拟器运动 API |
| 记录 | `log/projectairsim/` 下每轮独立时间戳目录，保留失败结果 |
| 本机通信 | ROS domain 188；原生 topic/service 9189/9190；HUD 9011 |
| PX4 | 现有 `aerohub-course-wmm-px4`，system 21 / instance 20，不修改固件或镜像 |

| 算法 | 平台输入 | 执行链 | 本赛道状态 |
| --- | --- | --- | --- |
| SUPER | LiDAR、PX4 odom、ROG 地图 | Adapter → FollowTimedPath | 已有真实飞行；5 m/s 一轮在约110.4 m后轨迹过期，未完成 |
| YOPO | 前视轴向深度、CameraInfo、PX4/AS2 pose/twist、真实相机TF | CUDA Inference → YOPO Planner → FollowTimedPath | front_center模板、标准Inference入口下，3m/s单轮到点、停车、Land/disarmed通过；非恒高验收 |
| IPC | 点云、odom、控制会话 | 既有 FollowReference external_acro → 角速度/推力 | 已接入；名义3m/s单轮到点、Land/disarmed通过，碰撞事件与完整控制映射验收未覆盖 |
| DiffPhys | 专属前视深度、状态、循环策略会话 | CUDA Inference → FollowReference attitude → 姿态/推力 | 标准Inference入口已实飞；5m/s、3m起飞后控制量超限退出，Land/disarmed通过；导航未通过 |

切换算法复用场景、SITL、任务原点、起降和证据格式；传感器、模型、Planner、执行接口和停止判据按算法契约保留。
YOPO 的原生到点定义是 `ARRIVED_5M` 后物理停止，不等同 SUPER 的 1.5 m 到点门槛。
实际速度分布与峰值须看记录；设定 5 m/s 不代表整段恒速或实测绝不超出 5。

## YOPO 输入与部署

`prepare_yopo_scene.py` 从本机 WMM 包读取机器人配置，并从匹配 ProjectAirSim 源码的
`robot_quadrotor_classic.jsonc` 复制原生 `front_center` 模板，生成独立 scene/robot/profile 到
`log/projectairsim/yopo-course-config/`；保留磁场覆盖，不改原归档。
相机为 `yopo_front`，保留上游机体前方0.46 m、pitch 0安装姿态；按YOPO输入契约修改为320×185、HFOV 90°，只启用浮点 DepthPlanar。
关闭本任务不消费的下视相机；保留 LiDAR 和飞控必需传感器。

标准输入为：

- `/drone0/sensor_measurements/yopo_front/depth_planar/image`
- `/drone0/sensor_measurements/yopo_front/depth_planar/camera_info`
- `/drone0/self_localization/pose`、`/drone0/self_localization/twist`
- `/tf`、`/tf_static`，光学系 `drone0/yopo_front_optical`

保留原图 stamp、轴向米制深度和真实光学 TF，不能用 DepthPerspective 改名代替。
Planner 检查相机几何、采样同步、新鲜度、推理服务和 Behavior 就绪，再允许任务开始。
YOPO worker 的 `YOPO_VELOCITY=5` 与 Planner 的 `max_speed: 5.0` 同时设置。
不修改训练权重，不将原生加速度缩放称作独立 2 m/s² 限制。

模型依赖按仓库标准 Inference 入口准备：初始化锁定的 `src/tool/navigation/planner/yopo/upstream`，
通过 `deployment/inference/setup_models.py` 或模型 `setup.sh` 准备本模型 `.venv` 和
`artifacts`。赛道入口复用 `deployment/inference/start_yopo.sh`，只设置本轮服务端口和
速度；不再接受赛道专有的源码、解释器或模型目录覆盖。模型启动校验标准源码文件哈希。
worker 使用 CUDA、18081 端口，每轮独立进程；按 PID 与启动时刻确认归属后回收。

ROS 包在固定 `uav/apps-super-forest:onboard-v0.1.1` 镜像中补编译，不重建共享镜像：

```bash
docker run --rm --network host -v "$PWD:/repo" --entrypoint bash \
  uav/apps-super-forest:onboard-v0.1.1 -lc \
  'source /opt/ros/humble/setup.bash && source /root/aerostack2_ws/install/setup.bash && source /opt/uav_execution_ws/install/setup.bash && bash /repo/deployment/airsim/demos/yopo_single/build.sh'
```

运行明确加载本工作区的 `log/yopo-single/ros_ws/install` 和固定镜像的 AS2/FollowTimedPath。
`yopo_compute.py` 监管 AS2、Inference ROS Adapter 和 Planner；任务仅通过 ROS 调用。
本轮是 PC 共置测试，不表示 RoboPi 四方部署验收。

## 观察与结果

每轮打印证据目录。SUPER 保存 `flight.jsonl`、地图 bag 和 `result.json`；
YOPO 保存 `worker.log`、worker metadata、`mission.jsonl`、`plans.jsonl`、
起飞后的原始深度快照及 `result.json`，最终 PX4 状态单独保存。
任务成功、到点、停车和上锁分别看结果字段，不能仅看进程退出。
HUD 的 FINISHED 表示计时结束，失败也可能结束计时；不是到达终点标志。
目前航线计时从提交导航目标开始，不包含起飞阶段。

原生碰撞事件尚未桥接：本入口明确 `collision_coverage: unverified`。
运行者看到的画面可用于判断本轮现象，但缺少原生碰撞事件时不宣称自动无碰撞验收通过。
磁场修正已通过地面航向检查；各算法飞行结论独立记录。

HUD 优先使用配置端口（赛道默认9011）；每次启动以不启用 `SO_REUSEADDR` 的 bind
检查端口。有旧监听或 TIME_WAIT 时立即选择系统分配的空闲端口，不等待旧 HTTP 连接回收，
也不终止未知占用进程。实际端口写入 `runtime/hud-profile.json`、`runtime/display.json`
和运行状态，并同步传给遥测、SUPER 与 YOPO/IPC/DiffPhys 任务容器；启动时打印该端口。
固定仿真通信端口仍等待释放，最长90秒，超时按具体端口检查归属。
原运行命令无需增加端口参数。
2026-11-16 已实测9011仍被占用时自动选用33609，打包场景启动至HUD HTTP就绪约3.0秒，
全程未起飞；结果保存在 `log/projectairsim/hud-port-verification.json`。

旧入口及算法契约见 [YOPO](../yopo_single/README.md)、[IPC](../ipc_navigation/README.md)、
[DiffPhys](../diffphys_forest/README.md)。这些旧入口仍针对 AirSimNH，不能直接用于本包。

## IPC 名义3m/s赛道运行

```bash
python3 deployment/airsim/demos/projectairsim_course/run_ipc.py
```

每次从起点重启赛道和原版PX4。保持Chase主视角，IPC消费LiDAR而非前视相机。
`--prepare-only`只运行未解锁输入/模式与影子会话探针；其结果不代表空中求解通过。
默认影子探针在地面可能因起点占据拒绝路径，须保留结果。

沿用既有IPC平衡配置：名义巡航3、参考3.6、参考加速1m/s²、MPC逐轴速度4、
加速度3、jerk10、固定yaw、自由XYZ；目标相对本轮地面ENU [0,120,2]。
无独立高度越界终止，仍保留横向[-6,6]和前后[-2,123]m边界、0.6rad倾角终止。
点云原始输入`/drone0/sensor_measurements/lidar1/points`，NavigationInputs真实变换到earth；
IPC里程计及会话目标也为earth。任务中的odom目标显式经TF变换，不改frame冒充变换。
接管前读取PX4实际vehicle_attitude_setpoint悬停推力，不使用YOPO或旧飞行的油门。
原生Takeoff → 现有FollowReference external_acro → AS2角速度/推力链 → PX4；
原生Land并独立导出最终vehicle_status。

首次准备（复用固定 `uav/apps-super-forest:repo-native-20261109`，不重建镜像）：

```bash
mkdir -p log/ipc/native-deps
git clone --branch v0.6.2 --recurse-submodules https://github.com/osqp/osqp.git log/ipc/native-deps/osqp
git clone --branch v0.6.3 https://github.com/robotology/osqp-eigen.git log/ipc/native-deps/osqp-eigen
bash deployment/airsim/demos/ipc_control/build_overlay.sh
bash deployment/airsim/demos/ipc_navigation/build_core.sh
bash deployment/airsim/demos/ipc_navigation/build_node.sh
```

既有脚本校验完整锁定提交，overlay同时重编译匹配的controller/platform/FollowReference。
构建日志在log/ipc/；核芯5项数值测试、ROS构建5项测试和实际平台控制帧配对测试通过。
运行只读挂载本工作区overlay及native-deps，不混入其他工作区的插件二进制。

## DiffPhys 目标5m/s赛道入口

```bash
python3 deployment/airsim/demos/projectairsim_course/run_diffphys.py
```

运行前构建：`bash deployment/airsim/demos/diffphys_forest/build.sh`。
复用固定repo-native-20261109镜像与外部`log/diffphys/attitude-overlay`，该overlay包含
姿态命令源stamp/配对补丁、FollowReference及DiffPhys ROS接口，不混用IPC旧二进制。
模型依赖使用标准 `src/basic_function/inference/models/diffphys/{.venv,artifacts}` 与锁定算法子模块，
赛道入口复用 `deployment/inference/start_diffphys.sh`，显式选择 high_speed、CUDA和18082端口。
不再通过 DIFFPHYS_MODEL_DIR 改变模型或算法源码定位；两种profile均保留来源/权重哈希校验。

模型为官方high_speed，目标5m/s、10m目标前视、指令倾角锥0.5rad；真实姿态超过0.6rad
记失败。深度相机为640x480、HFOV91°、pitch20°、机体FRD [0.5,0,-0.1]，15Hz。
原生camera→标准ROS轴向米深度→82%中心裁剪/nearest48x36→CUDA逆深度/pool3，
保留原始标定/TF和采样stamp；主窗口独立Chase。该视觉任务关闭无消费者LiDAR。
ProjectAirSim标准深度发布为BEST_EFFORT，Planner显式`camera_reliable:=false`匹配；
CameraInfo采用RELIABLE/TRANSIENT_LOCAL匹配一次性标定，不能直接套旧相机的QoS。

原生Takeoff2m后（2026-11-16 调整；目标高度仍为地面相对3m，当前配置尚未实飞验收），实测本轮PX4悬停油门；策略通过既有FollowReference ATTITUDE执行。
进入目标6m范围后取消策略，确认Action结束及POSITION模式，再原生GoTo2.5m/s收敛到
地面相对ENU [0,120,3]，误差≤1.5m、速度≤0.3m/s驻留2仿真秒，最后Land/disarmed。
目标速度是策略输入，不表示严格实测上限；独立记录峰值和实际分布。
横向/前后赛道边界保留，高度不设独立任务终止阈值。原生碰撞事件仍未桥接。

`--prepare-only`不解锁：真实深度/状态/几何、CUDA模型服务ready检查；悬停油门仅使用
明确标注的地面占位数，不发送飞行控制，不视作实际悬停标定或真实飞行通过。
已执行：实际平台ATTITUDE配对测试、5项解码对照、真实CUDA官方预处理及30帧循环
推理对照通过；飞行结果须查看各轮result.json，不从其他森林的历史结果继承通过。

DiffPhys 大图传输使用随入口提供的 `diffphys-dds.xml`：桥、标准接口、算法与任务均
显式加载 32 MiB Fast DDS SHM transport。默认传输在本主机出现大图秒级缺口；同输入
15 秒对照中，该配置将原始/标准图最大采样间隔恢复至 72 ms。不要靠增大控制超时
掩盖传输丢帧；只允许原生 Pair0 话题的一个消费者，诊断探针须与桥顺序运行。

## 最新验证与剩余工作

2026-11-14，标准源码/模型定位及统一启动脚本已完成CUDA数值对照、真实相机记录HTTP重放及两算法实飞：

| 算法/配置 | 实飞结果 | 证据目录（log/projectairsim/） |
| --- | --- | --- |
| YOPO 3m/s、起飞2m | 到达5m门、停车、Land、上锁通过；速度中位2.981、峰值3.268m/s；后半程升高，不代表恒高 | yopo-course-20261114-190858-790815 |
| DiffPhys 5m/s、起飞3m | 策略控制量被拒后watchdog退出；Land、上锁通过；未到终点 | diffphys-course-20261114-191143 |

输入与接管高度的离线实验发现见 [DiffPhys 输入对照记录](diffphys-input-findings.md)；该轮实验产物已按要求清理。
DiffPhys仍需解决掉高与控制量超限。原生碰撞事件及完整故障矩阵未验收，不宣称完整复现成功。
准备与运行均使用仓库锁定子模块、标准模型目录和现有Inference启动脚本；已移除赛道专属源码/模型路径配置。
历史失败、配置变迁和详细结果见 [验证记录](validation.md)。

## 新 HUD 重跑（2026-11-16）

四套 `run_*.py` 共用的启动层默认选择
`log/projectairsim/hud-v1/package/Linux/Blocks/Binaries/Linux/Blocks`。
可通过 `COURSE_BINARY` 指定另一个已构建且包含相同赛道与新 HUD 的可执行文件。
启动时在 runtime 下生成 `hud-profile.json`，只替换二进制路径，不覆盖原 WMM profile，
也不更改算法速度、相机输入或执行门限。这里只替换已有包，不自动构建或烘焙。

Manager 的遥测进程依据实际展开场景登记 streaming 视角，优先选择 Drone1 Chase RGB；
Tab/G 可切换车辆视角和动态 Global。登记与遥测只使用原生 service 连接，
不启动占用 topic 连接的独立 display_session，不与传感器 bridge 争用 Pair0。

四套任务通过 SDK-free 扩展 RPC 对显式 `Drone1` 调用 ControlRun。
提交导航目标时进入 RUNNING 并启动计时；结束任务时进入 FINISHED 并冻结计时，
与当前观察视角无关。左标题显示算法名，右标题显示“120 m 赛道”。
FINISHED 仍不代表到点或成功；最终结果以 result.json 为准。
SUPER 任务容器新增仓库只读挂载，以加载同一份公共扩展客户端。

本次完成 Python 语法、8 项既有 HUD 客户端/视角回归及部署配置替换检查；
尚未执行新包与 PX4/ROS 联合运行、实际画面或飞行验证。
录屏前先逐套执行 `--prepare-only` 检查，确认画面、输入和 PX4 就绪，再运行完整任务。
SUPER 的 prepare-only 仅检查场景与 PX4，不代表其 Planner 已就绪。

YOPO 就绪诊断：`mission.jsonl` 的 `ready_checks` 分别记录相机几何、观测、推理、
Action、modify 和 stop 接口。CameraInfo 使用 RELIABLE/TRANSIENT_LOCAL 接收桥的
持久化标定，避免 Planner 晚启动后只收到深度而收不到标定；相机平移按
`camera_x_m: 0.46` 校验。任务结束保留 `compute-console.log` 供核查。

## DiffPhys 失败会话阻塞重跑

`run_diffphys.py` 遇到仍运行的任务客户端时拒绝切换场景；任务退出但 PX4
仍解锁时保留算力执行链。这两种情况不能只凭容器存在就视为无用残留。
先查看 `log/projectairsim/diffphys-course-current.json` 对应目录中的
`result.json` 与 `px4-final-status.txt`。任务运行中用 Ctrl+C 请求恢复并等待退出。

如果上一轮已失败退出，决定放弃当前仿真状态、从起点重来，执行：

```bash
python3 deployment/airsim/demos/projectairsim_course/run_diffphys.py --reset-sim
python3 deployment/airsim/demos/projectairsim_course/run_diffphys.py
```

`--reset-sim` 统一调用本目录的 `reset_sim.py`，明确放弃共享赛道仿真，
不执行降落，也不把失败改成成功。范围和证据保留规则见
[放弃当前赛道仿真并重新运行](#放弃当前赛道仿真并重新运行)。
正常启动仍要求明确的 PX4 上锁反馈，空白或未知状态不会按已上锁处理。

2026-11-16 的 `diffphys-course-20261116-151202` 在倾角超过 0.6 rad 后失败，
恢复报 `condition timeout: {}`，最终仍解锁。因此其“已有任务运行”提示对应的是
保留的失败执行链，并非锁文件残留。重置只解决下一轮启动阻塞，不能解决策略倾角超限。

### 降落期间保留仿真窗口

HUD 的 `FINISHED` 仅结束导航计时，随后仍须完成原生 Land 和上锁。
DiffPhys 此时显示“降落中，请保持仿真窗口开启”，确认上锁后改为“已降落上锁”。
请以终端结果中的 `land_success: true` 和 `disarmed: true` 为关闭窗口依据。
PX4 lockstep 依赖场景持续提供仿真输入；提前关闭窗口会让落地判定、Action 结果和
上锁流程停在当时状态，保留的算力容器无法在没有场景的情况下继续完成降落。

2026-11-16 `diffphys-course-20261116-152608`：GoTo 已到点，Land 于
15:27:04（本地时间）接受并运行；15:27:13 场景日志记录窗口销毁、
`All Windows Closed`、`RequestExit(0)`。最终 PX4 为 `ground_contact: True`、
`maybe_landed: True`、`landed: False`，仍解锁。日志证明窗口退出先于恢复超时，
无法据此确定窗口关闭事件由谁或什么触发。退出期间的 HTTP ensure 在退出请求之后出现，
不能将它认定为退出原因。原始场景日志另存于该轮 `scene-exit.log`。

可单独验证起飞、降落和上锁，跳过 DiffPhys 策略与120m赛道：

```bash
python3 deployment/airsim/demos/projectairsim_course/run_diffphys.py --takeoff-land-only
```

该模式结果使用 `scope: takeoff_land_only`，不代表导航成功。
`diffphys-course-20261116-153241` 已在有屏 ProjectAirSim + PX4 SITL 上通过该测试：
`success: true`、`land_success: true`、`disarmed: true`。
这次短流程不替代终点处完整导航与降落验收。
Action 超时现在区分服务发现、目标接收、执行结果阶段，报告时钟是否推进及当前平台状态，
避免只留下 `condition timeout: {}`。

## 放弃当前赛道仿真并重新运行

四个入口均支持 `--reset-sim`，调用相同的 `reset_sim.py`。此选项只停止会话并退出，
不启动新飞行。无需先降落、上锁，也不要求任务客户端先退出；它在获取飞行锁之前执行。
例如放弃 YOPO 失败会话并改为3m/s重跑：

```bash
python3 deployment/airsim/demos/projectairsim_course/run_yopo.py --reset-sim
python3 deployment/airsim/demos/projectairsim_course/run_yopo.py --speed 3
```

重置范围为本地赛道共享 SITL、场景、按明确名称规则识别的四个 demo 任务及算力容器，
以及 PID 和启动时刻均匹配记录的推理进程。它可以结束其他算法的当前赛道运行，
用于明确放弃这一共享仿真实例；不操作真实飞控或未知容器。任务、算力容器并行停止，
短超时后由 Docker 结束进程，不等待原任务的降落恢复。原始失败结果不改写，
证据目录新增 `simulation-reset.json`。所有入口使用相同重置语义，不再保留 DiffPhys 专用重置实现。
正常 Ctrl+C 仍请求降落恢复；明确放弃仿真时直接用 `--reset-sim`。

## 整理后的验证边界

启动辅助与故障会话管理回归：

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s deployment/airsim/demos/projectairsim_course/tests -v
```

当前13项通过，覆盖Action等待阶段提示、启动状态检查、HUD端口选择和
仿真重置范围（Docker/Manager操作使用mock）。端口测试需要本机socket权限。
本次仓库整理未重跑飞行，也未验证YOPO标定适配的完整动力学闭环；
不能将辅助测试通过提升为上述算法赛道复现成功。
