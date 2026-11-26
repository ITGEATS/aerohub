# FlowPilot：闭环验收与证据核查

用于已完成 FlowPilot 适配后的分层验收。先读 [验收 Skill](../SKILL.md)、[平台测量 Skill](../../uav-platform-sensor-adaptation/SKILL.md)、[FlowPilot 契约](../../uav-platform-sensor-adaptation/references/flowpilot-depth.md)，并核对 [Demo 当前记录](../../../deployment/airsim/demos/flowpilot_single/validation.md)。本页不包含私人主机地址或任务目录；不启动任何现有服务器，也不代表当前部署已通过。

## 1. 固定本轮身份与拓扑

运行前记录主仓提交与 dirty diff、上游 HEAD/工作树、实际加载 Python 文件及散列、权重散列、镜像 ID/digest、ROS/AS2 overlay、GPU/驱动、参数和相机配置。源码被挂载不等于进程已重新导入；需要按任务状态安全重启后才可记为运行新版本。

按四方职责列出：AirSim 地图/渲染、PX4 SITL、算力侧 worker/ROS Adapter/Planner/AS2、工作站任务。角色可以共置，任务客户端仍通过 ROS 调用。记录 domain、namespace、端口和网络命名空间，worker endpoint 从真实调用方检查。真值只评分，不用 AirSim 运动 API 代替 FollowTimedPath。

地图名称、可执行文件、settings 与出生点一起记录。历史 Linux 轮次使用 AirSimNH，不应据此声称本地 FixedSpawn 或其他地图已通过。渲染方式、窗口大小、现场观看和录屏状态分别记录；后续修改算法/配置时，旧有屏轮次不能自动代表该新版本的完整验收。

## 2. 运行前矩阵

每项记录适用理由、实际命令/操作者、指标及其来源、阈值、次数、期限、证据路径和源码/配置身份。以下为本实例的核查表，不是已执行的结果。具体次数在本轮开始前确定。

| 项目 | 判据和必需证据 | 不足以通过的证据 |
| --- | --- | --- |
| 构建/加载 | 当前接口、Planner、Adapter、匹配 FollowTimedPath 和 AS2 均实际加载 | 仅编译、旧安装前缀或全 skip |
| 模型 | 四份权重校验、独立锁定环境、真实 CUDA 完整输出对照 | nvidia-smi、import、CPU mock |
| ROS 服务 | 独立客户端进程实际请求/响应，身份与时间保留，错误输入拒绝且可恢复 | 图发现、同进程直接调用 |
| 真实输入/下游 | 有效 AirSim DepthPlanar、正确 CameraInfo/TF、配对状态、GPU 请求和 Planner 实际消费 | 静态深度截图、fixture 或只看发布率 |
| 同驻预检 | 实际渲染/传感器/worker 同驻；空闲后首请求和后续均满足配置与提交余量 | 预热连续请求、历史更宽期限 |
| ground-only | 当前会话 disarmed、clock/定位发布权、就绪及消费者成立；无运动请求 | ground success=true 视作飞行通过 |
| 正常导航 | Takeoff、多个 revision 实际生效、实际跟踪、预声明到达与停止、恢复链 | accepted 数量或触达到达半径 |
| 停止与最终位置 | 同任务 STOPPING→STOPPED；速度保持与期限、停稳后目标距离分别满足 | 上轮 STOPPED、随后 Land 时的低速 |
| 目标高度 | 预声明飞行窗口内的位置高度与目标差、极值及保持时间 | 世界盒地板或地面坐标直接当巡航高度 |
| Land/disarm/退出 | 原生 Land 结果、地面/武装与新鲜 Offboard 状态分别记录 | 服务 ACK、重启后初态、肉眼落地 |
| 主动取消 | 有效规划后真实取消、ACK、CANCELED 停止与独立恢复 | 任务自然完成后调用 cancel |
| 输入中断/推理故障 | 实际注入及恢复时间、正确原因与停止期限、重试/忙占用语义 | 任意任务失败、空日志或重启后存活 |
| 有屏平台 | 当前地图、版本、负载下真实窗口与深度输入；观察方式可核查 | 只有离屏通过或另一版本截图 |
| 清理 | 按拥有关系回收；共享任务不受影响；最终容器/PID/端口/GPU 状态 | SSH 断开、启动器退出 |

仓库要求的项不能通过“不适用”绕过。跨物理设备、RoboPi、真机或其他场景按本轮声明判断；未纳入时明确未覆盖，不能外推已有单机结果。

**STOPPING 行语义。** 最终状态 fresh ≠ 停机瞬间 fresh。`publish_status.ready/ages` 在 stop 决策后同 tick 末尾重算；`STOPPED` 的 `input_ages` 全新鲜不代表停机时刻就新鲜。停机根因必须比对 mission.jsonl 首次 STOPPING 行的各 ages（depth / pose / twist），不能只看终态。

## 3. 正式入口与环境前置

### 准备与构建

模型准备及 GPU 对照使用 [输入契约中的命令](../../uav-platform-sensor-adaptation/references/flowpilot-depth.md#6-可复验入口和证据等级)。准备入口不启动常驻服务。

ROS Humble + 匹配 AS2/FollowTimedPath 环境中，从仓库根执行：

```bash
bash deployment/airsim/demos/flowpilot_single/build.sh
source log/flowpilot-single/ros_ws/install/setup.bash
```

`log/flowpilot-single/ros_ws` 是本次构建可重新生成的输出目录，不是必须预存的脚本依赖。设置 `FLOWPILOT_ROS_WORKSPACE` 时同步修改 source 路径。build.sh 不构建完整 AS2 执行层；缺少匹配安装必须先按正式部署/构建说明准备，不能混用旧消息或插件。

### 已有平台上的模型/Planner 服务

按 [模型 README](../../../src/basic_function/inference/models/flowpilot/README.md) 用模型 venv 启动 worker，按 [Demo README](../../../deployment/airsim/demos/flowpilot_single/README.md) 在 ROS 环境启动 Adapter 与 Planner。endpoint、namespace、use_sim_time、相机 remap 与实际部署一致；这组命令假设平台和桥已正确启动，不等于完整空机部署。

被动预检从实际工作站客户端环境运行：

```bash
python3 deployment/airsim/demos/flowpilot_single/preflight.py --params deployment/airsim/demos/flowpilot_single/planner.yaml --namespace /drone0 --idle-s 15 --samples 5 --output log/flowpilot-new-run/preflight.json
```

当前配置推理预算 0.1 s，观测年龄 0.3 s、skew 0.05 s、handoff lead 0.2 s、reserve 0.05 s；这是实例配置，不是通用硬件指标。预检还比较真实运行参数，包括数组类型。明确检查退出码 0 才允许下一步，飞行编排内部重做该硬门；手动运行 mission.py 本身不保证先执行预检。保持与实际起飞等待相当的空闲时长，少量样本不能保证长期实时性。

**预检时序与硬门。** 起飞入口内部必须检查预检退出码；shell 分号串联不能形成硬门。刚重启后首采样 `current_inputs_fresh` 可能为 false（inputs 尚未齐），次样本即 true；预检瞬时 false 不构成飞行动力学失败证据，不应阻止 mission 执行——但脚本顺序缺陷（exit 1 后仍执行 mission）本身须在流程检查时标记。

### ground 与 flight

从已加载消息、传感器 RPC 依赖和 ROS overlay 的工作站环境运行：

```bash
python3 deployment/airsim/demos/flowpilot_single/mission.py --output log/flowpilot-new-run/ground --ground-only --rpc-host ACTUAL_AIRSIM_HOST --rpc-port 45471
```

`ACTUAL_AIRSIM_HOST` 必须替换为本地部署配置中的实际可达地址。飞行前另选新的证据目录，以下示例在同一次命令中用 `&&` 检查预检退出码，非零时不启动飞行：

```bash
python3 deployment/airsim/demos/flowpilot_single/preflight.py --params deployment/airsim/demos/flowpilot_single/planner.yaml --namespace /drone0 --goal 0 12 2 --idle-s 15 --samples 5 --output log/flowpilot-new-run/flight-preflight.json &&
python3 deployment/airsim/demos/flowpilot_single/mission.py --output log/flowpilot-new-run/flight --goal 0 12 2 --timeout 90 --rpc-host ACTUAL_AIRSIM_HOST --rpc-port 45471
```

每次复跑换唯一运行标识；mission.py 会以写模式打开证据文件，不能重复使用旧目录。该 mission 当前固定 `/drone0` 和 PX4 system ID 71，不能仅改 namespace 或端口就推广多机/其他实例。RPC 端口也需与实际 settings 一致，示例的 45471 不是所有 Demo 的默认。

### 启动脚本支持范围

[run.py](../../../deployment/airsim/demos/flowpilot_single/run.py) 是本组件的原生 Linux 入口：以 `--config` 指向本机 `*.local.json`，提供 scene-start/start/check/ground/flight/cancel-flight/stop/reset-round 子命令，先检查端口占用并拒绝与旧任务并行，不接管或停止已有任务。flight 内部先执行正式 preflight，非零即拒飞；人工直接调用 mission.py 不提供该硬门。复位入口已实现但尚未在平台实测。

[runtime.sh](../../../deployment/airsim/demos/flowpilot_single/runtime.sh) 假设容器内 `/repo`、`/state`、`/evidence` 及多个 overlay 均已准备。Linux 共享服务器曾实际运行的任务编排/故障脚本若只留在私人任务目录或忽略日志中，仍是**正式交付缺口**。本页不要求读取这些目录才能执行已有正式检查，也不声称补齐其自动化。独立新主机复跑前，应先交付参数化的 Linux 启动、检查、故障注入、跨轮复位和停止入口；缺少时对应项保持未完成，不伪造命令。

## 4. 停止、最终位置与模式判读

当前生命周期源码用 `5 − arrival_brake_margin_m` 作为正常到达触发半径；当前 planner.yaml 余量为 1.25 m。触发原因仍名为 ARRIVED_5M，它不是最终距离保证。

当前停止判据同时要求停止请求已确认、无未完成执行、Pose/Twist 新鲜、HOVER/POSITION 模式新鲜，以及速度范数 <0.2 m/s 连续 1 s；停止超时为 10 s。按源码区分单调墙钟期限与仿真时间保持，保留暂停/回退测量。不要修改阈值来使已有结果通过。

从 mission.jsonl 同一 task_id 的首次 STOPPING 配对后续终态，排除另一任务或早先 STOPPED。低速段只使用停止阶段、原坐标语义明确且新鲜的 velocity；缺测和过大间隙不能插值凑够保持时间。日志采样不足以证明阈值时记为缺证据。

当前正常 mission 的 success 包含正确停止原因、stopped、land_success、disarmed、final_within_5m、offboard_released、collision=false、无任务 error 和 max_effective_revision≥2。离线核查时逐项复算，不只相信 success。最终距离还需核对原始 pose 与目标；result 会四舍五入到毫米，门限附近不能仅靠舍入值证明原始距离。高度是另外的指标，当前成功布尔值没有完整覆盖高度保持。

区分 AS2 退出服务是否支持、返回值、PX4 ACK 和平台是否实际退出。当前 mission 在 disarmed 后含工作站侧 PX4 POSCTL 兜底，它是该验证客户端的实现事实，不代表 AS2 退出服务已修复，也不是允许绕过 Behavior 做运动控制的通用方案。复制到其他部署前核对控制权和平台规则；缺少 ACK 或状态更新时间时保留证据缺口。

当前实现限制：`offboard_state_released()` 仅检查缓存 PlatformInfo 的 offboard 值，没有独立的该消息年龄门；result 中 disarmed 也来自缓存，`final_distance_m` 取末次缓存 Pose。不能仅由这些字段宣称状态新鲜度已经验收。复核时需提供事件之后持续接收的平台/定位日志及时间依据；无法证明时将对应项标记缺证据。任务的 ground-only 分支不解锁、不发送导航目标，但仍调用 AirSim 碰撞 RPC 并依赖真实平台就绪，不是脱离平台的纯单元测试。

## 5. 故障、渲染和跨轮恢复

- 取消：至少有两个有效 revision 后调用真实 cancel，记录请求时刻、ACK、同任务 CANCELED→STOPPED、Land/disarm/Offboard 和碰撞评分。已自然结束的任务不算有效注入。
- 深度中断：暂停自有独立传感器发布进程/容器，保留 PX4 与时钟，记录最后新鲜测量、故障触发与恢复时刻；观察 STALE_OR_INVALID_INPUT。恢复后复测真实消息，不把单纯 unpause 当恢复通过。
- 推理超时/忙/异常：分别验证首请求无结果、HTTP 失败与迟到响应；确保未形成多请求堆积。fixture、真实 worker 故障与飞行故障分别标注，不能由 fixture 外推完整平台结论。
- 故障任务的普通 mission success=false 可以是预期；需另附故障评估结果及原始时间线。没有正式故障驱动/评估脚本时，按运行前判据记录可复核操作与结果，不伪称自动化入口已经存在。
- 停止振荡：采集运动参考、实际控制命令、模式与定位，对照当前镜像实现；先做原生 Takeoff/GoTo/停止/Land，再跑完整模型。当前专属 PID 配置只在记录的平台上验证，不直接推广真机或认定共享控制器存在通用缺陷。
- 有屏卡顿：同时采集深度年龄、唯一戳率、渲染方式/分辨率和同驻负载。远程桌面、截图和渲染可能耦合，但降分辨率后通过不独立证明具体瓶颈；必须保留原失败，不能因此关闭真实相机或用离屏替代有屏要求。
- 跨轮：按实测单连接/残留状态要求，先停止本任务 PX4，再重启本任务 AirSim，最后启动 PX4；AS2 FSM 残留另行处理。仅认本次启动后的就绪日志，重新检查 clock、定位、输入和 disarmed。进程清理由 ID/可执行路径/启动代次约束，不用全局名称杀进程。
- 长时运行 AirSimNH 的图像 RPC 劣化恢复：控制 RPC（4780）稳定不代表图像 RPC（45471）健康；depth 持续超时/间隙越界且 fresh 栈实测 max gap 显著更小时，用 fresh AirSimNH + fresh `-userdir` 恢复，不是调大阈值。重启顺序固定为：先 stop vehicle/sensor 容器 → 杀 AirSimNH → 等 ~12 s GPU 释放 → 起 AirSimNH → 起 vehicle → 确认 4780 ESTAB → 起 sensor → 重建 compute；clock 容器跨飞不重启。AirSimNH SIGKILL 后不可立即重启（GPU 未释放会 signal 6 core dump，实测复现）；vehicle/sensor 容器在 AirSimNH 死亡期间保持运行时 `docker start` 是 no-op（内部 RPC 已死），必须显式 stop+start。违反顺序实测 PX4 重连竞态 → `TcpClientPort::accept` 异常 → signal 6。每飞换独立 mission 输出目录（`exist_ok=False`，root 属主残留需容器外特权删）。

## 6. 证据交付与结论

每轮保留 result.json、mission.jsonl、plans.jsonl、preflight.json、输入/模式/控制记录、故障注入与恢复时间线、源码和配置散列、镜像身份、清理报告及工件索引。任务 collision=false 的监测窗口与落地后的原始 CollisionInfo 分别说明；不能声称未采样的整个生命周期始终无碰撞。

validation.md 保留失败与修正顺序。2026-11-24 的后续章节报告过离屏正常/故障、低分辨率有屏和高度跟踪的分项结果，但它们不是天然属于同一版本的完整矩阵；演示长航线/树边记录也明确不是验收，并包含碰撞。编写或复核 Skill 不产生新的飞行结果，更不验证用户正在运行的另一份服务器源码。

最终逐项标记 pass/fail/uncovered，并给出可取得的证据。入口尚未正式交付、源码身份不一致、指标未预声明或任一适用项未完成，都不能报告完整复现成功。具体历史数值留在组件 validation.md，不在通用 Skill 中持续追加跑次。
