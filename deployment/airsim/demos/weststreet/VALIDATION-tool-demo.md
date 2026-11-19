# WestStreet Tool Demo 历史验证记录

当前维护版本及 side 会话修复见 [恢复版说明](README-before-plaza.md)，
入口为 `launch_before_plaza.py → demo_before_plaza.py`。
本文各轮结果保留为历史证据，不代表当前版本的结论或启动方式。
失败的绕柱／转场实现及其 Navigation 停止更新补丁已撤出活跃源码。
本次整理遵照用户要求，未重新运行测试或仿真。

**结论：真实链路已运行，5秒移动追踪未通过，整体未完成。**

工作站demo只调用System、Tracking、Navigation；本轮场景启停在宿主fixture，
算法/适配/AS2在算力容器，飞控在隔离PX4容器，场景为有屏UE5.2 ProjectAirSim
WestStreetCircuit。实际Runner先暂停，起飞并取得有效目标后才通过场景RPC启动。
实际GPU模型为yolo26n-seg，先验身高1.7m，无目标真值反馈进入算法。

## 通过的阶段

- `log/weststreet-demo/live-system-04/`：System实际Takeoff3m、Land成功，
  2152个里程计样本、最大爬升2.9605m、最大速度0.5681m/s，最终PX4上锁和清理。
- 实际导航地图：上置FRD(0,0,-.2)m、水平环扫±15deg的16通道LiDAR，形成
  14826个膨胀占据点并通过就绪门。地图配置考虑ROGMap虚拟地面膨胀语义。
- 真实RGB→CUDA YOLO→已知身高测距→Tracking→任务Navigation目标更新→
  SUPER多项式→扩展FollowTimedPath→AS2→PX4链已实际消费，非替身测试。

## 移动阶段失败及证据

| 轮次 | 实际行为 | 判定 |
| --- | --- | --- |
| live-follow-08 | 起飞、有效目标、人物跑步、2目标更新、17执行参考；输入窗口位移0.085m | 失跟，未完成5s；取消并System降落上锁 |
| live-follow-09 | 所有检测3D有效，检测往返56–78ms，但真实输入33.903→34.458间隔555ms超过Tracking500ms有效期 | 失跟，安全停止降落；保留原阈值 |
| live-follow-10 | 320×240同90deg FOV、匹配CameraInfo、RGB-only，输入间隔99–138ms；15次有效3D检测 | 关联拒绝后失跟；未完成5s/1m门，安全降落上锁 |

live-follow-10在跑步初始阶段的测距x序列包含7.42→9.13→8.30→10.35→10.29→12.03m；
相邻帧变化约1.7–2.1m，Tracking状态从tracked变predicted，最后有效观测33.411、
valid_until33.911，随后失效。原EKF使用0.02m²测量噪声及空间关联门，不能直接假定
适合这种跑姿相关的尺寸估距误差。尚未修改锁定上游或放宽关联/失跟门。

最新录包：2216个里程计样本、最大爬升2.9699m、292个SUPER多项式消息、43个
实际轨迹参考、4次目标更新；日志更新窗口最大水平位移0.222m，未达预声明1m。
这不是持续追逐5m/s人物的成功证据。精确时间线见tracking-states.jsonl、
detections/calls.jsonl、mission/targets.jsonl和evaluation.json。

## 修复与仍缺能力

- 已修复AS2准备服务同步等ACK时发布暂停的适配；服务结果明确且恢复原0.75s
  新鲜状态后才提交起飞。ROS新增暂停/取消回归后9项通过。
- 已修复本地/全局坐标查询混用、工作站overlay的nounset加载、运行日志stderr遗漏。
- 原640×480 RGB-D继承配置有大图传输间隔；本轮高度策略不需要depth，明确改成
  小RGB并重建实际内参，未将resize当作几何校正或放宽时间保护。
- **待补Tracking能力**：尺寸先验测距的不确定性、跑姿跳变/异常值处理与运动关联，
  需要独立数据回放及真实移动复验。不是通过提高门槛把失败轮改判通过。
- 原`follow=True`仍为原地朝向，SUPER目标yaw链、统一追逐+朝向会话、路口转弯、
  整圈遮挡、避障保证和完整故障矩阵仍未通过，不宣称完整复现成功。

## 清理与复验

最新轮System Land结果landed/disarmed/stop_confirmed，PX4实际arming_state1；
cleanup.json确认所属服务停止。原有keen_kare和aerohub-course-wmm-px4未停止。
原始失败轮保留；live-system-03的独立PX4恢复不计System成功。

入口`run_tool_demo.py`，离线评估`evaluate_tool_demo.py`，依赖与命令见README。
构建7包/模块加载通过；任务策略9项、System ROS9项通过不替代本轮动力学结果。
相关Skills依据实际问题更新：平台传感器适配及AirSim/PX4参考、端到端验证。

## 平均/过滤阶段更新

真实`live-filter-02`通过录包证明短窗平均被实际消费：8次smoothed、4次warming_up、
2次multiple_candidates_bypass；相对原观测修正中位0.178m、最大0.885m，未作真值
误差改进声明。实际水平位移1.399m、98条运动参考，仍因有效观测间隔0.519s超过
既定0.5s门提前失跟，5秒任务失败。正常System降落/上锁/清理已确认。

初版`live-filter-01`0.4s历史未能覆盖稀疏观测，全部预热，不构成真实平滑证据；
修正到5样本/0.8s并新增0.3s间隔测试后再跑，不改判旧轮。默认none仍保留上游
数值对照；36项相关回归通过。原记录与filter-audit/evaluation包均保存。

## DDS修复后的连续任务推进（2026-11-17）

`live-native-audit-05`在单变量提高接收缓冲后完成5秒任务，独立录包门全部通过，
19个不同capture目标更新、实际水平位移19.44m、248条执行参考；详情TIMING.md。
不把该轮通过推广为持续可靠性或完整环线通过。

后续预声明20秒的`live-track-20s-01`提前失败：5次目标更新、实际位移0.135m，
任务错误`Tracking target unavailable: predicted`。正常System降落、解除武装及所属服务
停止均已确认，host接收上限恢复到212992。图像传输未回归：原生13.14墙钟Hz、
检测13.02Hz、buffer12.62Hz，原生→桥接收与桥放行→检测无缺帧，UDP drop增量0。

真实测量回放`association-audit.json`重现同一失跟：35.094s最后接纳后前向速度估计
1.024m/s；35.175s观测与预测距离3.067m（马氏分数6.945低于11.345），因3m硬距离
门被拒绝，随后差值扩大。35.616s blind=0.522s。滤波19次smoothed，观测持续到达，
不能把本轮失败再归因为图像掉频。3m硬门和0.5s失跟门未修改。

只在离线比较了20m/s²加速度标准差候选：同段保持关联，但速度估计峰值10.8m/s，
且只覆盖短段、不是独立验证；保留`acceleration-model-candidate.json`，不应用于飞行、
不视为已校准模型或修复。下一缺口是Tracking面对静止→5m/s突启动及尺寸测距噪声
时的运动模型/不确定性适配；无需新增Tool或Behavior。转弯、整圈和可靠性仍未完成。

联合入口已增加图像任务启动前`rmem_max>=4194304`检查并记录实际值，不自动修改
宿主；避免后续在已知不满足的传输条件下重复飞行。System-only不受此图像检查限制。

## 平滑启动与短期历史修正复测（2026-11-17）

人物插件已实际验证默认1秒smoothstep启动，原start/stop/reset RPC、路线和动画
不变。以原20秒预算和原Tracking门测试，未完成；不能将启动检查通过记为追踪通过。

| 轮次 | 结果 | 实际证据 |
| --- | --- | --- |
| live-track-ramp-20s-01 | 20秒门失败，启动段不再重现原3.067m关联拒绝 | 9次目标更新、位移0.212m；速度估计逐步达到约5.4m/s，后因预滤波连续4次jump_rejected失跟 |
| live-track-ramp-20s-02 | 20秒门失败，有效跟踪约5.20秒 | 19次目标更新、目标更新源时间跨度4.767s、位移15.617m、250条执行参考；后因有效观测中断失跟 |

01原始观测重放发现一次额外候选导致平滑历史清空，随后3个下降的尺寸测距
重新定义反向趋势，接下来的4帧被连续拒绝。只修正`robust_linear_v1`多候选旁路：
候选原样送给原关联、不加入平滑，同时保留仍满足原时间窗口的近期历史。
没有修改Qt/Rt、3m关联硬门、预滤波3m异常门、0.5秒失跟或窗口长度。
40项回归通过（含新增历史误判回归），4个overlay包增量编译通过。

02为独立场景新数据，不是旧片段回放。检测12.59墙钟Hz、buffer12.25Hz、UDP drop
增量0，两段传输无缺帧；48次smoothed、3次多候选旁路、2次no_measurement及6次
jump_rejected。末段38.865s接纳后，38.934/39.009/39.156/39.225四帧预滤波拒绝；
39.303/39.375s采样后续恢复，但在其处理前有效期39.365s已到，39.366s反馈lost。
不能把仍有RGB或新的采样stamp当作已及时接纳有效观测。

原多候选清空引起的失效未在02重现，但20秒连续目标仍未通过。剩余问题是尺寸
测距噪声与前置趋势拒绝造成有效观测中断，需要同帧原始框/TF/测距/预测及独立
真值评分进一步区分；不继续叠加参数或放宽失效门获得表面通过。
两轮System降落、解除武装、所属服务清理通过，宿主rmem_max均恢复212992。
原始证据分别位于`log/weststreet-demo/live-track-ramp-20s-0{1,2}/`，含任务结果、
录包、滤波/关联/传输审计、evaluation及cleanup；40项回归见ramp-history-tests.log。

## 可控残差软更新（用户授权，2026-11-17）

新增opt-in `soft_innovation_v1`，默认上游/旧innovation策略保留。WestStreet使用
`soft_innovation_v1 + position_filter=none`：创新残差以Huber权重增大本帧测量噪声，
Kalman增益缩放限制单帧位置和速度修正，并用Joseph形式更新协方差。新策略不再
以旧3m距离或前置短窗趋势直接拒绝可控偏差。超出原运动预算推导的8m几何范围、
多候选分数接近仍不强行关联；不提供外观身份保证，也没有新增远处候选重捕获。
0.5s真正无有效观测失效、时钟/世代/几何保护未放宽。

43项回归通过：原算法数值对照保留，两段真实失败观测新增回放不再失跟；单帧
跳点修正有界、持续偏差可修正、远处异常不刷新观测、歧义候选隔离、实际缺测停止、
协方差半正定检查通过。首次配置分支遗漏报错已修复，保留首次失败与最终通过日志
`soft-tests.log`/`soft-tests-final.log`。4包增量编译通过。

`live-track-soft-20s-01`独立实测任务未通过，原因是
`Navigation ended: EXECUTION_FAILED:FOLLOW_PATH_UPDATE_REJECTED:drone0`。
本轮Tracking19次更新、有效反馈65条、确认后无invalid反馈；实际权重均为1且未触发
软限幅，说明仅覆盖正常更新阶段，未充分激励真实软降权分支，不据此声明噪声飞行
问题已修复。无人机位移0.172m、43条执行参考，20秒/1m门未通过。

检测12.99Hz、buffer12.56Hz、UDP drop增量0。原执行服务只返回accepted布尔值；
已记录adapter revision10/late_updates1、当时final轨迹与控制器stale/duplicate警告，
尚无证据将其确定为时序、连续性或生命周期哪个原因，不改执行门或忽略拒绝。
任务经原停止与System Land安全收口，landed/disarmed/cleanup确认，宿主rmem恢复212992。
后续阻塞点为Navigation更新拒绝的具体归因及恢复，再完成20秒软更新真实复测。

## Navigation更新拒绝的确定性归因与修复（2026-11-17）

原失败`live-track-soft-20s-01`的adapter状态记录：revision1–9的终点轨迹均覆盖到
35.153291604490114s，revision9从35.122s执行；revision10原生新曲线从35.163s开始，
adapter裁取的执行起点为35.281s，产生127.708ms覆盖空洞。原Adapter仍调用旧执行
client.submit（revision>1意味着modify），把两个不重叠执行阶段误当滚动替换。

**可证明的是该修改违反现有执行契约**。实际C++ Timeline使用原时间、其余分量
构造成连续的最小回放仍拒绝`stale, excessive pending, or incompatible update`；
因此只凭时间已足以拒绝，无须假设位置/速度超过门限。旧轮没有记录服务端是否
已清空timed_，不能事后声称其必定走inactive还是Timeline校验分支；两者都源于
旧覆盖已结束仍尝试修改。回放与原生日志附近的controller stale警告不能混为因果。

证据：`nav-gap-replay/replay.cpp`/result.txt为实际C++校验最小回放，
`nav-gap-red.log`为实际Adapter.tick新回归修改前3项失败（确实错误调用submit）。
修复后新增7项全部通过；加上原SUPER转换/数值测试16项、ROS接口13项、Framework
29项全部通过，见`nav-gap-final-build.log`。诊断临时DEBUG-nav-update标签已移除，
保留可复用的服务拒绝状态、执行终态和原时间诊断，不改变执行门限。

修复只在SUPER计划交接：记录最后提交的真实覆盖终点与final标志。新计划起点
晚于旧覆盖终点时，HOLD任务的终点轨迹走现有停止确认/重新接管流程，保持同一个
Navigation Goal和运动资源所有权，重新请求实际SUPER计划。交接期间缓存同一拥有者
的最新目标，确认停止前不启动新执行；取消/终止仍拒绝更新。非final或非托管场景
仍以TRAJECTORY_COVERAGE_GAP失败，不忽略拒绝、不移动时间戳、不填造中间轨迹。
状态新增submitted_segment与coverage_handoffs，便于区分滚动更新和重新接管。

真实验证：

| 轮次 | 条件 | 结果 |
| --- | --- | --- |
| live-nav-rejection-01 | 仅加打点，原20秒任务 | 未复现偶发拒绝；20秒全部阶段门通过，位移90.877m、1002条执行参考，257次观测中4次软更新 |
| live-nav-rejection-hold-01 | 仅加打点，识别后延迟2秒启动人物 | 未复现拒绝；任务完成并安全降落；此轮仅18秒左右人物运动，不冒充20秒移动测试 |
| live-nav-gap-fixed-20s-01 | 修复后原20秒无额外延迟 | 全部阶段门通过，位移90.258m、997条执行参考、262次观测中4次软更新 |

最后一轮coverage_handoffs=0、error为空，说明自然场景没有再次激励覆盖空洞分支；
不能把它当作该分支在真实飞行中的直接覆盖，也不宣称可靠性矩阵或完整环线通过。
该分支由原失败时间的实际Adapter回归及停止/目标缓存/取消协议回归覆盖。
各轮正常降落解除武装、所属服务清理，宿主rmem_max恢复212992。

## 已完成执行的自动路由补齐

用户明确要求modify发现旧执行完成时自动路由新Action。Adapter新增两个入口检查：
新目标更新时发现client.complete，以及新计划排队期间client.complete；成功完成才
可走现有停止确认/重新接管。目标未变只续期，取消/失败不会自动重启；Navigation
外层Goal保持同一，COMPLETE任务尚未结束时也可接受新目标。

新增3项单元回归先出现2项预期失败（completed-route-red.log），修复后真实ROS接口
14项、单元/转换19项、Framework29项共62项通过。真实ROS新增用例在停止确认延迟
期间连续更新目标，确认无提前启动、后续新执行只使用最新目标且外层句柄不变。
这些ROS测试使用受控执行double，明确属于协议证据。

`live-completed-route-20s-01`真实ProjectAirSim/PX4原20秒任务及独立录包全部阶段门
通过：实际位移90.145m、991条执行参考，正常停止、System Land解除武装并清理，
host rmem_max恢复212992。该轮不替代独立路由协议测试或完整环线/故障矩阵。
构建与回归见completed-route-build.log，原始飞行证据在对应run目录。

## 2026-11-17：第一视角与首个路口实测

HUD 原本仅登记 streaming-enabled，相机收敛到 capture-only FrontCamera 后列表为空，
Tab 无效。观察入口现显式登记现有 RGB 组件，不新增采集。SetView 返回 bool，
状态另用 GetDisplayState；已修正首轮把 bool 当 dict 的启动错误。
`live-turn-view-35s-02` 实际窗口 Tab/G/Tab 通过、截图确认人物可见，但复用场景
导航输入准备超时，没有飞行。复用 native relay 的生命周期仍需单独核查，
不把超时归因为已确认的传输原因；本轮恢复采用停止所属资源后完整重启。

`live-turn-view-35s-03` 实际 ProjectAirSim/PX4 35秒预算测试**未通过**。
人物在 target_acquired 后21.322秒进入首个路口；转弯前无人机速度约
(4.93,0.03)m/s，过弯后约2秒变为(3.64,3.26)m/s，证明确实开始转向运动。
约27秒时 Tracking 因 predicted/缺测失效，99次目标更新、1348条执行参考，
最大水平位移107.57m。这些数值不代表35秒或转弯跟踪成功。

最后有效观测采样78.312s，人物框约9×21像素；后续78.381s起多帧检测为空，
原图中人物仍可见，处理往返约54–78ms。第一失效链是连续无人物检测后
超过0.5秒有效期，不是已有有效测量被创新关联门拒绝。云台曾接近-97度
偏航，但空检测帧人物仍在画面内，不能据此认定是云台限位。

当前固定世界坐标跟随偏移、固定机身yaw没有随人物转弯改变，且过弯时
运动出现明显波动；需进一步拆分追随几何、测距误差及动力学/碰撞/定位变化，
不能仅扩大缺测窗口。原图对照loss-frame-pair.png、独立人物轨迹、turn-audit.json
和录包均在该轮log目录。

收尾Navigation/Tracking停止后System Land返回TIMEOUT，landed=false、
disarmed=true；PX4独立检查确认解除武装，不能报告正常降落通过。
按事先--keep-scene配置停止执行服务，保留所属场景/PX4/桥接/HUD供Tab观察；
evaluation.json明确mission_success=false、phase_passed=false。宿主rmem_max已恢复
212992。完整环线、可靠性和转弯成功仍未完成。

## 机头绑定相机现状

`live-body-camera-2m-01`起飞实际高度1.925m，配置相机FRD z=+0.8m，
关闭主动pointing，保留roll/pitch稳定、yaw随机头。转弯仍未通过，
报Tracking target unavailable: predicted；System Land成功，确认landed和disarmed，
无清理错误。尚未增加机头追人控制，不宣称此配置修复转弯。按用户要求
暂缓碰撞分析与转向修改，先推进YOLO同帧小窗。

## 新TARGET_YAW真实任务：live-target-yaw-35s-01（失败）

使用增量构建后的Adapter及新Demo：经System Tool查询自身位置，由Tracking
输出计算目标视线yaw（目标状态header时刻到当前+0.1s的预测，总前移限0.3s），
Navigation.to_pose(TARGET_YAW)同任务下达位置与朝向。默认不开启；
--target-yaw开启部署和调用。位置仍固定初始世界偏移，没有其他控制链。

原35秒预算真实ProjectAirSim/PX4任务未通过。2m起飞实际1.879m，
人物首弯在acquired后21.591s，首invalid为23.174s；最终Tracking target
unavailable: predicted。原缺测门保留，未降低验收标准。

录包sim时间/目标yaw/执行参考yaw/实际yaw（度）：

|sim|目标|执行参考|实际|
|---|---:|---:|---:|
|72.834|9.25|0.56|0.64|
|73.104|18.24|0.77|0.75|
|73.416|31.04|1.52|0.91|
|73.713|44.90|3.40|1.81|

SUPER日志收到目标yaw，说明调用/Adapter已贯通；延迟在执行参考中已明显
存在，不能主要归咎PX4收到了45度却不响应。源码的goal_yaw是末端约束，
滚动重规划从既有yaw状态接续，不等价于当前时刻持续朝向目标；实测参考
朝向增长明显慢于目标视线。后续应讨论将持续视线朝向与位置轨迹末端约束
解耦，仍留在同一Navigation/Behavior执行链，不另发竞争控制。

图像显示人物左移至边缘：73.467s框x=13.5..65.4，73.536s已截断
x=0..37.7，73.608s起连续空检测；该段推理往返53–66ms，不是推理长时间
卡住。最后可用于跟踪观测73.467s，73.967s过期。

System Land成功、landed/disarmed确认、无清理错误；执行服务已停止，按
keep-scene保留所属观察资源。host rmem_max恢复212992。evaluation.json
phase_passed=false。证据yaw-audit.json、yaw-reference-probe.json、录包及
各服务日志在本轮log目录。停止实验，未声称转弯验收通过。

## 连续yaw首轮：live-continuous-yaw-35s-01（未通过）

新增CONTINUOUS_YAW可选模式，默认关闭；位置/PVA仍用SUPER，yaw在
同一个FollowTimedPath执行器中限角速度/角加速度，统一发布运动参考。
目标更新服务仅刷新活动任务yaw，不启动运动、无独立控制发布者。
任务ID/版本/有效期校验，位置滚动替换不重置yaw；取消/过期停止。
C++限速/环绕/所有权数学测试、真实ROS编译Behavior的独立朝向更新、
错误任务/旧版本拒绝、取消/过期测试通过。反向协议测试最初0.6s比较
因限加速度减速过程不满足，按物理减速时间改为0.8s后通过，不改控制限制。

接口、Behavior、Framework、Adapter、Navi API和工作站接口在已有overlay
重新编译，没有重建镜像。Adapter23项、Framework32项、Demo10项回归通过，
Timeline和YawTracker两项CTest通过。原FollowPath/TimedPath ROS冒烟通过。
这些是接口和执行协议证据，不替代实飞任务结果。

实飞沿用35秒预算、人物5m/s、2m起飞/相机下悬0.8m、原位置偏移和缺测
门。实际起飞1.940m，首弯在acquired后21.436s，首invalid为23.171s。
任务失败：Tracking target unavailable: predicted。

|sim时间|目标yaw°|执行参考yaw°|机头yaw°|
|---|---:|---:|---:|
|71.127|20.49|8.46|0.77|
|71.412|35.31|15.77|2.43|
|71.694|48.11|27.17|7.26|

相比末端yaw模式（近45度目标时参考3.4度），参考显著提前，证明新执行
路径生效。但本轮仍有目标到平滑参考的延迟，以及参考到实际机头的延迟；
不能再单独归因SUPER末端规划，也不能声称无人机已能及时反应。
当前角度控制经AS2 yaw PID产生yaw_speed，实测配置kp=1，ki=0、kd=.001；
这提供下游延迟的待核查方向，不以一次飞行认定唯一根因或直接修改飞控。

正常System Land、landed/disarmed、无cleanup_errors；停止执行服务，按
预声明保留观察场景。host rmem_max恢复212992。evaluation.json标失败。
证据目录包含yaw-audit.json、yaw-reference-probe.json、原始录包与服务日志。
未放宽门限，不改失败结果，不继续无证据调参。

## 原路线1秒短时恢复：live-recovery-1s-original-35s-01（失败）

仅算法侧显式延长预测窗口至1秒，原急弯路线/5m/s/连续yaw/位置偏移不变；
恢复状态冻结最后位置目标、继续yaw，不宣称机体立即停止加速。默认0.5秒
与关联门/修正限幅原预算保留。真实先前0.804秒缺测回放已通过1秒候选。

实飞35秒预算仍未通过，约23.695秒首次invalid：最后有效测量54.630s，
截止55.630s；恢复采样55.500s（间隔0.870s）在55.656s才返回，晚于截止
26ms，尚未被Tracker消费时Mission已失效。不能把后来恢复倒写成该轮成功。
提示恢复预算应包含采样缺口和观测消费延迟，而不只看帧时间差。
后续1.2秒为独立候选，不修改本轮预算或结果。

System Land返回FAILED，landed=true但disarmed=false/stop_confirmed=false，
不能写正常起降通过；独立PX4最终确认disarm，所有所属执行/仿真资源停止。
fixture持有的小窗在finally退出，本轮不保留场景。原始录包和eval保留。

## 原路线1.2秒恢复：live-recovery-1p2s-original-35s-01（仍未通过）

基于上一轮0.870秒采样缺口+0.156秒到达延迟，独立预声明1.2秒预测预算，
不改变原1秒轮的失败结论，原路线和其他门保持不变。
首弯后recovery_started（age=.471s）到recovered（age=.156s），并持续
收到有效三维目标；末段Tracking仍valid=true，说明这次跨过了短时缺测。

随后Navigation update rejected: STOPPING；服务端精确原因是滚动轨迹
不连续：position=.001675m、velocity=.059314m/s、acceleration=1.508881m/s²，
超过当前1.4m/s²门。此轮不是再次因失跟失败；不能只看Mission最终STOPPING
而丢掉下游原始拒绝原因。未增加门限、未忽略拒绝，后续需核查恢复时的
目标/计划衔接，不能把局部恢复成功当35秒任务成功。

正常System Land、landed/disarmed、无cleanup_errors；全部所属服务停止，
小窗及子进程未保留，rmem_max恢复212992。eval phase_passed=false。
UE温和路线尚未用于该轮，不混入算法侧A/B。

## UE路线单变量：live-smooth-turn-default-window-35s-01（失败）

独立新地图WestStreetSmoothTurn与序列WestSmoothTurnMotion已实际落盘、
重载；runner-reset确认duration=110.8s。原地图和序列SHA256与生成前一致。
真实UE地面/人物胶囊检查通过，生成期间遇到原新直线切入障碍处，已通过
与原路径出弯切线正确拼接修正，没有跳过检查或缩小半径；详见支线记录。

本轮保持原0.5秒Tracking窗口，排除与算法侧1.2秒组合混淆。首弯独立
Actor真值7个样本：半径3.998806–3.999364m，速度中位2.749602m/s；
直线速度中位4.999505m/s。新路线运行与速度得到验证，不等于无人机
跟踪或无碰撞通过；动画逐帧相位/脚滑尚未进行独立视觉验收。

任务约27.506秒失效，最后有效观测58.050s。但在此之前，UE于
07:20:47.093 UTC记录Frame与light_streetlight_complete8碰撞，约为
acquired后25.656秒，z=193.615cm，早于首次失效约1.85秒。
存在碰撞后观测/状态漂移，不能只把本轮失败归因为0.5秒窗口。

System Land最终SUCCEEDED、landed/disarmed且无cleanup_errors；
全部所属服务及小窗停止，rmem_max恢复212992。eval phase_passed=false。
原始日志、actor-route-audit.json、录包都保留在该轮log目录。
按用户此前暂缓碰撞处理的范围，本轮未修改避障；组合实验因这个已知
新阻塞暂缓，未通过的35秒矩阵仍保留，不能删去碰撞项改判成功。

UE资产绑定复核另有缺口：已存override列表含旧runner和相机条目，
尚未逐GUID证明当前新序列唯一绑定；实际人物沿新路线运动的证据成立，
但不提升为绑定唯一性验收。生成/finalize源码补显式reset_bindings，
未重写本轮已落盘资产；该补充尚需实际UE执行验证，不能据语法通过改判。


### 2026-11-17 首弯右移2m独立实飞

用户授权清理的两个旧生成包 package-course-v7、hud-v1/package 已删除；
其余日志及WMM包保留。新独立地图 WestStreetShiftRight2m / 序列
WestShiftRight2mMotion 保留原图；起点不变，首弯沿UE-X负向平移2m，
进出段平滑过渡，保留4m圆弧、直线5m/s、弯道2.75m/s。
真实UE生成成功，新资产binding override恰为1；几何7项、图像2项、
ROS环境Demo13项测试通过，不代替实飞。

实际任务 live-shift-right2m-default-window-35s-01，continuous-yaw启用，
恢复窗口保持默认0.5秒。获得目标后26.05秒仍接触同一根
light_streetlight_complete8，随后失跟，任务失败。
原轮首次接触位置局部坐标(105.3481,5.2709,1.9041)m；新轮为
(105.1293,5.4962,1.8940)m，水平差约0.314m。
这是各自接触时刻的位置比较，不是同一时刻轨迹位移或净空提升；
人物路线右移不能等同于无人机轨迹同样右移2m，不据此断言继续平移即可解决。
新轮接触时参考与实际位置差约0.192m；需区分目标路线、规划参考和实际运动，
暂不修改碰撞体、避障参数或继续扩大平移。

Land SUCCEEDED，landed/disarmed确认，cleanup_errors为空；全部所属
服务及小窗停止，host rmem恢复212992。碰撞audit含降落路面接触，
不能把其airborne_contacts计数解释为追踪中的独立碰撞次数。
PNG证据模式复用实际HTTP推理请求已有PNG，无新增传感器采集或二次编码；
默认仍raw。磁盘低于128MiB则停止图像落盘、终止任务并走降落清理。
证据：log/projectairsim/weststreet-shift-right2m/ 及上述实飞目录。


### 延后4m候选真实结果（2026-11-17）

live-shift-right2m-late4m-default-window-35s-01已完成，mission success=true，
pursuit_window_complete=true，116个不同capture目标更新，追踪水平位移
115.79m，1740条实际执行参考；没有撞light_streetlight_complete8，
35秒预算覆盖首弯及后续段，未触发失跟。continuous-yaw及0.5秒窗口保持。
这是一轮有界首弯追踪通过，不代表全loop/重复可靠性或完整复现通过。

collision-audit仍passed=false：首次接触为wall1794903785.805，
ground_mod_curb_a136，局部高度0.1338m、垂直速度-0.525m/s，
发生在35秒追踪结束之后的降落阶段；共2条路沿接触日志。
必须保留该碰撞失败，不能以mission成功宣称全程无碰撞。
Land SUCCEEDED、disarmed=true、cleanup_errors=[]；所有所属服务/小窗停止，
rmem212992恢复。原地图未覆盖，无新Git提交。
证据：该轮mission/result.json、evaluation.json、collision-audit.json，
以及log/projectairsim/weststreet-shift-right2m-late4m/live-audits.log。


### 候选观察点第一轮结果：失败，保留默认关闭

live-observation-candidates-65s-01在获得目标约50秒后失效，未跑满65秒，
没有优于原第二弯约51秒失败，不能宣称方案有效。Land SUCCEEDED、
disarmed、cleanup_errors为空、所有所属服务及小窗清理、rmem恢复212992。
可见追踪段无接触，随后落地有道路接触，不能宣称全程无碰撞。

policy-failure-summary.json：t83.412估计人机距5.353m，之后t84.294
拉回7.354m，但目标方位与实际机头偏差增至约34.44度。YOLO原框
t83.871左边x9.78..75.64，t83.949开始左裁框，随后空检测。
与原轮从右侧裁框不同；距离改善和持续可见性不是等价指标。
该距离使用Tracking预测位置与同sim时间最近自身odom，含测距/同步误差，
不是精确人物真值距离。候选模型输出的转头需求可超过1.5rad/s，
其朝向状态直接按限速转动，未建模真实机头响应/加速度及跟随误差；
因此不能以简化rollout筛选当作真实轨迹可行性。

本轮没有调整跟丢/测距/连续性门，未加历史路径跟随，未改镜像或控制出口。
实验脚本与默认关闭开关保留，下一步先校准实际位置/转向响应后离线筛选，
不盲目扩候选或把仅选终点提升为Elastic轨迹优化。两圈目标仍未完成。


### 温和第二弯实飞复核：问题依旧，75秒未通过

live-gentle-second-75s-01获得目标后57.7367秒首次lost，未跑满75秒。
独立actor记录30个第二圆弧样本，速度1.9982..2.0010m/s；路线改动真实生效。
但出弯加速后再度迎面接近：sim86.112估计距离5.718m，sim89.631
2.993m，sim90.516仅1.526m；最后一刻机头与估计目标方位偏差约7.76度，
说明该失效不能主要归因yaw。90042000000.png人物正面占满画面，
下半身超幅，随后连续空检测；原2D框在下边缘附近，测距仍偶有接受，
因此不能把所有失败描述为明确truncated拒绝，本轮末态为lost/空检测。

同sim90.516，自身局部(90.598,118.051)，估计人物(92.098,117.771)，
下达目标(84.098,117.771)，人物运动朝局部-X；固定-8m偏移仍位于
人物前进方向一侧，无人机在其前方退让，没有变成后方跟随。
UE actor近邻坐标的粗略对齐同样支持近距离，但含原点标定和采样时差，
不把粗算0.3m当精确真值。review-geometry.json保留对照及时间差。

原生日志追踪期无接触记录，降落路面接触；Land SUCCEEDED、
landed/disarmed、cleanup_errors=[]，全部所属服务小窗清理，rmem恢复212992。
场景仅延缓失效，没有解决固定跟随几何和出弯接近问题；暂不继续
扩大半径、降低全程速度或新飞行，按用户要求审阅汇报。


### p/v负切向首轮实飞失败复核

live-tangent-pv-65s-01获得目标后27.993秒失效，尚未到第二弯，
Land SUCCEEDED、landed/disarmed、cleanup_errors=[]。追踪阶段无原生接触，
所属服务停止及rmem恢复另以cleanup元数据确认。
末段目标框30..40px高并间歇空检测，机体到Tracking目标估计约15.9..17.6m，
到下达参考点误差约9.4..10.9m，不能称已稳定保持8m或长期轨迹验收通过。

用户指出暂时跟丢即fail；逐帧复核支持退出过早是直接触发点：
最后有效采样59.115，默认截止59.615；59.499已有下一次有效三维检测，
但结果59.700才返回，晚85ms；两次采样间隔384ms，含处理201ms。
之后59.571/59.871/59.952也有有效三维检测。
这证明有效检测在恢复，尚未证明延长窗口后关联、轨迹和后续任务通过。
可用已有显式recovery-window=1候选独立实测：新鲜度/观测时间保持，
短暂缺测冻结位置目标、继续yaw；0.5默认不改，不无限续期或忽略lost。


### 两轮1秒恢复实测结论

第一轮recovery1s在29.440秒失败，18次recovered，26/96次更新冻结；
第二轮entry600ms在29.366秒失败，1次recovered，3/95次更新冻结。
降低正常延迟误触发并未延长追踪，不能把冻结多认定为主要根因。
第二轮最后有效采样59.247、返回59.481，框高29.2px；下一有效采样
60.249、返回60.489，间隔1.002秒且有0.240秒处理延迟，
已超过60.247有效期。不是仅85ms迟到的旧轮事件。

末段TargetState估计人机距离15.27..18.53m（非精确真值），
reference误差约7.62..3.99m；人物太小伴随连续空检测。
1秒窗口允许部分短时恢复，但未解决转弯后距离拉大和检测可靠性。
不继续放宽窗口；下一步应核对实际位置速度和下达轨迹响应，
区分参考点本身移动和SUPER/Behavior跟随滞后。
两轮均Land SUCCEEDED、disarmed、cleanup_errors为空，所属服务全部
清理、rmem212992恢复；65秒矩阵仍失败，两圈目标未完成。
证据recovery-review.json、mission/result.json、各轮calls.jsonl。


### 失跟真实帧CUDA检测对照（不飞行、不改部署）

log/weststreet-demo/yolo-loss-replay/保留compare.py、results.json、
summary.json、roi-review.jpg、background-results.json。模型独立venv
实际CUDA Predictor、锁定相同权重/预处理，原默认阈值0.35。
58.029..61.284s共33帧：默认8/33，0.2阈值14/33，0.1阈值18/33；
256x256因果ROI（由前一已接纳检测框中心初始化/更新）原阈值28/33。
默认重推33/33帧与录制输出一致；ROI不是按当前人物真值裁剪，
没有增加信息或使用未来帧。接纳ROI框逐帧人工看均为同一人物。
检出采样最大间隔默认1.002秒，ROI0.252秒；不含下游延迟/关联，
因此不能断言闭环窗口必通过。

同图右侧[384,128,256,256]无人背景33张，阈值0.35/0.1均0假人；
仅有限负对照，不能称通用误检率通过。降低阈值出现同目标重复框，
不把所有候选数等同人物召回。模型调用含已有PNG编码的中位耗时
全图12.62ms/ROI8.94ms（去前2帧），非ROS/DDS端到端延迟。
未启动workerHTTP/ROS/场景，未改变在线配置。建议后续显式ROI模式
恢复原图框/掩码坐标，用原相机内参/TF；追踪初始化/丢失须全图兜底。


### 不改感知重复轮：新执行故障，不判为漏检复现

live-tangent-pv-recovery1s-entry600ms-65s-02约20.80秒中止，
最终Navigation错误FOLLOW_PATH_REJECTED；继续读取录包和执行端时序
发现最早故障为STALE_LOCALIZATION，不是检测lost：
wall1794914068.464275，ROS52.473，revision188旧执行失败；
Adapter随后发布旧任务失败状态；wall4068.758新task出现，
4068.806新Action拒绝“initial point is not continuous with vehicle position”。
coverage_handoffs=0，不将此归因final覆盖切换；自行状态新鲜度门
既拒绝过旧也拒绝未来时间，具体原因仍未查明，不称定位数据源已断流。

YOLO0.35、无ROI保持，人物观测仍有效，此轮无法回答漏检是否偶发。
没有放宽连续性/定位门、没有启动同参数第三轮。Land SUCCEEDED、
disarmed、cleanup_errors=[]，全部服务小窗停止、rmem恢复212992。
证据onboard.log、rejection-adapter-states.json、mission/result.json。


### 1秒前看65秒实飞通过，继续长跑

live-tangent-pv-lead1s-recovery1s-65s-01完整65秒success=true，
3次恢复后接续，Land SUCCEEDED、disarmed、cleanup_errors=[]、
所属服务清理。YOLO仍0.35，无ROI，未额外增加漏检机制。
估计人机距min6.67/median10.91/max17.10m，非精确人物真值；
参考跟随误差median7.99/max13.37m，尚非精确8m跟随或速度前馈完成。
追踪阶段原生无接触，降落有路面接触，不能称全程无碰撞。
相对0.35秒前看本轮通过，单轮对照不能证明普遍因果/可靠性。

继续同参数live-tangent-pv-lead1s-recovery1s-240s-01，
预声明覆盖112.067秒路线两次回环，不重启跟踪会话；
真值仅用于评分回环，实际安全降落/清理另检。


### 长跑新限制：广场柱墩实体遮挡

live-tangent-pv-lead1s-recovery1s-240s-01获得目标后78.752秒失效，
无STALE_LOCALIZATION或接管错误，原执行故障未复现但未声称已修复。
65秒预算通过不代替长跑，两圈矩阵未完成。
最后有效采样108.075，截止109.075；下一有效采样108.981但结果
109.161返回，晚86ms。随后109.068/109.140持续有效且Tracker已恢复，
外层任务已退出。原图108075000000.png人物在右侧柱墩旁，
108489000000.png主体被实体柱墩遮挡，108981000000.png绕出重新可见。
属于真实遮挡，不是上一段中央小目标纯漏检；不据此盲目调YOLO阈值。

Land SUCCEEDED、disarmed、cleanup_errors=[]、全部服务清理、rmem恢复；
追踪阶段无原生接触记录，降落有地砖接触，保留全程碰撞边界。
本次按用户要求绕过偶发bug持续推进，依次完成同配置重试、1秒前看
65秒成功、同参数240秒新遮挡失败。当前候选可覆盖前65秒，
后续需讨论绕柱路径/可见性或遮挡恢复，尚不能持续循环。
