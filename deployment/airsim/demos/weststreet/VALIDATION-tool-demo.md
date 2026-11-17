# WestStreet Tool Demo 真实验证（2026-11-16）

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
