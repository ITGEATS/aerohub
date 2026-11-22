# AeroVLA 单步动作接入 GoTo 的源码核查

核查日期：2026-11-22。范围：静态源码、仓库配置及真实 ROS 原生 GoTo 合成状态联测；未完成飞行验收。
AS2 gitlink：`2336545fd04821ab3208ad7b2e394a8aeddd91c5`。
当前工作树子模块未初始化；读取主工作区中相同提交、无未提交修改的 AS2 源码。
以下 AS2 引用均指本仓库 gitlink 路径；须初始化子模块后查看。

## 结论

GoTo 的目标位置语义比 AeroVLA 当前人工生成的滚动 TimedTrajectorySegment 更接近单步位移动作。
推荐以位置版 GoTo 为复用基础。原插件原地转向可能提前成功，普通到点也不保证低速或 yaw 到位；
按用户后续裁决，先在 Tool 侧等待实测收敛并编排动作，不先修改 Behavior。新入口见
`src/tool/navigation/planner/aerovla/ros2_adapter/uav_aerovla_planner/goto_node.py`。原生宿主合成状态联测已验证提前 SUCCESS、最终 yaw 参考输出与 Tool 收敛门控；实际控制器消费和飞行仍待验证。
这不是已经完成的迁移方案或运行验收结论，默认入口的后续接入状态见末节。

## 接口、坐标与插件

- [GoToWaypoint.action](../as2/aerostack2/as2_msgs/action/GoToWaypoint.action)：目标为 `PointStamped`、`YawMode`、`max_speed`；结果为成功布尔值，反馈只有实际速度与目标距离。无时间化路径、加速度约束、yaw 误差、单目标超时字段。
- [YawMode.msg](../as2/aerostack2/as2_msgs/msg/YawMode.msg) 声明的枚举多于位置插件实际支持范围；位置插件仅支持 KEEP_YAW、PATH_FACING、FIXED_YAW。
- [go_to_behavior.cpp](../as2/aerostack2/as2_behaviors/as2_behaviors_motion/go_to_behavior/src/go_to_behavior.cpp)：目标 frame 必须非空，位置及 yaw 通过 TF 转到 `getEarthFrameId()`。调用者应提交带正确固定 frame 的 ENU 目标，不能直接提交裸 NED 数值。`max_speed=0` 使用默认速度；此层未验证有限性及正值，调用方需要校验。
- [共享配置](../deployment/airsim/config/as2/config.yaml) 选择 `go_to_plugin_position`。共享扩展 [启动入口](../src/behavior/follow_path/launch/motion_behaviors.launch.py) 启动 GoTo、Takeoff、Land 与扩展 FollowPath。AeroVLA [运行器](../deployment/airsim/demos/aerovla_blocks/run_case.ps1) 依赖未入库的 harness，因此不能证明历史运行实际使用了这个入口与配置。

## 位置插件实际执行与完成条件

源码：[go_to_plugin_position.cpp](../as2/aerostack2/as2_behaviors/as2_behaviors_motion/go_to_behavior/plugins/go_to_plugin_position.cpp)、[go_to_base.hpp](../as2/aerostack2/as2_behaviors/as2_behaviors_motion/go_to_behavior/include/go_to_behavior/go_to_base.hpp)。

1. 接受目标要求平台 FLYING 且已获得定位。定位由 twist 回调结合 TF 得到；并非只订阅 Pose 即可。当前检查没有定位新鲜度门槛。
2. 激活时计算 yaw 并缓存目标；运行时周期发送最终位置、yaw 和三轴速度参数，不生成完整时间化轨迹，也不调用避障规划。
3. 默认 `go_to_threshold=0.2 m`，`go_to_threshold_z=0` 时按三维距离判断；设置正的 z 阈值时，分别检查水平与高度误差。见 [默认配置](../as2/aerostack2/as2_behaviors/as2_behaviors_motion/go_to_behavior/config/config_default.yaml)。
4. `own_run()` 先检查位置完成条件，再发送参考。不检查 yaw 误差、速度或持续稳定时间。原地 FIXED_YAW 目标可能首周期成功，成功清理才发送一次目标位置/yaw，不能把 Action 成功当作转向已完成。小于到点容差的微动作同样可能直接成功。
5. 正常成功后再次发送最终位置/yaw 参考；其他执行结束发送 Hover。Hover 请求只是切换控制模式，不等于已确认实测速度收敛。
6. `max_speed` 被写入三个轴的参考 twist；在本提交的 PID speed controller POSITION 模式中作为各轴输出限幅使用，不足以证明三维合速度严格不超过该标量。见 [position_motion.cpp](../as2/aerostack2/as2_motion_reference_handlers/src/position_motion.cpp) 与 [PID controller](../as2/aerostack2/as2_motion_controller/plugins/pid_speed_controller/src/pid_speed_controller.cpp)。

## 更新、取消与跨行为互斥

- 位置插件实现 modify，能替换目标和重算 yaw，但不提供定时轨迹连续性保证；AeroVLA 逐步执行不需要持续 modify。base 的 modify 不像 activate 一样立即刷新距离，直到下一次状态回调才刷新；不可依赖快速 modify 后的距离立即对应新目标。
- [BehaviorServer](../as2/aerostack2/as2_behaviors/as2_behavior/include/as2_behavior/__impl/behavior_server__impl.hpp) 的取消调用 deactivate；位置插件返回成功，清理执行定时器并请求 Hover。该路径未显式对被取消 goal 调用 `canceled()`，也没有低速确认；接入时必须运行验证取消响应、原 goal 终态与实际停止，不能只等待 cancel 接受。
- 本次读取的 GoTo/BehaviorServer 没有单目标执行超时和 GoTo—FollowPath 跨节点资源锁；调用方/协调层须实现超时、排他控制及停止确认。
- 扩展 FollowPath 内部的原生入口与时间化入口互斥不覆盖 GoTo。控制模式丢失检测属于异常检测，不是提前拒绝冲突调用。

## 轨迹版 GoTo

[go_to_plugin_trajectory.cpp](../as2/aerostack2/as2_behaviors/as2_behaviors_motion/go_to_behavior/plugins/go_to_plugin_trajectory.cpp) 将单目标转成 GeneratePolynomialTrajectory Action，证明“GoTo 后接轨迹生成模块”已存在。
但本锁定版本 modify 明确未实现；deactivate 尝试异步取消下游却返回 false，且直接取得 goal future，不能认为取消链完整可靠。不建议本轮切换该插件。

## AeroVLA 映射建议与实现门槛

- 小转向位移动作：按观测快照与上游动作公式计算目标位置/yaw，使用 FIXED_YAW GoTo。明确这是目标位置控制，不能声称与上游恒速执行动力学相同。
- 大转向动作：保留先转向、再垂直移动、忽略水平位移的上游分支。原位置插件的 Action 成功不表示纯 yaw 完成；利用成功后的目标参考，在 Tool 侧等实测 yaw 收敛，再串联下一阶段。该执行假设尚待真实控制链验证。
- 每步完成：有效定位、位置与 yaw 达标、速度收敛并保持预声明时间，再完成上游悬停和新观测；两种等待的时间归属需明确，避免重复或漏掉。
- LAND：Tool 终止推理并编排原生 Land，检查结果与解除武装；不编码成单点轨迹。
- 先保持原生 Behavior 不变，在 Tool/Adapter 侧对齐；只有实际不能对齐时才向用户提出扩展方案，不预先新增插件或 Action。
- 当前 framework 以轨迹段为输入；改用 GoTo 需要显式支持单步目标/执行后端的生命周期与资源归属，不能让 Adapter 直接调用 GoTo 而旧 FollowTimedPath session 仍拥有控制资源。

实施前后验证矩阵：四向相对位移、正负升降、小转向、大转向先后顺序、纯转向、微小动作、位置已到但 yaw 未到、位置已到但速度未收敛、定位陈旧/丢失、目标拒绝、执行超时、取消时低速确认及 Action 终态、与其他运动 Behavior 冲突、LAND。适用用例需在真实 AirSim + PX4 SITL 链上验证；静态核查不代替上述验证。

## 2026-11-22 接口联测结果与边界

复用本地 `onboard_computer:v0.1.1`，补编译两组公共接口与 AeroVLA adapter，三个包构建通过。
`tests/check_ros_goto.py` 验证真实 AS2 消息类型、节点构造、参数，以及零四元数和执行中 frame 变化拒绝。
主机纯函数回归 66 项通过。

`tests/check_native_goto.py` 启动镜像内真实 `go_to_behavior_node` 和位置插件，输入合成的
FLYING、TF、Twist；控制模式服务是测试 stub，无控制器或物理仿真：

- 保持位置 (0,0,2)、实际 yaw=0，提交同位置 yaw=1 的目标，原生返回成功。
- 收到原生成功结束时发出的 yaw=1 位置参考。
- 再通过实际 `AerovlaGoToNode` 提交目标；原生成功后，Tool 仍处于 EXECUTING，步号不前进。
- 将合成实测 yaw 更新为1并持续提供新鲜状态；Tool 完成稳定等待及动作后保持，步号增加1。

这验证了“不改 Behavior，在 Tool 侧补充完成条件”的接口可行性，没有证明真实控制器消费最终参考后
能完成转向，也没有证明上游恒速动作与位置控制的动力学等价。镜像内二进制用于本轮运行验证，
静态分析的源码依据仍是上述 gitlink；本轮未重建原生 AS2 或独立认证镜像二进制对应的源码提交。
取消、拒绝、超时等完整 ROS 异常矩阵与真实 NPU、有屏 AirSim/PX4 SITL 仍未完成。

当前主机没有运行的 AirSim/Unreal/PX4 进程；历史 AeroVLA case runner 依赖未入库的 Windows/WSL
harness 和显式板端地址，不能直接作为本轮可运行入口。保留“适配验证中”，不修改 Registry 默认。

## 正式入口接通（2026-11-22）

用户接受后实施第1项：Registry 输出契约新增 `BehaviorExecution`，解析结果携带契约；
AeroVLA 正式 command 切换为 goto_node，既有轨迹组件保留原输出。
框架在此模式只管理 Tool 生命周期，不创建 FollowTimedPath、Gateway 或位置完成检测，
不要求虚构轨迹维持存活。Tool 以 task-scoped 心跳和 cancel/resume 服务反馈状态，暂停保留
进程，取消/失败先停再退出；停止未确认不能换任务。实际动作继续通过原生 GoTo。

真实 ROS 的 Registry 启动及暂停→确认→恢复→取消→确认退出联测通过；该测试无推理、无运动目标。
LAND 后续已按下节接入 Tool；共用观测/推理代码拆分仍为后续事项。未将接口通过提升为飞行验收。

## LAND 直接复用原生 Behavior

按用户裁决，降落与移动使用同一职责划分：Tool 解释策略输出并调用原生 Land 或 GoTo，
不再把 LAND 交给额外任务调用方。新增动作解析 guard，仅在动作后缀中识别独立 LAND 标记；
无标记时须有至少三个数字，继续沿用上游取最后三档、反量化和近零降落规则。
空输出或不足三个数字返回 INVALID_ACTION，不进入上游默认零动作的降落分支。
policy.py 保留上游数学基线，正式入口通过 actions.py guard 解释输出。

Land 使用 as2_msgs/action/Land，等待服务就绪、接受与结果，超时默认60秒；land_speed=0
交给 AS2 默认速度。只有 STATUS_SUCCEEDED 且 land_success=true 才报告 Tool SUCCEEDED，
框架收到该结果后完成任务并执行进程收口。拒绝、失败和超时进入失败/停止路径；
取消后的迟到成功不覆盖取消状态，仍需停止确认。

原生 Land 源码在成功后尝试 LANDED 事件及 disarm，但这些操作失败仅记录日志，
故 Action 成功不等价于独立确认 DISARMED；当前未额外声明解除武装验收。
ROS 合成 Land Action server 联测用于检查派发及成功/拒绝/失败传播，不替代原生插件或飞行验证。

## 共用观测与推理职责拆分

第3项已实施：新增 `AerovlaObservationNode` 管理观测、prompt、推理请求和共用任务状态。
GoTo 与旧轨迹入口成为两个独立子类。轨迹发布器、rolling 参数及 ref_handoff 只在旧入口中
创建；GoTo 继承链不再经过旧轨迹入口。共用初始化完成后才配置执行资源、启动框架任务，
避免构造过程中先发布未初始化的执行状态。

验证：构造检查确认 GoTo 无 trajectory publisher/ref_handoff/rolling 参数，旧轨迹入口仍可
构造；原生 GoTo 合成状态联测、合成 Land server、Registry 子进程生命周期联测均通过。
79项主机测试通过，4项ROS回调与42项框架测试在Humble环境全部通过。闭环继续暂缓。

## 残留清理（2026-11-22）

当前 Registry defaults/schema 和 Demo task 移除 `trajectory_timeout`、
`valid_from_lead_s`；它们仅属于历史轨迹入口。GoTo 与旧轨迹节点共享观测基类，
共用观测/推理拆分已完成。以上阶段记录中的待办描述保留其当时状态。
Demo 客户端改用 Tool backend、GoTo 状态和逐机停止确认；自动 Land 成功时跳过
重复 Land，失败恢复须等待停止确认。Windows/WSL harness 仍是本地依赖，详见
`deployment/airsim/demos/aerovla_blocks/README.md`。旧验收文件仅为历史证据，
当前飞行验收待后续执行。本轮未修改原生 Behavior。
