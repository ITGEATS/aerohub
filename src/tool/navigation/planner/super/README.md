# SUPER 的时间化 Behavior 接入

实际构建源码直接保存在本仓库 `ros2_adapter/native/`，AS2 适配包位于
`ros2_adapter/bridge/`；`upstream/` submodule 只作上游对照，不是构建输入。
原生输入、优化与主/备用轨迹计算不搬入 Behavior。当前验证对象是 PC forest8，
不表示 ROS/OHOS 算法部署或 RoboPi 闭环已完成。

## 执行链与接口

`super_planner` 输出 `/planning_cmd/poly_traj`（PolynomialTrajectory）。
`uav_super_adapter/super_timed_path_adapter` 将完整 committed 轨迹转换成
TimedTrajectorySegment，并复用机群客户端调用逐机 FollowTimedPath；
Adapter 不发布 motion_reference，不与旧 SUPER 单点桥同时启用。

SUPER 的 CmdTraj 在原生内部拼接主轨迹前缀和备用制动轨迹，导出的 committed
多项式已经包含该拼接结果。适配保留所有原生分段边界与整段未来覆盖，
不只截取主轨迹或终点。PositionCommand 的 flag=1/2 用于源状态和主/备用观测，
不会作为另一个运动参考源发布。单纯 HEART_BEAT 不重建或延长已缓存计划。

本任务选择固定接管航向，与旧 SUPER adapter 默认保持一致；没有实现原生
Yaw polynomial 约束适配，不宣称支持要求跟随航向轨迹的任务。

## 时间、坐标与结束

- 源使用固定局部 ENU `drone0/odom`。接口转换到统一局部 NED，再由既有客户端
  转回逐机 ENU；原点为零，不重复减去任务平移。frame/vehicle/namespace 须匹配。
- 保留源 generated_at 和原多项式执行 epoch，只裁掉已过去的前缀。
  默认在当前时刻后100ms执行原曲线上的对应后缀，不将曲线整体推迟。
- 原生七次曲线每20ms解析采样 PVA，并显式保留每个分段边界；Behavior 在节点间
  使用五次 Hermite 插值。它是近似表示，测试误差不是所有输入的统一误差上界。
- 过期或不足未来窗口的更新丢弃，既有计划的结束时间与过期保护不变。
  不允许给旧轨迹重盖时间戳充当新轨迹。
- 终点在任务 goal_tolerance 内且终端速度<=0.3m/s时可标记最终段；Behavior
  仍按自身0.25m误差及超时判断执行结束。任务还须检查Action成功、实际到点保持。
- 新鲜、递增的原生命令才续源健康状态；重复、错误frame/非法PVA不续期。
  执行中500ms没有新源命令则关闭执行；初次规划预算10s。
- `super_adapter/enable`（SetBool）是应用接管/退出入口。停用后需等待
  `super_adapter/state` 的 stop_confirmed，不能把服务响应直接视作飞机已停止。
- `super_adapter/state`（String JSON）包含任务、revision、complete、error、
  source_flags、late_updates、源曲线元数据与停止确认。曲线元数据是最近解析值，
  不是每个版本已接受的证明；结果与取消以Action和停止反馈为准。

## 构建与验证

完整PC场景入口见 `deployment/airsim/demos/super_forest/run.py`。镜像基于现有 onboard_computer，
原生算法在现有 ROS Humble desktop 镜像离线编译，运行层携带所需非ROS动态依赖。

数学测试可直接运行；真实ROS消息转换、重复/过期样本、备用命令及节点构造
须在新镜像中验证。C++ interpolation_test.cpp 使用实际Behavior采样器对照
七次测试曲线。真实Behavior终点模式及外部接管测试位于
`src/behavior/follow_path/tests/terminal_mode_test.py`。


## 公共导航调用层

PC Demo 通过 AS2 `NavigationModule` 调用统一 `NavigateToPose` Action，地址为
`/drone0/navigate_to_pose`。SUPER Adapter 在 `navigation_action:=true` 时提供该入口，
内部继续配置目标、调用原目标 Topic 并复用现有执行链；规划核心和多项式转换不变。

此模式下 `super_adapter/enable` 拒绝外部请求，目标参数也不能在活跃 Action 中修改。
`super_adapter/state` 继续作为诊断与 Demo 独立观测，不作为公共调用协议。
开关默认 false，保留旧部署的调用方式；这不代表板端已验证新接口。

公共 Python API、约束和结果语义见 [Navigation Module](../../python_api/README.md)。
PC 构建、运行与回归入口见 Demo README 的“公共导航接口 overlay”。

### 终点覆盖结束后的新目标

滚动替换必须与旧计划在时间上重叠；旧final覆盖已结束、新目标的计划晚于其结束
时刻时，不能继续修改旧FollowTimedPath Action。Adapter记录最后提交计划的原覆盖
终点，HOLD任务在该情形走既有停止确认→重新接管，重新请求SUPER计划；不改动
原生时间戳或填造过渡轨迹。交接期间接收并缓存拥有者的最新目标，停止确认前不
启动新执行；取消/终止保护仍有效。非final或非托管计划的覆盖空洞仍报错。
`super_adapter/state`中的`submitted_segment`与`coverage_handoffs`记录这些区别。
原失败时间35.153291604→35.281s的回归位于bridge/tests/test_dispatch.py，
完整实测与覆盖边界见WestStreet VALIDATION-tool-demo.md。

已成功完成的执行自动路由：同一Navigation任务仍有效时，新目标到来若发现旧
FollowTimedPath已成功完成，Adapter不再向旧Action发modify，而是先走停止确认、
再请求新计划并提交新Action。新计划排队期间旧Action完成也走该路由。
外层COMPLETE任务尚未终止时同样支持；目标未变只续期，取消/失败不自动重启。
外层Goal和运动资源所有权不变，交接期间只缓存最新目标，不并发启动执行。

### 可选目标yaw接入

Adapter参数`target_yaw_enabled`默认false；默认UNCONSTRAINED/KEEP_YAW继续
保持执行接管朝向。需要目标yaw时，在部署前同时启用SUPER的
`fsm.click_yaw_en`、`super_planner.goal_yaw_en`与Adapter参数，调用Navigation
`to_pose(target, orientation_mode=NavigateToPose.Goal.TARGET_YAW, ...)`，
target提供有限、归一化的纯yaw四元数。不需要yaw的调用者无需填充朝向。
不支持TARGET_ORIENTATION全姿态控制。

原始yaw多项式按其独立start_wt_yaw和time_yaw采样，与位置曲线共同覆盖
当前提交区间；缺失/非法/覆盖不足报错，不静默丢弃yaw或外推。经fleet NED
转换后交给现有FollowTimedPath，不另发运动参考；只更新朝向也触发目标发布。
目标yaw是规划目标约束，不保证整段持续盯住运动目标。

WestStreet部署入口`run_tool_demo.py --target-yaw`统一开启两侧部署配置，
但不自动更改Demo任务调用：当前任务仍to_point、不下达yaw，不能将开关
开启当成已完成机头追人。安装后的Adapter23项回归通过，已进行真实转弯闭环，任务失败：目标yaw44.9度时执行参考仅3.4度，
实际1.81度；接口贯通不等于及时视线跟踪。详见WestStreet VALIDATION。
