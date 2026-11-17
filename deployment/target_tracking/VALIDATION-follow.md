# Follow选项验证（2026-11-16）

follow=false默认只观测；true请求转向人物，冻结自身xyz，不追逐。新增Result
stop_confirmed。Tracking只调用Navigation公开Action/更新服务，不直接控制Behavior。

独立overlay构建通过，复用固定onboard_computer:v0.1.1和只读query overlay，
不修改共享install或镜像。首次构建因当前colcon不支持allow-overriding参数失败，
删除该兼容性参数后构建通过。原始证据在log/target-tracking-follow/。

已执行25项：10项朝向/任务生命周期单元测试、6项原EKF核心测试、5项原Tracking
真实ROS回归、4项新增follow真实ROS传输测试。全部通过。新增ROS测试使用合成目标、
自身状态和Navigation替身，验证默认不依赖导航、导航缺失、固定自身位置＋yaw更新、
停止确认和UNSUPPORTED传播；不是实际飞行或真实SUPER导航证据。

当前真实SUPER和YOPO均拒绝TARGET_YAW。follow=true尚不能在这些后端完成转向；
接口存在不代表执行能力已补齐。未执行本版GPU感知消费/ProjectAirSim＋PX4转向、
动态人物闭环；此前身高估距地面证据不提升为本版follow验收。
