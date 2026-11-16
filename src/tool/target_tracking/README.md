# Target Tracking Tool

独立、只观测的文本查询目标跟踪：`TrackObject` → 串行 `DetectSnapshot`
（`estimate_position=true`）→ 锁定 YOPO-Tracker EKF → Action 状态反馈。
不调用导航、Behavior、运动参考或 Mission，不发布自身状态/TF。

源码分为 `interfaces/`、`ros2_adapter/`、`python_api/`。原算法与 MIT 许可证直接从
已锁定参考子模块安装到 `share/uav_target_tracking/upstream/`，不改动、不复制 Fork。
原算法关联最近的门内目标，保留 M/N 确认、可达域重捕获和短时预测；没有外观身份或
跨遮挡身份保证。Qt/Rt 是上游模型假设，不是检测器报告的测量协方差。

## 数据与时间契约

- Goal：非空 `query`、有限正 `timeout`（墙钟秒）；query 的支持范围由部署的检测后端决定。
- 地址：`tools/target_tracking/track`，相对车辆命名空间。每进程只接纳一个会话。
- Feedback：`TargetState`，ENU 米/米每秒，frame 必须来自固定世界观测系；首个 frame
  固定本会话，后续变化拒绝，不通过更名进行坐标变换。配置检测器输出为车辆 odom。
- `header.stamp` 为状态估计/预测时刻；`observation_stamp` 为最后接纳的目标位置采样时刻，
  `valid_until` 为其后0.5秒。`valid=false` 时位置/速度/协方差不可消费。
  协方差为行优先6×6，顺序 `[px,py,pz,vx,vy,vz]`。
- `session_id` 为每次 Goal 的 UUID；`generation` 为进程内会话世代；`capture_id` 为
  本会话实际接纳观测所属的请求序号（允许跳号），不是相机/检测器的全局身份。
- `confirm`：未完成 M/N 确认；`tracked`：最近观测关联成功且仍有效（帧间保持该状态）；`predicted`：
  最近观测缺失或请求不可用，已确认后仍在盲区预算内预测；`lost`：超过盲区；`unavailable`：尚无可接纳三维采样。
- 间隔使用原始 depth/header 采样时间差，更新原 EKF 的 dt、A 和 B 后调用原方法。
  首样本无前驱，沿用上游0.1秒初始化步长。反馈外推不修改 EKF 的观测状态。
- 拒绝非有限位置、未来、陈旧、重复/倒序采样和换 frame；不把 now 盖到旧观测上。
  默认观测接收年龄≤0.5秒。ROS 时钟倒退使会话 ABORTED/CLOCK_RESET，必须新建 Goal。
  暂停 ROS clock 不暂停墙钟超时；状态有效期属于 ROS 时间域。

## 生命周期与客户端

部署先启动 Object Detection 及其 Inference/GPU、标定、深度和历史 TF。
本 Tool 的 Action 可发现只代表服务就绪，当前目标就绪必须查看 `state.valid`。
检测不可用时保持反馈并在有效期后失效，不制造位置。无目标的正常观测进入原算法失跟逻辑。
已发请求保持串行占用，取消/超时/时钟重置后的晚回复只排空，不能污染新会话。
若服务永不回复，本进程不再发第二个请求，后续会话反馈不可用；重启恢复请求通道。

`timeout` 到期以 Action SUCCEEDED / `code=TIMEOUT` 表示完成有限观测窗口，
不表示已确认目标。标准取消为 CANCELED；异常/重置为 ABORTED。
最终 Result 的 state 强制无效；消费者只消费活跃会话的有效反馈。

在持续 spinning 的 AS2 DroneInterface 上加载模块：

```python
# AS2_MODULES_PATH 指向 install/uav_target_tracking_python_api/share/
# uav_target_tracking_python_api/modules（与已有模块路径一起配置）。
drone.load_module('target_tracking')
goal_future = drone.target_tracking.track('car', timeout=10., feedback_callback=on_feedback)
# 待 Future 完成并确认 accepted，再保存 handle。
handle = goal_future.result()
result_future = handle.get_result_async()
cancel_future = handle.cancel_goal_async()
```

`track()` 返回标准 Action Goal Future，不阻塞调用线程；调用方管理结果及取消 Future。
共享检测器忙时仍受其原互斥/推理 worker 约束，不代表通用 Tool 仲裁已经实现。

构建、运行、测试和未完成项见 [部署说明](../../../deployment/target_tracking/README.md)。

## Query schema 0.2.0

`TrackObject.Goal.query` and `drone.target_tracking.track(query='car', timeout=10.)`
forward the same query to each fresh `DetectSnapshot` request. No public category
field remains. CLI uses `--query` (default `car`); deployment selects the detector.
DINO phrases need not equal query; Tracking consumes only `position_valid=true`
positions and applies the unchanged EKF association/loss policy. A valid box alone
cannot initialize a 3D target. Box-based positions are visible surface measurements,
not object centroids; this difference matters for association and accuracy.

Build all interfaces and consumers together with
`bash deployment/object_detection/build_query_overlay.sh`, then load that overlay
in detection, inference, tracking and clients. Current query regressions and saved
CUDA replay are recorded in `deployment/object_detection/VALIDATION-query.md`.
Live DINO tracking, identity/accuracy and flight remain unverified (partial).
