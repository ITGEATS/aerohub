# Target Tracking Tool

默认只观测的文本查询目标跟踪：`TrackObject` → 串行 `DetectSnapshot`
（`estimate_position=true`）→ 锁定 YOPO-Tracker EKF → Action 状态反馈。
`follow=false` 不调用导航；`follow=true` 经 Navigation Tool 请求保持自身位置并转向人物。
两种模式均不直接调用 Behavior、运动参考或平台服务，不发布自身状态/TF。

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


## Follow 选项（接口0.3.0）

```python
drone.target_tracking.track(query='person', timeout=20., follow=False)  # 默认只观测
drone.target_tracking.track(query='person', timeout=20., follow=True)   # 请求转向人物
```

`follow` 是严格bool，只表示是否转向，不表示追逐或平移跟随。开启后，Tracking等待
有效目标与同固定坐标系的新鲜自身位置和飞行状态，冻结首次请求时自身xyz，
根据目标水平位置计算yaw；通过Navigation的TARGET_YAW＋HOLD请求，最多5Hz串行更新。
没有有效目标时不提交运动，原有有限观察窗口到期TIMEOUT不代表已看到或已转向。

Navigation独占运动资源；不能同时另起一个Navigation位置任务。今后要边飞边转向，
须合并到同一个Navigation任务，不能两个入口分别争抢位置/yaw。
目标失效、取消、超时或失败时取消自有Navigation并等待stop_confirmed；
停止未知返回STOP_UNCONFIRMED并阻止本Tracking进程再次开启follow，不能自行抢占恢复。
最终Action终态在Navigation清理后返回，Result新增stop_confirmed。

**SUPER现可显式启用TARGET_YAW末端约束，但这不代表保持原位连续转向已支持；YOPO仍不支持。默认关闭或后端不支持时明确失败，不静默忽略。**
此提交完成选项、Tool调用和生命周期，尚未完成SUPER转向执行和仿真转向验收。
真实ROS替身测试通过不代表当前SUPER已能转向。应先使用follow=false完成观测。

Action schema有变化，所有Tracking接口/服务端/客户端必须加载同一新overlay。
独立构建和回归：

```bash
bash deployment/target_tracking/build_follow_overlay.sh
bash deployment/target_tracking/test_follow_overlay.sh
```

新overlay默认在log/target-tracking-follow/workspace，旧query overlay只读复用；
部署先加载旧query overlay再加载新follow overlay，TRACKING_WORKSPACE指向后者。
详细证据见deployment/target_tracking/VALIDATION-follow.md。

### 测量来源的不确定性

ROS参数 `measurement_stddev_m` 默认0（保留锁定上游Rt=0.02m²）；显式有限正值
在Adapter内设置各轴Rt=stddev²，不修改上游、不改变失跟时限和关联硬上限。
值必须来自测量源分析，不应为某次轨迹通过随意扩大。各向同性只是当前显式先验，
不是已校准概率保证；多测量来源混用时需要进一步传递每观测协方差。

WestStreet known-height部署显式1m，依据之前独立身高测距约0.6–0.9m误差；
默认深度路径不变。真实录制位置fixture说明原模型在跑姿跳变后失跟，按来源配置
噪声的回放可保持有效，且过期仍拒绝；回放不等价于新一轮实时飞行成功。

显式 `association_policy:=innovation_covariance_v1` 使用创新协方差
`Sigma_pp + Rt` 的三维Mahalanobis关联（名义chi-square阈值11.34486673），同时
保持上游`gate_max=3m`硬距离上限。需要显式正`measurement_stddev_m`，默认仍为
`upstream`策略。它改变的是关联模型，不表示原门曾通过，也不是身份/遮挡保证。
`tests/fixtures/weststreet_height_positions*.json` 来自独立真实失败轮；回放与实时
验收分别记录，禁止通过调大噪声或改变失跟时限把旧失败轮改判通过。


公开只读目标状态流为 `<UAV>/tools/target_tracking/state`（`TargetState`），语义与Action
反馈一致，包括valid、observation_stamp、valid_until、session/generation；会话结束发布
无效状态。平台相机指向可按部署显式绑定此流，不能把场景Actor身份/真值作为替代。
相机指向不占无人机运动控制；现有 `follow=True` 的原地机头朝向语义不因此改变。

### 短窗平均与跳点过滤（显式启用）

`position_filter:=robust_linear_v1`在Adapter进入EKF前处理固定系三维位置，默认none
不改变旧路径。最多5个原始采样、0.8s历史，以稳健局部线性趋势将样本对齐到当前
原始采样时刻，再对残差做限幅平均。匀速5m/s测试保持当前值，不引入普通后向平均
的固有滞后；真实加减速仍可能延迟，不能据测试宣称任意运动无延迟。

至少3个历史点后，偏离预测>=3m的孤立测量被拒绝，作为本次无观测送给原EKF；
不会重复上一个数值或刷新最后有效观测时间。无检测不生成值；多候选不互相平均，
直接交给原关联，不加入平滑历史；保留仍在时间窗口内的历史，避免偶发额外候选
导致重新预热并误判运动方向。长间隔/新会话清空历史，frame改变仍拒绝。
此单候选处理不提供身份/遮挡保证，Rt保留原声明值，不因平滑自动缩小协方差。

诊断流`tools/target_tracking/measurement_filter`为String JSON，包含原始stamp/frame、
raw/filtered数组和过滤状态；原检测calls记录仍原样保留。真实回放中两段维持有效，
但已有漏检/输入间隔失败段没有被滤波“救活”；时间门不变，不能将缺观测伪装成功。

### 最新结果消费模式

`observation_mode:=stream`消费既有三维检测topic，不调用DetectSnapshot；默认仍为
snapshot。`stream_query`必须与部署检测器固定query一致，不同任务query拒绝。
订阅回调仅更新容量1结果槽，任务循环取新序号的最新帧：无新结果不重复推进EKF。
原始采样新鲜度、墙钟接收年龄、session开始时间和clock epoch均校验；生产者
(tool/worker/model)世代变化结束会话。生产停止仍按原0.5s有效期失效。

采集与推理是两个逻辑职责，ROS执行器另有通信线程；不宣称整个进程只有两个线程。
流式下游仍需检查frame.valid/位置有效性，最新buffer不解决上游无图、测距拒绝或
模型漏检。test_stream_ros.py在无快照服务环境验证实际DDS消费、失效及世代隔离。

### 有噪声观测的软更新

显式`association_policy:=soft_innovation_v1`配合`measurement_stddev_m:=1.0`、
`position_filter:=none`，将平滑与鲁棒观测更新合并到同一个EKF，避免前置趋势门和
跟踪关联门各自拒绝。默认upstream和已有innovation_covariance_v1行为不变。

用创新协方差归一化残差，Huber权重`min(1,2.5/sqrt(score))`通过增大本帧测量
协方差降低影响，而不是将几米残差直接判为无观测；再缩放Kalman增益，把位置
修正限制到`target_max_speed*dt`、速度修正限制到
`target_max_speed/max_blind_time*dt`。使用同一缩放增益的Joseph形式更新协方差。
这些是单帧修正上限，不是绝对位置误差或飞行速度保证；噪声参数仍为模型假设。

保留几何可达保护：`gate_max + target_max_speed*max_blind_time`，当前为8m，
超出不作为当前轨迹的有效更新；多候选归一化残差分数相差<1时不强选。
该策略不提供外观身份保证，长期偏置或相近人物仍需独立验证。不新建远处候选
重捕获通道；沿用上游失跟后的确认流程。无观测、过期帧、错误坐标/时钟和世代
不被伪造为新测量，0.5秒真实观测失效预算保持。

诊断JSON的`association`记录残差、可达范围、权重、增益缩放与单帧修正大小。
两段真实失败数据新增回归覆盖持续有噪声运动，另验单帧跳点、可控范围连续
修正、远处异常、歧义候选、协方差数值与实际缺测停止；回放通过不替代飞行。

### 可选预测恢复窗口

参数`max_prediction_time`默认0.5秒，显式可设0.5..2秒；只延长已有EKF
短时预测/有效期，不延长输入图像新鲜度，不伪造新观测时间。
soft_innovation_v1关联reach与单帧速度修正继续使用原模型blind预算，
避免放宽恢复时间同时偷偷扩大关联门。1秒候选已覆盖真实0.804秒缺测
回放；这不是持续实飞通过证据。任务方必须消费valid_until并声明预测
期间的运动策略，不能把预测当成真实测量。
