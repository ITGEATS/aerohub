# Elastic-Tracker ROS 2 Humble

Elastic-Tracker 是单机滚动目标跟随 Planner。原始 [Elastic-Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker) 通过 `upstream/` 固定在 `0a302a2a9cc8b733e74941fdd4af3fb449447bea`，用于源码与算法对照。AeroHub 维护的 GPL-3.0 ROS 2 移植位于 `ros2_adapter/native/port/`，ROS 2 包直接编译这些源码。

RGB-D → 共享 Object Detection → 共享 TrackObject → `elastic_tracking_bridge` → Elastic Planner → 标准 FollowTimedPath → AS2 控制器与平台。TrackObject 使用 `follow=false` 提供观测；Elastic 转换完整的时间化轨迹，首次发 Action Goal，后续对同一任务发 `modify`。只有 FollowTimedPath 发布运动参考。

## 构建和运行

使用 [组件清单](../../../../../deployment/elastic_tracker_humble/component.yaml) 指定的开发环境。检出仓库后准备锁定的 YOPO-Tracker EKF 源码，在 Humble 算力环境编译同一套接口及消费者。

```bash
git submodule update --init deployment/airsim/demos/target_tracking/reference/yopo_tracker
bash deployment/elastic_tracker_humble/build.sh "$PWD" "$PWD/log/elastic-tracker/build"
source log/elastic-tracker/build/install/setup.bash
```

先按 [共享推理组件](../../../../../deployment/inference/yolo26n_seg.component.yaml) 启动 CUDA YOLO26n-seg worker 和 ROS facade。复用已部署的 AS2／FollowTimedPath、同域时钟、车辆里程计、RGB-D 和采集时刻 TF；相机请使用配套的内外参。以下命令在各自终端运行。

```bash
ros2 launch uav_elastic_tracker_adapter tracking_object.launch.py \
  detector_config:="$PWD/src/tool/navigation/planner/elastic_tracker/config/detector-rgbd.yaml"

ros2 launch uav_elastic_tracker_adapter stack.launch.py \
  camera_config:=/absolute/path/to/calibrated-camera.yaml \
  depth_topic:=/drone0/sensor_measurements/FrontCamera/depth_planar/image \
  target_mask:=true fresh_native_output:=true

ros2 action send_goal /drone0/tools/target_tracking/track \
  uav_target_tracking_interfaces/action/TrackObject \
  '{query: person, timeout: 120.0, follow: false}' --feedback
```

任务侧完成起飞与悬停，确认 Action 有效状态和 `/drone0/elastic_adapter/state` 的 `input_ready` 同时成立，再调用 `/drone0/elastic_adapter/enable`（`std_srvs/srv/SetBool`）启用规划。结束时先禁用并等待 `stop_confirmed`，再取消已拥有的 TrackObject 会话。Land 动作和 PX4 状态由任务侧确认。

## 时间门限

- Object Detection 要求目标区域内至少 20 个有效深度点才能计算三维位置。
- `max_capture_age` 判断原始观测到达时有多旧，`max_prediction_time` 控制最后一次关联量测之后的状态有效期，默认各 0.5 秒。可显式传 `max_capture_age:=1.0 max_prediction_time:=1.5` 做仿真对照。更长的外推会推迟目标丢失判定。
- Adapter 的 `target_timeout=0.5` 单独判断最新目标里程计多久没更新。增大滤波时限并不会更改这个门限，也不会提高取图或推理吞吐。

## 验证结果

使用 `tests/verify.sh` 在只允许本机回环的 Humble 容器内执行源码构建、接口检查、原生算法回放和标准 FollowTimedPath 消费验证。交付工作区基于 AeroHub `dev` 的 `c3e541a`，在固定 Humble 开发镜像构建 17 个包，54 项单元及契约测试通过。合成输入驱动真实 Planner 和标准 FollowTimedPath，接纳 87 版并输出 217 条参考，零迟到更新，停止获确认。合成回归使用测试传感器与控制器。

历史 Blocks AirSim + PX4 SITL 走实时视觉和标准执行路径。第 16 轮使用两个 0.5 秒默认值，31 帧有效三维量测中 30 帧到达时已超过 0.5 秒，未启用规划。第 19 轮使用 1.0/1.5 秒后短暂产生有效状态，任务初始化晚于人物起步，未启用规划。

第 21 轮先悬停约 29 个仿真秒，人物离开起点约 0.25 米才启用规划。场景真值仅用于触发试验，目标位置来自真实 RGB-D。0.88 个仿真秒后执行首条参考，共 88 条、覆盖 13.09 个仿真秒。量测中断前出现推理超时、服务忙和深度点不足；目标里程计停更 0.516 秒触发 Adapter 保护。任务正常降落并确认解除武装，无碰撞记录。以上是迁移工作区的历史飞行证据；本提交的验证状态仍为 **partial**，持续移动目标跟踪、交付镜像和真机尚待验证。
