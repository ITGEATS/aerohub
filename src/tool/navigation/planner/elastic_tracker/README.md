# Elastic-Tracker ROS 2 Humble

Elastic-Tracker 是单机滚动目标跟随 Planner。组件目录下的 `upstream/` 是唯一的上游参考子模块，指向官方 [ZJU-FAST-Lab/Elastic-Tracker](https://github.com/ZJU-FAST-Lab/Elastic-Tracker)，锁定在 `0a302a2a9cc8b733e74941fdd4af3fb449447bea`，用于源码与算法对照。Humble 构建使用 AeroHub 的 GPL-3.0 ROS 2 移植源码，位于 `ros2_adapter/native/port/`。

工作站 Mission 通过 `FollowObject` 请求跟随。算力侧 Following 调用共享 TrackObject，仅消费本次 Goal 的反馈，并将目标状态交给 Elastic Planner。RGB-D 观测由共享 Object Detection 提供。Elastic 转换完整的时间化轨迹，首次发 FollowTimedPath Goal，后续对同一任务发 `modify`。只有 FollowTimedPath 发布运动参考。

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

ros2 action send_goal /drone0/tools/following/follow \
  uav_planner_interfaces/action/FollowObject \
  '{selection_mode: 0, query: person, timeout: 120.0}' --feedback
```

任务侧在发送 Goal 前完成原生起飞与悬停。Following 在有效观测和车辆输入就绪后启用规划。超时或取消时，Following 确认运动停止后再请求取消自己启动的观测会话。停止无法确认时返回失败及 `stop_confirmed=false`。任务侧负责 Land 和 PX4 状态确认。`elastic_adapter/enable` 与 `elastic_adapter/state` 是组件内部控制和诊断接口。

公共 `FollowObject` Action 支持两种选择方式。Elastic 接受 `QUERY=0`，要求非空查询。YOPO-Tracker 接受 `AUTO=1`，要求查询为空。具体实现由部署选择，同一车辆只启动一个公共 Action 服务端。已有 YOPO-Tracker 部署可使用 [YOPO Following 适配入口](../../../../../deployment/yopo_tracker_humble/README-following.md)。

Following 公共入口与 System、SUPER Navigation 共用 `uav_motion_resource`。算力进程通过 `AEROHUB_MOTION_LOCK_DIR` 指向同一持久目录，跨容器使用同一宿主目录。Following 接单时获取车辆运动锁，确认停止后释放。托管任务通过 Following Action 发起运动，内部 enable 服务用于组件集成与诊断。

已部署同一车辆的 Object Detection 和 TrackObject 时，启动 Planner 的 `stack.launch.py`，再用 `ros2 launch uav_elastic_tracker_adapter following.launch.py namespace:=drone0` 接入共享服务。上面的 `tracking_object.launch.py` 是独立复现入口，负责一起启动观测节点和 Following。两种入口择一，同一车辆只运行一个 Following 实例。使用共享观测部署时，查询范围、RGB-D/TF 和时间门限由该部署配置决定。

## 时间门限

- Object Detection 要求目标区域内至少 20 个有效深度点才能计算三维位置。
- `max_capture_age` 判断三维量测从采集到滤波器消费时有多旧。`max_prediction_time` 控制从最后一次关联观测的采集时刻起，目标状态能维持有效多久。两项默认均为 0.5 秒。更长的外推会推迟目标丢失判定。
- Adapter 的 `target_timeout=0.5` 单独判断最新目标里程计多久没更新。增大滤波时限并不会更改这个门限，也不会提高取图或推理吞吐。

### 本机 Blocks SITL 启动配置

本机同时运行有屏 AirSim 和 CUDA YOLO 时，取图、推理及三维定位合计的量测到达延迟常接近或超过默认的 0.5 秒。第 22 轮在这台机器上使用 **1.0 秒量测时限、1.5 秒外推上限**，减少慢帧和短暂缺测导致的停追。该组数值是本机 SITL 启动覆盖值。

将 [Blocks 相机配置片段](config/blocks-airsim-camera-320.json) 中的 `SceneJpegQuality` 和 `Vehicles.PX4.Cameras.FrontCamera` 合入场景设置。RGB 与 `DepthPlanar` 均为 320×240、水平 110°，安装和内参对应 [Planner 相机配置](config/blocks-camera-320.yaml)。YOLO 的模型输入仍为 640。

```bash
ros2 launch uav_elastic_tracker_adapter tracking_object.launch.py \
  detector_config:="$PWD/src/tool/navigation/planner/elastic_tracker/config/detector-rgbd.yaml" \
  max_capture_age:=1.0 max_prediction_time:=1.5

ros2 launch uav_elastic_tracker_adapter stack.launch.py \
  camera_config:="$PWD/src/tool/navigation/planner/elastic_tracker/config/blocks-camera-320.yaml" \
  depth_topic:=/drone0/sensor_measurements/FrontCamera/depth_planar/image \
  target_mask:=true fresh_native_output:=true
```

Elastic 桥支持 AeroHub 现有的字符串状态格式和新版 Goal UUID／数字状态格式。新版还要求 `velocity_valid` 才发布目标里程计。ROS 消息类型、发布者与订阅者须从同一套源码构建并加载。

### 滚动更新与计时

单机直连时，FollowTimedPath 明确拒绝一版 `modify` 后，Elastic 丢弃候选，从上一条已接纳轨迹继续规划。目标或里程计失效、旧轨迹覆盖不足、应答超时均进入已有停止流程。`elastic_adapter/state` 报告已发送与已接纳的 revision，以及被拒次数。

同机诊断可在视觉和 Elastic 进程各自设置 `ELASTIC_TIMING_TRACE`。按 `stamp_ns` 和 `mono_ns` 关联目标状态桥发布、输入节点接收／转发、Adapter 接纳，以及深度编码和 RPC 耗时。

## 验证结果

以下记录来自 Following 入口接入前的 Planner 运行链，Following Action 需单独验证。

使用 `tests/verify.sh` 在只允许本机回环的 Humble 容器内执行源码构建、接口检查、原生算法回放和标准 FollowTimedPath 消费验证。交付工作区基于 AeroHub `dev` 的 `c3e541a`，在固定 Humble 开发镜像构建 17 个包，54 项单元及契约测试通过。合成输入驱动真实 Planner 和标准 FollowTimedPath，接纳 87 版并输出 217 条参考，零迟到更新，停止获确认。合成回归使用测试传感器与控制器。

本次更新使用两套各自一致的 Humble 接口验证目标桥，并复测执行客户端、滚动接续和原生直连。27 项检查通过，旧消息环境单独跳过一项新版速度有效性检查，该项在新版环境通过。

历史 Blocks AirSim + PX4 SITL 走实时视觉和标准执行路径。第 16 轮使用两个 0.5 秒默认值，31 帧有效三维量测中 30 帧到达时已超过 0.5 秒，未启用规划。第 19 轮使用 1.0/1.5 秒后短暂产生有效状态，任务初始化晚于人物起步，未启用规划。

第 21 轮先悬停约 29 个仿真秒，人物离开起点约 0.25 米才启用规划。场景真值仅用于触发试验，目标位置来自真实 RGB-D。0.88 个仿真秒后执行首条参考，共 88 条、覆盖 13.09 个仿真秒。量测中断前出现推理超时、服务忙和深度点不足；目标里程计停更 0.516 秒触发 Adapter 保护。任务正常降落并确认解除武装，无碰撞记录。以上是迁移工作区的历史飞行证据；本提交的验证状态仍为 **partial**，持续移动目标跟踪、交付镜像和真机尚待验证。

第 22 轮使用本机工作区重新构建的 Tracking 和 Object Detection。320×240 与上一轮 640×480 的取图频率分别为 7.89 Hz 和 3.12 Hz，取图 RPC 中位耗时为 97 ms 和 227 ms，有效三维量测到达延迟中位数为 441 ms 和 585 ms。两轮使用的感知版本有差异，数据用于整链路对照。

第 22 轮完成了 30 秒跟随窗口，173 条 FollowTimedPath 参考覆盖 30.08 个仿真秒。目标里程计更新最长空档约 288 ms，未触发 500 ms 停追门限；正常降落并解除武装，碰撞记录为零。此次无滚动拒绝，拒绝后的恢复由离线回归覆盖。本机原始证据位于 `log/elastic-pure-ros2/blocks-turn-22/`，不同工作区和硬件上的闭环效果按各自运行结果记录。
