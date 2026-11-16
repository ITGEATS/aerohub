# Target Tracking 部署

<a id="dino-default"></a>

## 当前 Demo 默认：Grounding DINO / car（尚未完成平台验收）

本 Demo 固定选择 `grounding_dino`、`query=car`；链路为 Mission → Tracking →
DetectSnapshot → InferGroundingDino。worker 为 `grounding-dino-tiny`，HTTP 8003。
这只是 target_tracking Demo 的替代后端选择，共享 Object Detection Harness 默认仍为 YOLO。
参数统一使用 `deployment/target_tracking/params.yaml`，显式 endpoint、backend、query，
box=.30 / text=.25，无 fallback。框无 mask；`position_valid=false` 不得用于跟踪初始化。
竞争深度拒绝规则保持不变。HTTP/Engine/Detect 上限10s，Tracking 单次 snapshot 上限1s、
采样年龄0.5s；较长推理可能被跟踪拒绝，不扩大 freshness 换取成功。
Mission max-age=1s，初始等待默认15s（Mountains观察10s）、静态门12s、移动窗12s，
均保留现有预算；这些上限不是实时性能保证。

统一构建并加载 `log/object-detection-query/workspace/install/setup.bash`，不叠加旧
`log/yopov2-tracking/build` 或 `log/object-detection-tool/workspace`。固定 compute 镜像为
`onboard_computer:v0.1.1`；已有 flight_board 不变。独立部署时先启动8003 CUDA worker、
`ros2 run uav_inference grounding_dino_node`、Detect、Tracking，
所有 ROS 节点使用同一 overlay、命名空间、时钟和该参数文件。
Mountains入口复用匹配且CUDA健康的8003 worker；端口空闲才启动自己的DINO worker，
最多等待120s，只清理本轮启动的资源。
宿主需 msgpack，独立相机审计解释器需 NumPy/OpenCV/Pillow；可用
`CAMERA_PROBE_PYTHON=/path/to/prepared/python` 指定，不调用该环境中的模型。静态地面入口要求操作者预先启动worker。

先做只观测地面运行（会启动场景；本次修改未执行此命令）：

```bash
bash deployment/object_detection/build_query_overlay.sh
bash deployment/object_detection/test_query_overlay.sh
# 按本机实际路径设置；宿主准备 msgpack，审计解释器准备 NumPy/OpenCV/Pillow。
export MOUNTAINS_PACKAGE=/path/to/Mountains
export PYTHONPATH="$PWD/deployment/robopi/vendor${PYTHONPATH:+:$PYTHONPATH}"
python3 deployment/target_tracking/airsim/mountains/run.py \
 --scene-package "$MOUNTAINS_PACKAGE" --mission-query car --estimate-position \
 --output log/target-tracking-dino-ground-new --dry-run
# 审核计划后移除 --dry-run：仅地面观察，不起飞、不导航。
```

悬停必须显式选择，且先独立通过飞行门；以下命令留给操作者之后执行，本轮未运行：

```bash
AIRSIM_RENDER_MODE=windowed python3 deployment/target_tracking/airsim/mountains/run.py \
 --scene-package "$MOUNTAINS_PACKAGE" --flight-only --mission-query car \
 --output log/target-tracking-dino-flight-new
AIRSIM_RENDER_MODE=windowed python3 deployment/target_tracking/airsim/mountains/run_hover.py \
 --scene-package "$MOUNTAINS_PACKAGE" --mission-query car \
 --output log/target-tracking-dino-hover-new
```

原生Takeoff/悬停/Land、静态确认、深度、新鲜度和停车门均未放宽。悬停观察不追逐。
以下旧截图、YOLO类别诊断及VALIDATION均属于原后端的历史阶段，不证明DINO通过。
`diagnose_classes.py` 保留为显式YOLO历史诊断，不由DINO入口自动运行，不读取DINO状态。
已有保存皮卡可检出二维car，但竞争深度使三维无效；实时DINO有效跟踪与飞行仍待验证。


只维护目标状态的算力侧 Tool；能力卡片在 `cards/tool/target-tracking.yaml`。
完整接口语义见 [源码说明](../../src/tool/target_tracking/README.md)。

```bash
git submodule update --init deployment/airsim/demos/target_tracking/reference/yopo_tracker
bash deployment/object_detection/build_query_overlay.sh
bash deployment/object_detection/test_query_overlay.sh
```

构建入口使用已有 `onboard_computer:v0.1.1`、只读源码、独立外部工作区
`log/object-detection-query/workspace/`，不重建镜像、不操作已有容器。依赖接口在同一 overlay
重生成；输出路径可作为首个参数指定。构建容器无网络，锁定 Python wheels 在宿主准备并校验；测试入口仅运行合成 DDS/几何测试，不是平台验收。

在现有算力容器挂载本仓库及构建工作区，加载匹配的 ROS/AS2/检测 overlay 后运行：

```bash
source /opt/ros/humble/setup.bash
UAV_NAMESPACE=drone0 bash deployment/target_tracking/start.sh
ros2 action send_goal /drone0/tools/target_tracking/track \
 uav_target_tracking_interfaces/action/TrackObject '{query: car, timeout: 10.0}' --feedback
```

默认参数启用仿真时钟，部署必须提供唯一 `/clock`；实机显式覆盖
`-p use_sim_time:=false`。跟踪和 DetectSnapshot 必须共享车辆命名空间与时钟域。
先启动检测依赖及真实传感器，不能把 Action 可发现当作目标健康。
取消使用标准 Action 取消；SIGINT 只停止自己启动的 Tool 进程。
回滚：取消会话、停该进程、恢复源码并重编 overlay、重新启动；EKF 无持久状态。

本组件单独登记，不加入 Navigation Planner Registry（该 Registry 只负责规划算法）。
未实现 Mission 或跟随导航。验证、失败尝试及平台缺口见 [验收记录](VALIDATION.md)。

## 静态真实平台探针

先按 Object Detection 的说明准备模型与外部工作区并启动实际 CUDA worker：

```bash
INFERENCE_PORT=8003 bash deployment/inference/start_grounding_dino.sh
# 另一终端，DISPLAY/XAUTHORITY 使用本机已授权图形会话：
AIRSIM_RENDER_MODE=windowed \
PYTHONPATH="$PWD/deployment/robopi/vendor${PYTHONPATH:+:$PYTHONPATH}" \
python3 deployment/target_tracking/airsim/run.py --estimate-position \
 --vehicle-x 17.6 --vehicle-y -3.4 --output log/yopov2-tracking/airsim-ground-new
```

此入口复用既有 Object Detection 地面 Harness，自建隔离 DDS 域和自有场景/容器，
并在 finally 中只回收自己创建的资源。必须使用新的输出目录。
地面探针通过 AS2 Module 消费 TrackObject，预声明至少四次有效 tracked、有效时间
与协方差、标准取消、前后 disarmed。现有场景汽车为静态输入，不插入合成目标。
不要传 `--near-target` 或 `--collect-failures`（它们属于检测器原评分探针）。
本入口不是移动目标精度、遮挡恢复、跨设备部署或飞行验收。

本次默认切换的实际测试与未执行项见 [DINO默认验证](VALIDATION-dino-default.md)。
