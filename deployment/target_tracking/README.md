# Target Tracking 部署

只维护目标状态的算力侧 Tool；能力卡片在 `cards/tool/target-tracking.yaml`。
完整接口语义见 [源码说明](../../src/tool/target_tracking/README.md)。

```bash
git submodule update --init deployment/airsim/demos/target_tracking/reference/yopo_tracker
bash deployment/target_tracking/build.sh
```

构建入口使用已有 `onboard_computer:v0.1.1`、只读源码、独立外部工作区
`log/yopov2-tracking/build/`，不重建镜像、不操作已有容器。依赖接口在同一 overlay
重生成；输出路径可作为首个参数指定。无网络容器只跑合成 DDS 测试，不是平台验收。

在现有算力容器挂载本仓库及构建工作区，加载匹配的 ROS/AS2/检测 overlay 后运行：

```bash
source /opt/ros/humble/setup.bash
UAV_NAMESPACE=drone0 bash deployment/target_tracking/start.sh
ros2 action send_goal /drone0/tools/target_tracking/track \
 uav_target_tracking_interfaces/action/TrackObject '{category: car, timeout: 10.0}' --feedback
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
INFERENCE_PORT=8002 bash deployment/inference/start_yolo26n_seg.sh
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
