# YOPO-Tracker 推理 Worker

模型 ID 为 `yopo-tracker`，与普通 YOPO 的权重和接口独立。使用上游 YOPO-Tracker MIT 源码 `d313f8610195dc986caceff7a3616f63d7e6f31f` 及其中的 `YOPO/saved/YOPO_1/epoch50.pth`。准备脚本核对源码提交与权重 SHA-256。

## 准备与启动

使用 Linux x86-64、Python 3.10、uv 和可用的 NVIDIA CUDA GPU。在仓库根目录执行：

```bash
export YOPO_TRACKER_UPSTREAM=/absolute/path/to/YOPO-Tracker
bash src/basic_function/inference/models/yopo_tracker/setup.sh
bash deployment/inference/start_yopo_tracker.sh
```

`setup.sh` 按本目录锁文件准备独立 `.venv` 与本地 `artifacts/epoch50.pth`，并用实际 CUDA 加载和预热。通过公共 [Inference 准备入口](../../README.md) 选择该模型时，同样提供 `YOPO_TRACKER_UPSTREAM`。worker 默认监听 `127.0.0.1:18082`，可设置 `INFERENCE_HOST` 与 `INFERENCE_PORT`。运行时可通过 `YOPO_TRACKER_WEIGHTS` 指定同一校验和的权重副本。就绪检查使用 `GET /health/ready`。

## 输入与输出

Planner 复用原生预处理，把 320 × 192 RGB-D 转为 BGR 加深度的 `float32[1,4,192,320]`，同时发送原生网格状态 `float32[1,6,3,5]`。 [InferYopoTracker ROS 服务](../../interfaces/srv/InferYopoTracker.srv) 保留 RGB、深度、里程计的原始 Header 和请求 ID。ROS Adapter 校验测量时差及帧龄，通过共享 HTTP runtime 将模型输入交给独立 worker。

worker 返回 3 × 5 网格上的五组原始预测头、权重摘要和设备信息。目标 EKF、候选解释、轨迹求解与制动由 [YOPO-Tracker Planner](../../../../tool/navigation/planner/yopo_tracker/README.md) 保有。闭环回放入口见 [适配说明](../../../../tool/navigation/planner/yopo_tracker/ros2_adapter/README.md#默认-followtimedpath-接口复验)。
