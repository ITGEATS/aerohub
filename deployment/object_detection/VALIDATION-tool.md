# Object Detection Tool 0.3.0：交付与验证

> 以下为历史版本证据；当前 query 接口及调用示例见 [Detect README](../../src/tool/object_detection/README.md)。历史 category 字段及 CLI 不适用于当前 schema。

2026-11-15：当前版本可用于单机任务的按次目标检测，默认RGB-only，可选RGB-D定位。
公共AS2 Module、真实AirSim相机输入和CUDA推理调用链已接通，静态近距定位已通过
预声明评分。飞行、全距离精度及长期网络可靠性保持未验收状态。

## 交付接口

```python
drone.load_module("object_detection")
observation = drone.object_detection.detect(category="car", timeout=2.0)
positioned = drone.object_detection.detect(
    category="car", timeout=2.0, estimate_position=True)
```

- 实现位于 `src/tool/object_detection/`，模型仍位于基础Inference层。
- 每次选择接纳后新收到的首帧RGB，保留采集stamp；不保证图像在客户端调用后才采集。
- 定位默认关闭，不启动深度、标定及TF消费者；首次三维请求后消费者保留，后续二维仍跳过投影。
- `success`表示二维检测完成；无目标可以成功，三维不可用可返回PARTIAL，位置仅在
  逐对象 `position_valid=true` 时可用。远目标无有效深度是正常观测边界。
- 超时和停止使在途结果失效，不取消已运行的GPU kernel；后台完成前维持忙位。
- 保留旧DetectObjects与可选stream接口，共享同一Adapter和推理worker。
- Card v1及组件清单契约版本0.3.0；三维条件依赖在组件清单声明。

构建使用固定 `onboard_computer:v0.1.0` 镜像与外部overlay，入口为
`build_tool_overlay.sh`。接口、Adapter、Module及消费者须同步重编译并重启。
调用与部署细节见 [Tool README](../../src/tool/object_detection/README.md)。

## 已执行检查

证据相对根目录为 `log/object-detection-tool/`，原始日志与图像不纳入Git。

| 检查 | 结果 | 证据 |
| --- | --- | --- |
| 固定镜像外部overlay构建 | 6包通过 | build-tf-qos.log |
| AS2 Module、ROS Service与生产Adapter | 8项通过，含三维失败后切回二维、超时、忙、停止与迟到丢弃 | contracts-final-tf-qos.log |
| 深度/投影、采样时刻TF、预算内晚到及持续缺失 | 7项通过 | contracts-final-tf-qos.log |
| 原YOLO ROS契约 | 4项通过 | contracts-verified-final.log |
| 真实CUDA模型准备与加载 | 通过 | yolo-setup.log |
| Module到真实CUDA调用链 | 4个人物检出；深度/TF为软件夹具 | module-cuda.json |
| 实际AirSim RGB-only | 汽车检出，未初始化定位消费者 | airsim-rgb-only-01/probe/result.json |
| 修正后实际AirSim近距三维 | 连续10/10有效，独立表面评分通过，最大深度误差0.168762m | near-tf-qos-final/probe/result.json、near-tf-qos-final/near-score.json |
| 动态TF订阅对照 | 100次中TF超时由4降为0；修正后98次有效、2次深度配对失败，该压力轮整体未通过 | near-tf-stress-06、near-tf-qos-01 |
| Card v1与接口绑定 | 本Tool无错误；全仓另3张旧卡共57项错误 | cards-closeout.log |

ROS回归使用确定性推理替身，与真实模型、实际仿真传感器证据分开。
软件夹具CUDA探针的公开RGB字节SHA256为
`c02019c4979c191eb739ddd944445ef408dad5679acab6fd520ef9d434bfbc63`；
模型权重SHA256为
`361fbfabab285c3237700b6bb91d7ecfa602cd945fffda8dbe1242829b71e73f`。
原模型验证见 `src/basic_function/inference/models/yolo26n_seg/validation.json`。

## 已知边界与复验

实际输入保持原50ms配对、0.25s TF等待及深度支持门限。动态TF采用sensor-data
best-effort，静态TF保留可靠transient-local。当前仍有偶发深度配对失败，按契约
保留二维结果，不将无效位置交给下游。全局GPU仲裁与通用任务生命周期仍由后续基础设施承担。

当前精度证据限于静态近距表面深度；同一TF的代数往返残差不代表独立的世界坐标精度。
历史失败记录、TF对照和评分限制见 [POSITIONING.md](POSITIONING.md)，
环境准备及完整命令见 [AirSim复验说明](airsim/README.md)。

复验入口保留被动TF/里程计记录、失败收集、颜色核对与独立表面评分。既有诊断证据
还包含当时的投影起止字段；当前入口不改写生产Projector方法。
清理后入口帮助检查及8项公共/7项投影回归通过，见 `cleanup-checks.log`。
