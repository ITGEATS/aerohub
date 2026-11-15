# Object Detection 公共 Tool 封装验证

2026-11-15。本轮为已有 YOLO/RGB-D 实现增加公共任务接口及 AS2 Module，
不是重新宣称完成模型复现、现场测量精度或飞行验收。

## 变更与边界

- 已有实现移至 `src/tool/object_detection/`；包名、旧接口类型和推理服务不变。
- `DetectSnapshot` 接收 category/timeout，适配层从部署绑定相机选择接纳后收到的第一帧RGB，
  复用原推理、类别过滤、深度测量、历史TF与generation失效机制。
- 新 `ObjectObservation` 只返回观测头、模型身份和测量对象，不暴露内部request_id/generation。
- `drone.load_module('object_detection')` 提供同步 `detect(category='person', timeout=2.)`。
- 无目标是成功；2D已完成但3D不可用返回成功PARTIAL，逐项position_valid决定位置能否使用。
- 超时后丢弃结果，在后台处理结束前保留该公共任务的忙位；节点退出会终止等待帧，
  不承诺取消运行中的GPU kernel。
- 旧DetectObjects和stream兼容保留；部署示例默认关闭连续流，避免与按次请求竞争。
- Card迁至v1，当前契约版本0.3.0；具体服务地址、相机和支持类别保存在组件清单/配置中。
  不新增动态发现、全局GPU仲裁或Agent生命周期管理。

## 预定检查与执行结果

| 检查 | 结果与证据 |
| --- | --- |
| 固定镜像外部overlay构建 | 6包通过，`log/object-detection-tool/build-verified.log` |
| 实际AS2 Module + ROS Service + 生产适配层 | 7项通过，推理为确定性替身，`contracts-verified-final.log` |
| 原RGB-D投影回归 | 5项通过，同上日志 |
| 原YOLO ROS契约回归 | 4项通过，同上日志 |
| 真实CUDA模型准备与加载 | 通过，原uv.lock与权重SHA，`yolo-setup.log` |
| Module到真实CUDA完整调用链 | 通过，`module-cuda.json`、`module-cuda-01.log` |
| Card v1与清单、接口绑定 | 本Tool通过；全仓因另外3张旧卡未迁移而失败，`cards.log` |

所有上述日志位于 `log/object-detection-tool/`。
公共接口测试覆盖：首帧新鲜接收、逐次person/car类别、有效位置、无目标、
无深度保留2D、无新图超时、在途推理超时/忙/迟到丢弃、停止、非法类别，以及在途停止丢弃。
测试使用真实ROS/TF/生产测量代码和独立DroneInterface执行器；不将替身输出当模型证据。

使用已有 `onboard_computer:v0.1.0`，不重建共享镜像；独立工作区为
`log/object-detection-tool/workspace`，构建入口为 `build_tool_overlay.sh`。
AS2 Python API从当前锁定源码加入overlay；Torch不进入ROS环境。
本轮没有改动YOLO模型、权重、推理适配、投影和深度算法。
一次最终测试曾错误地断言“图像采集时刻晚于客户端调用”；DDS传输中的新收到图像
可能在调用前刚采集，不能从接收保证推导该条件。按已定义契约修正测试为正采集时间、
RGB/depth时间保留，并用独立无新RGB用例检查绝不复用缓存；失败日志保留在
contracts-complete.log，最终结果在contracts-verified-final.log。

## 真实CUDA证据范围

公开输入图像来自
`https://raw.githubusercontent.com/ultralytics/ultralytics/main/ultralytics/assets/bus.jpg`，
本轮下载字节SHA256为
`c02019c4979c191eb739ddd944445ef408dad5679acab6fd520ef9d434bfbc63`。
模型权重SHA256为
`361fbfabab285c3237700b6bb91d7ecfa602cd945fffda8dbe1242829b71e73f`。

`probe_module_cuda.py` 经实际AS2 Module、新Service、RGB-D适配、ROS推理Facade、
HTTP worker及CUDA推理得到4个人物检测；model_id=yolo26n-seg、device=cuda:0，
模型版本和完整元数据保存在module-cuda.json。
深度固定2m、CameraInfo和静态TF明确为软件夹具，检查输出坐标和采集时刻协议。
这不是实时相机/仿真RGB-D精度、目标跟踪或飞行通过；这些不在本轮接口封装验收中。
原模型验证与限制仍见 `src/basic_function/inference/models/yolo26n_seg/validation.json`。

本轮服务、ROS测试进程和独立CUDA worker均已回收；模型环境/权重及证据保留在忽略目录中。

## 后续真实 AirSim RGB-D 接入

地面接入、传输修正、逐次观测与未通过的三维测量项见
[AirSim 验证说明](airsim/README.md)。实际输入与CUDA调用链通过；
汽车深度测量为PARTIAL，一次空结果有TF等待超时，不提升为三维或飞行验收成功。

## 0.3.0：可选三维定位

DetectSnapshot新增estimate_position，默认false；ObjectObservation新增
position_requested。二维路径直接返回YOLO检测，不调用Projector，按需启动三维
消费者。旧DetectObjects保持原RGB-D语义。
6包构建通过，8项公共测试及5项投影测试通过；真实RGB-only与显式三维的
两轮地面调用均完成，三维汽车结果仍PARTIAL。完整证据与补跑限制见airsim/README.md。
Card仅声明基本RGB依赖，条件定位依赖写入组件清单；全仓校验仍有无关旧卡问题，
没有扩展Card Schema。


## 2026-11-15 收尾复核

`contracts-closeout-tf.log`：8项公共接口、7项投影测试通过，覆盖三维失败后
切回二维不调用project、预算内TF晚到及持续陈旧拒绝。近距真实AirSim/CUDA
10/30/30次调用均得到有效近目标；静态深度误差0.167–0.169m，满足预设0.30m。
同帧远目标保持PARTIAL，逐对象判定。RGB字节顺序已由同批raw/PNG核对并显式配置BGR。
历史表格保留当时检查结果，当前定位状态以 [POSITIONING.md](POSITIONING.md) 为准；
动态TF订阅可靠传输链落后已通过观察者对照定位；改用sensor-data best-effort后
100次中0次TF超时，98次有效、2次配对失败，保留该轮失败状态。
最终构建6包、8公共+7投影回归通过（contracts-final-tf-qos.log）。
不宣称三维完整复现或飞行验收完成。

修正后最终 `near-tf-qos-final` 连续10/10有效，独立碰撞表面评分通过，
最大深度误差0.168762m。该通过项不覆盖压力轮配对失败或未执行的运动验收。
