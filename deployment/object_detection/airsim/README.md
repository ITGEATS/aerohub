# YOLO Tool：真实 AirSim RGB-D 地面接入

2026-11-15：真实相机输入、公共 AS2 Module 与 CUDA 检测调用链已通过地面检查。
静态近距测量已有表面参考评分；间歇TF与传输失败另行记录。
不将这些地面证据扩展为全距离精度、目标检测准确率或飞行验收通过。

## 调用与数据链

```text
AirSim FrontCamera（同一次 simGetImages 请求 RGB + DepthPlanar）
  → 共享 uav_as2_sensor_adapter（唯一相机 RPC 采集者）
  → ROS RGB8 / 32FC1 + 两路 CameraInfo
  → ObjectDetectionTool：接纳后收到的首帧
  → ROS Inference Facade → 独立 YOLO26n-seg worker → CUDA
  → 原实例掩码深度测量 + PX4 → AS2 提供的历史 TF
  → DetectSnapshot → drone.object_detection.detect(query, timeout)
```

Tool 不读取 AirSim RPC；相机绑定、几何与传输由部署层配置。
640×480、水平视场90°，RGB与平面深度共置，光学坐标系为
`drone0/FrontCamera_optical`，输出为 `drone0/odom`。
保留 AirSim 每路采样 stamp；两路不伪造成同一时刻，配对上限保持50 ms。
RGB源显式使用 `source_color_order=bgr`，桥输出rgb8，推理Facade按编码转换。
新入口在桥启动前同批抓取raw Scene和压缩PNG：解码PNG后与raw BGR比较，
MAE为0，交换通道后约18.7；记录于scene-reference.json/png。该检查核对当前
场景二进制的实际字节顺序，不代表色卡标定。早期08/09等运行误设rgb，
其传输和CUDA链路证据保留，但正确颜色仅由后续near-car-03起的运行验证。

## 复验

前置：可用有屏AirSim场景、NVIDIA GPU、现有 `flight_board:v0.1.0` 与
`onboard_computer:v0.1.0`，以及可用的显示会话。
不重建镜像；先在仓库根目录准备模型和外部工作区：

```bash
bash src/basic_function/inference/models/yolo26n_seg/setup.sh --check-cuda
bash deployment/object_detection/build_tool_overlay.sh
INFERENCE_PORT=8002 bash deployment/inference/start_yolo26n_seg.sh
```

worker保持运行，在另一个终端执行：

```bash
DISPLAY=:1 \
PYTHONPATH="$PWD/deployment/robopi/vendor${PYTHONPATH:+:$PYTHONPATH}" \
AIRSIM_RENDER_MODE=windowed \
python3 deployment/object_detection/airsim/run.py \
  --output log/object-detection-tool/airsim-rgb-new
```

`DISPLAY` 使用当前实际显示会话；输出目录必须是 `log/` 下尚不存在的目录。
入口检查8002端口worker的CUDA健康状态、选用未使用的DDS域，启动自有仿真和容器，
在finally中回收自有容器及场景。worker为外部前置，测试后由启动它的终端停止。
运行期间不解锁、不起飞；检查开始/结束时的平台armed字段。
已有其他场景或端口占用会使运行失败，先处理冲突，不停止无关容器。

容器使用共享 `deployment/airsim/config/dds/fastdds_udp.xml`、host网络及
`ROS_LOCALHOST_ONLY=0`，以支持PX4 XRCE Agent发现。
相机桥和Tool均显式设置 `camera_reliable=true`；Tool的默认false仍兼容原sensor QoS。
仅将发布端设为reliable不足以证明best-effort订阅端获得完整RGB-D配对。

输出含checks/settings/sources/images/transport、camera-timing.jsonl、各角色日志，
以及probe/result.json、rgb.png和depth-m.npy。测试先等相机与平台，再等真实深度
采样时刻的AS2历史TF；周期PlatformInfo或Module服务发现不代替测量链就绪。

## 历史接入记录（08/09，早于近距与TF修正）

证据目录：`dev/object-detection-tool/evidence/airsim-rgb-08/` 与 `airsim-rgb-09/`。
原始日志不纳入Git。09的结果如下：

| 项目 | 结果 |
| --- | --- |
| 真实RGB/深度格式与尺寸 | rgb8 / 32FC1，均640×480 |
| 标定 | 两路相同K，fx=fy=320，cx=319.5，cy=239.5，零畸变 |
| 预定10帧配对 | 全部≤50ms，最大9.000192ms |
| 初始观测窗口 | 11个唯一RGB stamp、10个唯一depth stamp，RGB接收约5.05Hz |
| RGB接收时年龄 | 仿真时钟差21–42ms；不等于整条检测链wall延迟 |
| 深度有限值比例 | 100%；有限不代表在测量算法有效范围内 |
| 模型 | 实际cuda:0，权重与模型身份保存在result.json |
| 公共调用 | person/car/bus，均为调用后新收到的图像，保留来源stamp |
| 安全状态 | 首尾disarmed，全程未请求解锁/起飞 |

09三次wall耗时约0.624/0.605/0.868秒，均在各2秒请求预算内。
person和bus没有检出对象；car检出1个对象，但有效深度点不足20，返回
`PARTIAL`、`position_valid=false`。原测量实现只接受0.02–10米，未改算法门限。
因此09轮没有有效汽车三维位置证据；后续近距通过结果见下文及POSITIONING.md。

09的bus空结果还保留了一次 `capture-time TF wait expired` reason：
原投影阶段0.25秒等待期间TF未覆盖对应stamp；响应后显式历史TF查询已可用。
无目标的公共契约允许成功返回，但不能据此宣称该次三维测量成功。
result.json的passed只表示checks.json列出的地面链路检查通过。
飞行、运动下TF连续性、持续负载、检测准确率及三维精度仍未验收。

失败尝试也保留：01–04的best-effort输入分别丢失RGB/深度，源stamp配对正常；
共享IPC/builtin传输尝试未解决。05两端reliable后配对通过，但AS2 TF缺失；
06–07等待仍失败，话题诊断显示PX4里程计发布端不可见。
08–09采用共享UDP配置及非localhost-only发现后，真实状态和检测链接通。
未通过降低分辨率、放宽配对阈值或伪造时间戳获得通过。

新增可选QoS后，既有7项公共接口测试及5项投影回归通过，日志为
`log/object-detection-tool/contracts-camera-qos.log`；增量6包构建通过，见
`build-camera-qos.log`。这些回归使用夹具，与上述真实输入证据分开。

## 0.3.0：二维默认与三维选项

当前入口默认只配置RGB捕获，Tool不启动深度/标定/TF消费者。
上文RGB-D复验命令现在须追加 `--estimate-position` 才执行三维检查。
公共调用默认 `detect(query='person', timeout=2.)`；定位需显式
`detect(query='person', timeout=2., estimate_position=True)`。

新接口实际验证：

- `airsim-rgb-only-01/probe/result.json`：真实RGB-only场景，三次调用均SUCCEEDED，
  汽车1个检测，深度头为空、无position_error，未初始化定位消费者。约4.97Hz。
  记录的ROS clock与图像stamp差为-18至9ms，含clock桥更新相位，不当作精确传输延迟。
- `airsim-optional-3d-01/probe/result.json`：显式三维选项真实CUDA回归通过链路检查；
  汽车仍因有效深度点不足返回PARTIAL，无有效三维坐标。三次调用TF无报错。
- `contracts-optional-position.log`：8项公共接口与5项投影测试通过。
  后续 contracts-closeout-tf.log 已补跑“三维失败后再切回二维时禁止调用project”
  断言；8项公共接口和7项投影测试通过。

所有上述路径均位于 `log/object-detection-tool/`。三维问题拆解见
`../POSITIONING.md`。ROS接口、Module、adapter须一起重编译并重启。


## 静态近距三维与TF追踪

```bash
DISPLAY=:1 \
PYTHONPATH="$PWD/deployment/robopi/vendor${PYTHONPATH:+:$PYTHONPATH}" \
AIRSIM_RENDER_MODE=windowed \
python3 deployment/object_detection/airsim/run.py \
  --estimate-position --near-target --vehicle-x 17.6 --vehicle-y -3.4 \
  --output log/object-detection-tool/near-new
```

使用既有汽车、不插入合成目标，默认连续10次；`--samples 30`可重复检查。
保存每次原RGB、深度、实例掩码、采样时刻TF，以及独立监听TF/里程计的
`probe/state-trace.json`。`near-score.json`使用AirSim场景碰撞射线评分。
门限、通过数据、失败尝试与评分局限集中见 [三维测量说明](../POSITIONING.md)。
近距10/30/30次有效与真实TF失败分别保留；动态TF订阅修正及压力对照见下文。
这些是静态地面接入证据，未完成运动与飞行验收。


诊断时可增加 `--collect-failures --samples 100`：保留每个无有效近目标响应，
继续采样以区分TF与深度传输问题；最后任一失败仍使本轮失败，不进入全通过评分。
near-tf-stress-06共100次：94次有效，4次TF超时、2次深度配对失败。
失败期间独立best-effort TF与里程计观察者仍收到新stamp，Tool内部默认reliable
TF Buffer则落后，已排除全链状态源中断。后续对照只将动态TF订阅改为
sensor-data best-effort，静态TF仍可靠且transient-local；查询继续使用采样时刻，
等待预算仍0.25s。最终对照结果见POSITIONING.md。
