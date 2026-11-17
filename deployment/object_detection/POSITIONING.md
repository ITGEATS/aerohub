# Object Detection 可选三维定位：现状与问题拆解

2026-11-15。三维链路来自 gaojincheng 的 `3311f8e`（2026-11-13），本轮保留
原深度测量与投影算法，将它改为公共请求的 `estimate_position=True` 选项。
默认false只做RGB检测，不等待深度或TF，不初始化其消费者。
初次三维/旧接口调用后消费者保留到进程退出；后续二维请求仍完全跳过投影。

## 接口边界

- 二维必需：新收到的RGB、YOLO推理服务。输出类别/置信度/二维框/实例掩码。
- 三维额外需要：配准的平面深度、RGB/深度CameraInfo、采样时刻相机到输出系TF。
- `result.position_requested` 说明是否请求三维。二维的position_valid=false不是失败，
  position_error为空，header为原RGB头，depth_header为空。
- 请求三维后仍保留2D结果；目标定位失败为PARTIAL，逐对象position_valid与
  position_error决定可否使用坐标。无目标按现有约定成功返回，reason中的测量错误
  仍保留，不用空对象列表证明定位链有效。
- Card v1没有条件依赖字段；Card列基本RGB输入与硬件，组件清单
  optional_position_inputs保存三维选项的附加依赖，不改动全仓Card Schema。
- 这是ROS消息Schema变更；须连同接口、adapter、Module及其他消费者重编译并重启，
  不能让旧客户端与新服务混用。公共契约版本0.3.0。旧DetectObjects类型保持不变。

## 2026-11-15 实际验证与边界

默认二维与显式三维路径均已执行实际 AirSim RGB / CUDA / AS2 Module 调用。
近距测试使用场景既有汽车，车辆静态初始位置为 AirSim 世界 NED
X=17.6、Y=-3.4、Yaw=0；全程 disarmed，无运动或解锁调用。
目标位置、相机 pose 和场景碰撞查询只供评分，不进入 Tool。

预声明检查为：2–8m可见表面、至少10次连续有效近目标；独立场景碰撞射线参考的
视轴深度误差≤0.30m，输出系变换残差≤0.05m。评分使用
AirSim ECC_Visibility 碰撞表面，与渲染深度为不同来源，但碰撞模型与视觉表面
可能不同，且地理坐标换算有精度限制。变换残差是同一TF的代数往返检查，
不能证明绝对定位精度、外参真值或运动下的时间正确性。

| 检查 | 结果与证据（均位于 log/object-detection-tool/） |
| --- | --- |
| 真实近目标 | near-car-05连续10/10；near-tf-stress-01、02各30/30；修正后near-tf-qos-final连续10/10且独立评分通过 |
| 静态深度 | 3.244–3.246m；碰撞参考3.413m；误差0.167–0.169m，满足本轮0.30m门限 |
| 输出变换 | 代数往返残差满足0.05m门限；不是独立的世界坐标精度评分 |
| 同帧远目标 | 有2D检测但有效深度不足，逐目标无效，整体PARTIAL；近目标仍有效 |
| 公共接口与模式切换 | contracts-closeout-tf.log：8项公共/7项投影测试通过，含三维失败后切回二维不调用投影 |
| TF预算内晚到与持续缺失 | 新增原stamp等待成功和持续陈旧失败回归；保持0.25s默认预算，无latest回退 |
| TF间歇超时 | stress-06基线100次中4次；动态TF改用sensor-data best-effort后，near-tf-qos-01同配置100次为0次，另2次深度配对失败 |

保留失败证据：near-car-01旧颜色配置无检出；02树木遮挡无检出；03近目标有效但
测试误要求整帧SUCCEEDED，同帧远目标应为正常PARTIAL，后续修正逐对象断言；
04为TF超时。near-tf-stress-03在调用前10帧输入检查失败，首张RGB无匹配深度
（210ms），因此100次调用未执行；04首次调用缺少CameraInfo，随后补全测试就绪条件；
05在21次有效后配对失败；06收集完100次的所有失败，没有放宽50ms同步门限。

原算法0.02–10m、至少20点的深度支持门限及测量实现保持不变。
低深度支持是正常观测边界，不能从全图finite推断目标区域有效。
飞行、运动下TF连续性、多距离检测/测量精度和长期传输可靠性尚未验收。

## TF订阅修正与对照

near-tf-stress-06失败时独立best-effort观察者的TF落后clock为0–21ms，
Tool默认reliable动态TF Buffer落后537–735ms；源里程计仍持续收到新stamp。
这将问题定位到Tool动态TF订阅链，而非PX4/AS2状态源整体停顿。
固定Humble TransformListener默认动态TF为reliable/depth100；本Tool仅将
动态TF改为sensor-data best-effort/depth5，静态TF保留可靠transient-local默认值。
相同场景100次对照为98次有效、0次TF超时、2次深度配对失败；基线为94次有效、
4次TF超时、2次深度配对失败。支持订阅策略修正，但没有抓包证明中间件的具体
重传过程，也不承诺长期/所有网络负载下零超时。
采样stamp、0.25s等待、50ms配对和深度门限均未放宽。
修正后6包构建及8公共+7投影回归通过（contracts-final-tf-qos.log）。

## 收尾状态

真实静态近距、模式切换和契约回归已有通过证据，动态TF订阅修正有对照证据。
100次压力轮仍因2次配对失败整体未通过；保留此前10/30/30成功轮与全部失败证据，
不能将本轮标记为完整三维/飞行复现成功。原始RGB、深度、掩码、CameraInfo、
采样时刻TF、状态到达追踪和评分结果保存在对应运行目录。
具体命令、颜色核对与历史证据见 [AirSim验证说明](airsim/README.md)。
