# Mountains 侧视真实飞行与目标观察 — 2026-11-16

**FAILED / partial：侧视原生飞行通过，车辆检测与静态跟踪未通过，12s移动观察未执行。**
没有修改检测阈值0.35、类别映射、有效期或飞行门槛；没有用黄色外观强制指定身份。
无导航跟随、无人机传送或AirSim运动API，无镜像重建、上游修改、commit或push。

| 运行（均位于 log/yopov2-tracking/） | 实际结果 |
| --- | --- |
| side-ground-20261116-01 | X3.5/Y0/Yaw180侧视；碰桥侧护栏，水平法线、穿透0.3505m；PX4 readiness超时，未解锁；保留失败 |
| side-ground-20261116-02 | X2.6/Y0；仅桥面垂直接触、穿透0，地面审查通过 |
| side-native-20261116-01 | 原生3m起飞、悬停、Land、landed/disarmed通过；31个悬停物理样本，高度2.9053–3.0763m；侧面图仍有机臂遮挡 |
| side-native-20261116-02 | 相机FRD(.8,0,-.6)、Pitch−55°后完整侧面入镜；同一飞行门通过，31样本、高度2.8185–3.0582m |
| side-observe-20261116-01 | 飞行通过，71样本、高度2.9150–3.0738m；51次真实CUDA均无受支持类别框；Mission初始等待无有效目标；静态失败 |
| side-observe-20261116-02 | 同配置复核并保存实际TF；飞行通过，77样本、高度2.9081–3.0745m；50次CUDA均无受支持类别框；12s静态预算到期失败 |

四次飞行均无顶部接触，原生Takeoff/hover/Land/disarmed全部通过。
两次完整观察均为 **0/4不同有效静态stamp和capture**；车队始终未启动，独立皮卡
位移0m。不能用静态失跟后的零位置或空集合宣称几何、三维精度或移动观察通过。
Mission容器内会话总上限32s包含静态准备；车队实际移动循环仍固定完整12s，因门失败未进入。

## 配置与实际类别

`side_settings.json` 为独立地面初态：NED(2.6,0,0)、Yaw180°。场景皮卡实际朝−Y，
此出生点观察其侧面；避开Y12装甲车和X3.5护栏。起飞后没有移动/传送无人机。
前视相机真实杆臂FRD(.8,0,−.6)、Pitch−55°，RGB/DepthPlanar640×480、水平90°。
原地面和Y6飞行默认配置保留。实际车辆图像见各轮 `probe/hover-parked.png`，
原始模型输入见 `probe/cuda-inputs/*.npy` 与配对PNG；地面raw字节另存`initial-raw-scene.npy`。

同CUDA后端对保存的真实输入做未过滤COCO类别诊断，阈值仍0.35：

- 初次带遮挡的侧面图：car0.2382，truck0.0616，无框过阈值。
- 移除遮挡后的独立飞行图：**microwave0.7717**，框约[101,192,543,359]。
- 完整观察01首/中/末帧：microwave0.7898/0.7953/0.7951。
- 完整观察02首/中/末帧：microwave0.8381/0.7699/0.7507；car最高0.00923，truck最高0.00134。

上述误识别框覆盖车辆区域，是模型输出事实，不是车辆身份。部署仍只支持
person/car/truck/bus，未把microwave或train重映射为车辆。观察请求选择受支持car
（来自初次侧图最高受支持分数），并未更改检测映射或伪造目标。
全类别回放只用于诊断，不进入Tracking；首中末采样也不冒充101帧全类别统计。
当前证据定位为模型输出不满足车辆契约，尚未确定其更深层原因。

## 标定、失败谓词与收口

最终复核`probe/camera-tf.json`保存depth stamp1794795093.457644处的实际
body→optical及earth→optical TF。body光学平移FLU(.8,0,.6)，四元数
[−.6743797,.6743797,−.2126311,.2126311]，与相同settings推导一致。
物理相机杆臂误差4.52e−7m、旋转误差2.47e−5°；实际ROS外参误差0m/1.71e−6°。
渲染投影矩阵fx320/fy320.00001px与CameraInfo配置320px一致。
RPC另报FOV89.9036°，原值保留，不用该标量代替实际投影矩阵。
这是相机相对几何检查，不是earth绝对定位精度验收。
首轮清理时临时ROSCLI导出得到空`tf-static.yaml`，不作为TF证据；复核轮补齐实际Buffer证据。

每轮预声明`checks.json`、源码/镜像/场景输入SHA和飞行源码快照保留。
`probe/predicate-samples.json`含飞行高度/漂移/速度/健康及静态唯一stamp/capture/spread；
`probe/mission/mission.jsonl`含wall/monotonic/ROS时间、capture、观测stamp及逐有效性谓词。
失败RGB-D、实际相机pose、推理框/投影调用、physics-samples、PX4/AS2日志及实际桌面截图保留。
本轮桌面截图确实可见仿真场景窗口；没有声称生成视频。

六轮收口均确认三车实际速度≤0.1m/s连续5次，实测均0；飞行轮先停车、排空Tracking，
再原生Land，确认PX4 landed且平台/PX4解除武装后清理。没有恢复错误或遗留所属资源。
`cleanup.json`及最终`runtime-final.json`确认两个既有容器ID/StartedAt未变。

## 复验与交付

```bash
# 使用本机已有图形会话、RPC依赖及只读场景包；必须使用新的log输出目录
PYTHONPATH="$HOST_RPC_DEPS" python3 deployment/target_tracking/airsim/mountains/run.py \
  --scene-package "$MOUNTAINS_PACKAGE" \
  --settings-file deployment/target_tracking/airsim/mountains/side_settings.json \
  --hover-observe --mission-category car --output log/yopov2-tracking/side-new
python3 deployment/target_tracking/airsim/mountains/evaluate_flight.py \
  log/yopov2-tracking/side-observe-20261116-02/probe
python3 deployment/target_tracking/airsim/mountains/evaluate_camera.py \
  log/yopov2-tracking/side-observe-20261116-02
```

汇总：`log/yopov2-tracking/side-view-analysis-20261116/validation.json`；
完整证据索引：同目录`evidence-sha256.json`（768工件），含全类别回放、最终运行时审计。
每轮源码快照优先于交付后新增的checks元数据与离线评估脚本，不把收尾改动冒充已飞源码。
Mission逻辑7项通过、0跳过；Python编译及3份Skill格式校验通过。实际4次飞行与2次
真实CUDA静态失败分别报告，不以逻辑测试代替实际识别验收。

过程：`skill/temp/yopov2-tracking/SIDE-VIEW-20261116.md`。
回顾并完善platform-sensor-adaptation、sim-e2e-validation、component-deployment正式Skill，
依据为护栏失败/修正、物理相机与TF核对、误类别框及清理证据；未创建新Skill。
