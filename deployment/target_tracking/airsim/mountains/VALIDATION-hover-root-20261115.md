# 原生悬停根因修复验证 — 2026-11-15

结论：起飞/悬停失败已修复；完整目标观察未通过，移动阶段未执行。
无硬件、追踪运动、上游改动、镜像替换或commit。单一代理执行。

| 用例 | 结果 | 证据目录（log/yopov2-tracking/） |
| --- | --- | --- |
| 原配置独立复现 | Takeoff超时；197个起飞样本撞02Armored-V；取消后EMERGENCY导致原生Land拒绝，PX4 failsafe落地解除武装 | hover-root-native-baseline |
| Y6独立原生飞行 | Takeoff/hover/Land/disarm全部通过；实际悬停2.906–3.070m | hover-root-native-fixed |
| Y6、水平相机观察 | 飞行通过；目标被画幅截断，静态门失败，未启动车队；原生Land通过 | hover-root-observe-fixed |
| Y6、-20°相机独立飞行 | 全部通过；32物理悬停样本，2.906–3.075m | hover-root-native-camera-fixed |
| 最终配置完整观察 | 飞行通过；静态门失败，51次真实CUDA空框；未启动车队；原生Land/disarm通过 | hover-root-observe-camera-fixed |
| 原始输入CUDA回放 | truck最高0.01694、train0.53457；阈值保持0.35 | hover-root-analysis/cuda-view-diagnosis.json |

旧证据另有：mountains-hover-observe-02、mountains-hover-audit-20261115。
其原SHA集合复核未变。额外disarmed检查保留在hover-root-ground-*及hover-root-rays-01；
缺少宿主msgpack、未设置图形环境的启动失败也记录在过程文档。

原生锁定position插件以激活时earth z加请求增量生成目标，POSITION→SPEED控制链
持续给向上参考。旧轮实际位姿与估计值的分离、车底图像和新诊断轮动态碰撞对象
共同定位场景出生点问题。初态向上ECC_Visibility射线全部通畅，不能替代动态车辆
实际碰撞证据。修复只改专属飞行出生点与实际相机俯角，桥的外参继续同源读取。

运行前门槛保存在各轮checks.json：3m/0.5m/s，Takeoff40s；高度±0.3m、XY≤0.5m、
速度≤0.2m/s连续2s；静态≥4不同stamp/capture且spread≤1m；移动完整12s。
没有降低门槛、改类别或用真值输入算法。真值仅用于physics-samples离线评分。

复核物理门：

```bash
python3 deployment/target_tracking/airsim/mountains/evaluate_flight.py \
  log/yopov2-tracking/hover-root-native-baseline/probe
# 应失败
python3 deployment/target_tracking/airsim/mountains/evaluate_flight.py \
  log/yopov2-tracking/hover-root-observe-camera-fixed/probe
# 飞行阶段应通过，不能把它视为完整观察通过
```

真实ROS/PX4记录器1项、Mission7项回归通过，无skip；Python编译通过。
最终每轮cleanup.json显示所属容器清空，原有两个容器ID/StartedAt不变。
停车先测三车≤0.1m/s连续5次，再收口观察、原生Land、确认解除武装后清理；
修复后的正常降落没有EMERGENCY或竞争hold发布者。

汇总：hover-root-analysis/validation.json、evidence-sha256.json。
过程：skill/temp/yopov2-tracking/HOVER-ROOT-FIX.md。
更新正式Skill：as2-extension-integration、uav-sim-e2e-validation、
uav-platform-sensor-adaptation，分别补充起飞语义、物理碰撞交叉验证和视场/TF核查。
后续仍需解决模型在当前悬停视角的目标识别，原静态门通过后才能执行移动12s。
