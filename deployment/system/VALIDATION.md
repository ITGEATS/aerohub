# System Tool 初版验证（2026-11-16）

状态：构建、单元测试和真实 ROS 替身测试通过；尚未完成仿真飞行验收。

2026-11-16 已执行：

- 固定 `onboard_computer:v0.1.1` 中独立工作区重编四个包：
  uav_motion_resource、uav_system_interfaces、uav_system_python_api、uav_system_tool。
  本轮增加查询自身 pose/twist，保留定位 frame/stamp；最终源码已重编。
- 宿主6项单元测试通过：参数拒绝、互斥、确认停止后交接、不同车辆隔离、
  跨进程排他及进程退出后残留占用不自动释放。
- `bash deployment/system/test.sh`：7项真实ROS传输测试全部通过。
  包括起飞/降落/查询（含自身位姿）、Navigation占用拒绝System、System拒绝竞争占用、
  取消原生Behavior并回滚地面解锁、原生失败、非法参数和陈旧输入保留占用。
  测试使用可控平台/Behavior替身，不是PX4动力学或真实飞行证据。
- 修复取消与原生Action提交响应的竞争：清理时先等待已提交的原生Goal响应，
  再取消并等待终态；不能确认时仍保留占用。

原始本轮构建/ROS测试输出保存于 `log/weststreet-demo/system-build.log`、
`log/weststreet-demo/system-ros-tests.log`。此前审批服务403曾阻塞测试，
本轮审批已通过并实际执行，旧阻塞不再适用。

待执行：SUPER更新接口和双方竞争的完整回归；有屏ProjectAirSim＋PX4 SITL中
通过System Tool起飞、稳定、降落、实际landed/disarmed及资源释放；超时、取消、
输入失效的仿真证据仍缺。此前West的原生Behavior成功不替代本版Tool验收。

卡片校验：新增 `cards/tool/system.yaml` 无报错；全仓库校验仍因既有
`cards/inference/diffphys.yaml`、`cards/inference/yolo26n-seg.yaml` 和
`cards/planner/gcopter.yaml` 的旧schema字段失败，未修改这些任务外卡片。
原始输出保存于 `log/system-tool/card-validation.log`。

## 真实 WestStreet/PX4 门（2026-11-16）

`log/weststreet-demo/live-system-04/` 实际执行工作站System客户端 → 算力System
→ AS2 Takeoff/Land → PX4 → ProjectAirSim：两项Action均SUCCEEDED，stop_confirmed，
Land确认landed/disarmed；PX4最终arming_state1，所属服务回收。
离线 `evaluation.json` 记录2152个里程计样本，最大相对爬升2.9605m，最大速度0.5681m/s。
这证明本轮正常起降链，不代表目标追踪、完整故障恢复或多轮可靠性通过。

保留失败轮live-system-03：AS2平台服务同步等待PX4期间状态发布暂停触发保护，
Tool保留占用；以独立PX4 commander land恢复，不计System降落通过。
修复先收口准备服务并重新取得原0.75s门内状态才发起Behavior；ROS测试扩展到9项
（含服务暂停与服务中取消）全部通过，未提高运行新鲜度门限。
