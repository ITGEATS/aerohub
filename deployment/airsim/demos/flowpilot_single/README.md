# FlowPilot 单机导航 Tool

通过公共 `uav_planner_interfaces/action/NavigateToPose` 与 `drone.navigation.to_point(...)` 使用 FlowPilot，计划由 FollowTimedPath 执行。CUDA worker、ROS Adapter、Planner、AS2在算力侧；独立工作站容器发任务。没有通用跨Planner仲裁，单机首期固定drone0/PX4 system71，限同一台原生Linux主机，AirSim地址仅支持本机回环。

## 支持范围

支持有限坐标点、COMPLETE、部署速度以及不小于5m的请求容差（实际仍保留原5m结果限制）。终端朝向、HOLD、目标流watchdog和在线目标更新明确UNSUPPORTED；需要改变目标先取消并确认停止，再发新Goal。公共入口启用时，私有goal/cancel不允许绕过所有权。status/events保留诊断用途。

原100ms推理期限、PVA/yaw交接门和v<0.2m/s连续1s、10s墙钟停止判据保留。到达触发余量与最终距离分开，Action成功要求STOPPED、新鲜最终位姿及未舍入距离≤5m。工作站任务另判Land、disarm、Offboard释放和多个revision实际生效；Land失败仍独立等待实测disarmed，但未证明落地时不凭odom高度猜测并强制解除武装；导航Action成功不等于已降落。

## 准备

按 [模型说明](../../../../src/basic_function/inference/models/flowpilot/README.md) 准备权重并构建 [组件镜像](../../../inference/flowpilot/README.md)。原生Linux、Docker NVIDIA runtime、真实Vulkan渲染、AirSimNH资源为前置；场景由 `config/resources.local.yaml` 覆盖资源安装路径，默认profile绑定airsim_nh与本目录settings。

复制 `runtime.example.json` 到自己的 `*.local.json`，填写镜像、独立prefix/domain、evidence。路径相对该配置文件解析：若移动配置位置，必须重新填写路径，推荐本机配置使用明确绝对路径；不要将私人配置提交。每个case用新名称，重复证据目录会拒绝。该实例固定使用RPC45471、HIL4780、HTTP18082和Agent8961，不能与同端口旧任务并行；scene-start/start先检查相关端口占用并拒绝冲突，不接管或停止已有任务。测试前安排受控测试窗口或独立环境。

```bash
python3 deployment/airsim/demos/flowpilot_single/run.py --config PATH_TO_RUNTIME_LOCAL_JSON --dry-run start
python3 deployment/airsim/demos/flowpilot_single/run.py --config PATH_TO_RUNTIME_LOCAL_JSON scene-start
python3 deployment/airsim/demos/flowpilot_single/run.py --config PATH_TO_RUNTIME_LOCAL_JSON start
python3 deployment/airsim/demos/flowpilot_single/run.py --config PATH_TO_RUNTIME_LOCAL_JSON check
python3 deployment/airsim/demos/flowpilot_single/run.py --config PATH_TO_RUNTIME_LOCAL_JSON --case ground1 ground
python3 deployment/airsim/demos/flowpilot_single/run.py --config PATH_TO_RUNTIME_LOCAL_JSON --case flight1 flight
```

先替换配置文件占位路径。flight内部先执行正式preflight，非零即拒飞；人工直接调用mission.py不提供该保护。启动后worker可能需要较长加载/校验，等待真实就绪再检查。GPU计算与渲染同驻时记录空闲后首请求和后续期限。

运行只挂独立输出、settings场景输入；权重已在镜像内，不挂仓库/venv/overlay。新入口不依赖旧服务器任务目录。开发用build.sh另有组件workspace，仅用于日常迭代，不是自包含复现证据。

## 故障、复位与清理

在有效revision后，用独立终端执行 `pause-depth` 注入实际深度中断，执行 `resume-depth` 恢复，注入操作带容器身份及时间线落盘。保持PX4/clock不暂停，按运行前判据核对STALE_OR_INVALID_INPUT、停止、降落与恢复。取消从拥有任务的NavigationModule发出；`--case cancel1 cancel-flight` 会先预检、等待至少两个有效revision后注入取消，单独写fault-result.json并检查ACK、正确原因及完整恢复。正常mission的success=false不会被改成true；只有独立fault_passed才使该故障命令退出0。两类故障的真实平台执行仍待复验。深度中断的独立评分当前需依据保存的停止/恢复时间线核查，不能把任意失败当通过。

平台已解除武装后，执行 `stop`：先采集新鲜disarmed证据，逐个保存并回收本任务标签容器，再经场景manager停止本任务场景。空中或状态不明时拒绝清理，先恢复任务和降落。start失败的现场保留，不自动杀共享进程；未形成平台状态时需要按任务身份诊断回收，不能绕过地面门。

`reset-round` 按stop→scene-start→start串行完整轮换：先地面确认，停止本任务栈和场景，再重新启动场景及PX4/compute；健康依据本次启动日志，失败即停止后续步骤并保留现场。复位入口已实现但尚未在平台实测，不应连续盲跑；AS2残留按相同完整轮换收口。

观察画面按共享实现持续跟踪实时质心俯视，模型flowpilot_front相机保持原深度几何；鼠标自由出窗和实际有屏渲染需要现场验证。不能用离屏替代仓库要求。

## 验证状态

见 [正式验证说明](../../../../docs/flowpilot-validation.md)。现有本地检查不替代Humble实际Action、镜像构建、CUDA和AirSim/PX4。当前组件保持partial；服务器正在运行的旧版本未被本次调整改变。
