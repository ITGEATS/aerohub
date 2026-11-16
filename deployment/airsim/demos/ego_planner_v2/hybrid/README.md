# RoboPi Drone0 + PC Drone1 混合部署

Drone0 的 EGO、每机 Adapter、导航输入和 AS2 在 RoboPi；Drone1 的同类能力在 PC 的独立容器，由一个独立线程监管。PC 还运行 AirSim、两套 PX4 SITL/传感器桥、时钟和 ROS 任务客户端。共享 `deployment/swarm` 提供拓扑解析及实时质心相机，应用入口保留自己的安全停止条件。

本入口只支持这一已验证拓扑、两机命名及 UDP 映射方案。原 PC 集中式 Demo 和 RoboPi 单机入口保留；同板双算力节点、宽松长航程/队形演示不在本入口中。

## 构建与安装

使用上一级 [构建入口](../README.md) 创建独立外部 EGO 工作区：

```text
python deployment/airsim/demos/ego_planner_v2/run.py build --workspace <absolute-external-workspace>
```

本次 FollowPath 退出修复直接进入 `src/behavior/follow_path`，由既有构建入口编入 ground_overlay；不需要历史 hybrid_lifecycle overlay 或固定命名 volume。运行前比对工作区 source-manifest 与当前源码，缺失或不匹配时先重新构建。

RoboPi 复用既有 OHOS 构建、AS2 runtime 和 `build_lib.sh ego-planner-v2` 入口，再按上一级说明准备独立候选。执行层源码变化需要重新构建 AS2 runtime 中的 FollowPath；只复制 Python 文件不能更新该 ELF。部署包不包含本目录任务、故障注入或审计脚本；故障测试仅按会话显式传入。

历史最终故障验收使用 `ego-v2-lifecycle2-20261113`，其安装文件与前两项一致；更早短程、交叉使用此前候选。新交付候选 `ego-v2-pr1` 与当前 gitlink 的板端重编译/飞行尚未验收，不能把改名视作验证。参见 [验证范围](../VALIDATION.md)。

## 配置和准备

复制 `config.example.json` 到本目录的 `config.local.json`（Git 忽略），填入实际 candidate、HDC target、PC/板端地址、AirSim TCP 目标地址和独立 DDS domain。candidate 须与部署时的 --as2-root 一致，为 /data/aerohub/validation 下的独立目录；示例候选名不绑定本机或历史测试轮。用绝对路径 `pc_workspace` 或 `EGO_PC_WORK` 选择前述工作区；共享 swarm 总是来自当前仓库，不再引用另一份 AeroHub checkout。

`camera_host` 可选择宿主可达的 AirSim RPC 地址，默认 127.0.0.1；容器使用 `airsim_host`。相机客户端复用仓库 deployment/robopi/vendor 的有来源记录的 msgpack 纯 Python 实现，无需迁移副本 vendor 或本机扩展。UE 需要已有双机配置和 `swarm_overhead` 外部相机；prepare 输出 settings.json 供核对，不自动覆盖场景或重启 UE。

`mission_stage` 为 short、crossing 或 fault。故障阶段另设 `fault_vehicle` 为 drone0/drone1、`fault_case` 为 cancel/planner_exit/cloud/odom。任务参与者默认18，避开PC compute的6..17；故障包装仅分配一次DDS参与者。未知或混合无效选项在 prepare 阶段拒绝。

`allow_clock_sync` 默认 false。仅在明确获准对时后设置 true，仍须新鲜未解锁反馈；clock_sync_margin 只决定维护触发点，飞行时差门槛保持0.5秒。入口不会改防火墙、系统网络、Docker Desktop 或共享镜像。

```text
python deployment/airsim/demos/ego_planner_v2/hybrid/run.py prepare --config <local.json> --output <new-session>
python deployment/airsim/demos/ego_planner_v2/hybrid/run.py ground --output <new-session>
python deployment/airsim/demos/ego_planner_v2/hybrid/audit_ground.py <new-session>
```

prepare 只生成文件并在现有镜像中离线解析共享拓扑，不访问板端或发送运动命令。ground 会部署本轮会话、启动两侧服务、完成独立40秒观察和质心相机验证，再安全停止、导出日志。地面检查通过只证明对应输入/通信范围，不证明规划运动。

## 显式飞行验收

飞行使用新的 session，prepare 后执行：

```text
python deployment/airsim/demos/ego_planner_v2/hybrid/flight_run.py <new-session>
python deployment/airsim/demos/ego_planner_v2/hybrid/audit_flight_session.py <new-session>
```

flight_run 会先地面检查，再原生起降 basic，最后执行配置选定的 short/crossing/fault。它会解锁和移动仿真飞机，应在明确授权及场景就绪后使用。保持原时钟、PVA、距离和停止判据。规划输入故障不作用于 AS2 定位；健康机继续原目标，故障机停止后由客户端发起原生 Land。

停止部署不等于取消飞行。正常任务或恢复先确认停止、降落及 disarmed，部署层随后按 EGO/Adapter → AS2 顺序退出并检查子进程。任一安全停止未确认时保留 PX4/时钟，不能直接 compose down。

## 中断后的检查、收集与回退

板端用 `../robopi/run.py status|stop|collect --output <session>` 管理本会话。PC compute 名为 `ego-hybrid-compute-<run-id>`：先读取 `/runtime/status.json`，安全恢复后写 `/runtime/stop.request`，等待容器退出并核查 `/runtime/stopped.json`，再将 `/runtime/.` 复制到本轮 `pc-runtime/`。两侧安全停止证据齐全后才结束本轮 compose 并移除 compute 容器。未知状态不能当作空闲。

回退先停止本轮，保留日志，再选择保留候选及与其源码匹配的外部工作区，新建会话；不热替换运行库，不续接旧规划任务。默认和系统安装保持原样。

## 离线检查与历史证据

`tests/test_entry.py` 检查配置、路径、参与者归属和实际故障 launch；在容器中从 `/input/deployment/airsim/demos/ego_planner_v2/hybrid/tests/test_entry.py` 调用，以保留完整仓库目录层级。`tests/test_plan.py` 需要 prepare 的 `/session`、本仓库 `/input`、`pc/settings.snapshot.json` 挂载为 `/snapshot.json`、robopi 目录为 `/single`，在已加载本工作区 ROS 环境中运行。网络 none 的真实 Runtime 构造不启动 launch 或发运动指令。

`tests/test_flight_audit.py`、`test_crossing_audit.py`、`test_fault_audit.py` 分别读取 `/frozen` 的历史短程 mission、交叉 mission、完整故障 session；原档案从证据维护者获取，只读挂载，测试在临时目录改造副本。设置 PYTHONPATH 为 /hybrid:/single。这些测试验证审计逻辑，不代表新一轮飞行。

精简审计见 [混合证据索引](../evidence/hybrid/summary.json)；大日志、原始失败轮与源码快照仍在原归档。8项故障有成功会话，但经历失败和复验；输入间歇和长期稳定性未解决。当前工作站客户端仍以固定 onboard 镜像作 ROS 工具环境，尚未迁入通用 osrf 工作站镜像，不宣称此约定已完成。
