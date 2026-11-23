# RACER 纯 PC 双机分容器复现

新运行默认采用 `per_vehicle`。两架机各用一个固定算法镜像容器运行自己的 RACER 原生探索进程及求解器。车辆身份和容器放置由 [swarm 拓扑](../../../swarm/README.md) 的 `pc_simulation`、`decentralized` 模式解析，启动器按结果分配 worker。AirSim 出生位置和相机仍使用所选场景 settings。每机有独立的 PX4 SITL、AS2、导航和相机服务，输入桥和 Adapter 分别在各自的共享容器内运行双机 worker。工作站负责发起任务和收尾。

本入口用于固定镜像和外部构建 overlay 的 PC 开发仿真。[镜像锁定](../racer_single/images.lock.json)和[构建准备](../racer_single/README.md)沿用现有 RACER 入口。板端部署、交付镜像和探索覆盖率需另行验收。

## 准备

从[本地配置示例](forest-split.local.example.json)复制一份个人 JSON，填写新运行目录、项目名、可用 DDS domain、Docker 子网、容器可达的 AirSim 主机、settings 和 release 路径。将 fleet 和 flight 路径分别指向 [ForestScene 双机配置](forest-complete.fleet.json)和[本次任务配置](forest-split.flight.json)。任务配置相对原完整探索用例仅将深度样本最大年龄从 2.0 秒改为 2.2 秒；相对起飞点高度上限 2.0 米，每机完整探索预算 300 秒。

在 UE 中确认已加载 ForestScene 和匹配的双机 settings，并在上轮两机落地、解除武装及服务停止后重新 Play。入口从宿主检查活动 settings、车辆名与相对出生位置。AirSim 位于另一台宿主时，另填宿主可访问的 `airsim_probe_host`。地图名称由操作者核对。以下 PowerShell 命令从 AeroHub 根目录执行，`$local` 为填好的本地 JSON 路径。

## 运行与停止

```powershell
$demo = 'deployment/airsim/demos/racer_multi_flight'
python -B "$demo/full_flow.py" --local $local --mode explore-complete
```

完整入口执行分阶段启动、双机地面就绪、容器身份核对、飞控日志采集、原生探索、停止、降落和清理。新运行的拓扑写入 `swarm-topology.json`，每机的 Docker ID、镜像与 worker 身份写入 `verified-placement.json`。地面联调可依次执行 `run.py --local $local up`、`sensors`、`survey`、`start` 四个子命令，然后执行 `stop` 和 `down`。每个子命令都使用同一份 local JSON。随后飞行使用新运行目录和新的场景会话。

运行中需取消任务时按[已有的取消与恢复说明](FOREST_COMPLETE.md#检查结果停止与回退)向本轮 mission 发送 SIGINT，等待两机停止、落地及解除武装。若记录器退出异常，保留 `cleanup-result.json` 和原始失败结论；两机落地解除武装且所属 worker 已停止后，使用本轮证据目录的 `local.json` 执行 `run.py --local $local down`。该入口会重新核对 PX4 状态和容器所有权。

需要复现共享算法容器的用例时，显式设置 `"native_placement": "shared"`，选择与该用例相符的 flight 配置。旧版 R2–R5 的故障注入证据来自共享容器拓扑，复现旧用例需显式选用 shared。恢复已有运行时以原运行目录及记录的拓扑为准。

## 验收

```powershell
$case = Get-Content -Raw $local | ConvertFrom-Json
python -B "$demo/evaluate.py" --run-root $case.run_root
python -B "$demo/readback.py" --run-root $case.run_root --release $case.release --output (Join-Path $case.run_root 'recording-readback')
```

读回使用新的输出目录。分别核对每机激活轨迹、AS2/PX4 参考与实际运动、本轮原生结束原因、停止、Land、landed/disarmed，以及 worker 和基础服务退出状态。辅助 bag 持久化或读回失败时保留分项失败。

### 正式入口

双机地面复验中，省略 `native_placement` 后，两架机分别启动独立的 RACER 容器。23 项就绪检查通过，16 个容器均正常退出。

随后用同一飞行配置运行一次 `explore-complete`。两机跟踪约 150 秒，均在 300 秒预算内由 RACER 报告 `native_finished`。AS2 和 PX4 均记录到轨迹跟踪参考，双机停止确认、原生 Land、落地解除武装均通过。16 个容器正常退出，两机辅助录包均通过离线读回。

| 本轮观测 | Drone0 | Drone1 |
|---|---:|---:|
| 激活的轨迹版本 | 16 | 30 |
| 被拒绝的轨迹版本 | 5 | 2 |
| 距探索起点的最大水平位移（米） | 7.00 | 8.18 |

本轮验证了双机执行和原生结束流程。探索区域覆盖效果与多轮重复可靠性仍待确认。运行编号和证据文件校验值见[证据索引](pc-split-validation.json)。

### 早期分容器实验入口

| 实际运行 | 结果 |
|---|---|
| 双机地面联调 | 23 项就绪检查通过，确认两个原生算法容器及机间状态、地图消息。 |
| 深度上限 2.0 秒的首轮完整模式 | Drone1 深度样本年龄达到 2.001 秒，跟踪约 10.2 秒时收尾，两机落地解除武装。 |
| 深度上限 2.2 秒的首轮 | 跟踪约 22.6 秒，Drone1 的 AirSim 相对高度约 2.015 米，触发 2.0 米范围保护，两机落地解除武装。 |
| 2.2 秒同配置复跑 | 跟踪约 142.6 秒，两机各激活 16 个轨迹版本、零拒绝，均为 `native_finished`。最大水平位移 6.43 / 5.65 米，停止、原生 Land、落地和解除武装通过。Drone1 录包少 4,372 条，记录器退出 1，整轮自动验收未通过；随后通过地面状态复核并停止基础服务。 |
