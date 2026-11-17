# SPF 语言导航 Demo（spf_language_nav）

算法组件：`src/tool/navigation/planner/spf/`（契约与状态见其 README）。
迁移方案为本地工作文档（`.trae/documents/`，不随仓库分发）；接口、偏离与
验收边界以本 README、`src/tool/navigation/planner/spf/README.md` 和 [validation.json](validation.json) 为准。

## 拓扑（WSL2 适配四方）

| 角色 | 实际运行 |
| --- | --- |
| 仿真器 | Windows 宿主 Blocks.exe（本 Demo settings 变体写入 Documents\AirSim，原文件自动备份） |
| 飞控板 | WSL2 dockerd：flight-board v0.1.1 `vehicle` 容器（PX4 SITL + Agent，传感器关闭）+ `clock` 容器 |
| 传感器桥 | WSL2：flight-board 镜像独立 sensor 容器（FrontCamera 1280×720@10Hz，rpc 指向 Windows 宿主 IP） |
| 算力板 | WSL2：onboard-computer v0.1.1 `compute` 容器（AS2 + FollowTimedPath + `uav_spf_planner`） |
| 工作站 | WSL2：`proxy` 容器（osrf/ros humble desktop，`uav_spf_vlm_proxy`，出公网调智谱）+ `mission.py` 一次性容器 |

与 Linux 原生入口的偏离（记录于 skill/temp/spf-migration）：
Blocks 经 Windows interop 启停而非 manager.py；**每次实验 Blocks 必须先重启，PX4
是第一个 TCP 客户端**（单客户端槽位）；`AIRSIM_HOST` 用 WSL 默认网关 IP。

## 运行（全部在 WSL Ubuntu-22.04 内）

```bash
cd /mnt/c/Users/93290/Desktop/无人机/aerohub
export OPENAI_API_KEY=<智谱 key>   # 或 ~/.spf_vlm_api_key 文件
python3 deployment/airsim/demos/spf_language_nav/run.py prepare --output log/spf/pc-e2e/sitl-01
python3 deployment/airsim/demos/spf_language_nav/run.py start   --output log/spf/pc-e2e/sitl-01
python3 deployment/airsim/demos/spf_language_nav/run.py mission --output log/spf/pc-e2e/sitl-01 \
    --case ground --instruction "fly toward the orange ball" --max-cycles 8
python3 deployment/airsim/demos/spf_language_nav/run.py stop    --output log/spf/pc-e2e/sitl-01
```

故障用例：`--fault cancel|stale|vlm-down`（mission 运行中由 run.py 注入并保留恢复证据）。
评分目标 `--target` 默认 OrangeBall 真值 (34.25, 33.10, -3.68)，仅用于打分。
`start --observe` 启用质心随动俯视观察视图（复用 `deployment/swarm/` 基础设施；
mission 期间 observer 容器记录 `overhead/track.jsonl` 与俯视帧作为随动证据；
健康检查仅证明相机连通摆放，随动成功只以任务期质心序列移动为准）。

## 验收状态

**部分验证通过（最小端到端验收，2026-11-16）**：M5=1 指令 ×1 完整实例
3 循环连续、无碰撞、task_complete 收口（`run-20261116z6/formal1`）；
M6=cancel 注入正确收口（`run-20261116z7/cancel1`；stale/vlm-down 未执行）；
M7=质心随动证据（track.jsonl 质心移动 30.5m + 俯视帧目视居中确认）；
M8=算力角色 onboard-computer 容器完成 + import 闭包审计
（`log/spf/pc-e2e/container-acceptance-20261116.json`）。
详见 [validation.json](validation.json)。**不声明"复现成功"**（全矩阵
3 指令×3 实例、逐步误差统计、stale/vlm-down 注入、板端部署未执行）。

已知运行约束（证据与修复链见 validation.json attempt_history）：
同一 Blocks 会话仅第一个 mission 可靠 arm（teleport 后 PX4 GPS drift
preflight 拒锁数分钟）——每个用例独立 start→mission→stop；
该 Blocks 构建无 `simGetCameraPose`/`resetLastCollision` RPC，碰撞检测用
`time_stamp` 比对；触地/粘滞标志不计入导航期碰撞门控。

## 已知限制

- 智谱免费档延迟与 JSON 稳定性（D11）：闭环指标解释力受限，论文数值不可对标；
  重试退避 5s/连续失败上限 6 为容错配置。
- Blocks 场景与论文 TravelUAV 差异：目标语义（orange ball）替代 crane 类指令；
  橙球贴地导致 VLM 语义深度下降指令蹭地，执行层加高度下限 D12（min_altitude_m 1.2）。
- 单机；多机外溢声明见 `src/tool/navigation/planner/spf/README.md` §6。
- test_ros_planner.py 独立 DDS 生命周期测试未建（实际由 live smoke/formal 任务覆盖，如实记录）。
