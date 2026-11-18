# CityNav 语言导航 Demo（citynav_blocks）

算法组件：`src/tool/navigation/planner/citynav/`（契约与状态见其 README 与
`log/citynav/contract/result-20261117.md`）。迁移方案为本地工作文档
（`.trae/documents/`，不随仓库分发）；接口、偏离与验收边界以本 README、
planner/proxy 包 README 和 [validation.json](validation.json) 为准。

## 拓扑（四方，Windows 宿主 + Docker Desktop 适配）

| 角色 | 实际运行 |
| --- | --- |
| 仿真器 | Windows 宿主 Blocks.exe（本 Demo settings 变体写入 Documents\AirSim，原文件自动备份；经 schtasks 计划任务启动） |
| 飞控板 | Docker Desktop：flight-board v0.1.1 `vehicle` 容器（PX4 SITL + Agent，传感器关闭）+ `clock` 容器 |
| 传感器桥 | flight-board 镜像独立 `sensor` 容器：City* 五目阵 10 路图像（5×rgb + 5×depth_planar，256×256@5Hz）+ camera_info + 静态 TF |
| 算力板 | onboard-computer v0.1.1 `compute` 容器（AS2 + FollowTimedPath + `uav_citynav_planner`） |
| 工作站 | `proxy` 容器（osrf/ros humble，`uav_citynav_llm_proxy`，出公网调智谱）+ `mission.py` 一次性容器 + `--observe` 时的 observer 容器 |
| 推理服务 | Windows 宿主 GPU：grounded_sam :8100 / clip :8101（主任务维护，本 Demo 只消费并健康检查，不停止） |

与 WSL 版 spf_language_nav 骨架的偏离（记录于
`skill/temp/citynav-migration/2026-11-17-phase6-demo.md`）：

- 全部编排在 Windows 宿主原生运行（docker CLI → Docker Desktop 引擎；
  powershell → schtasks/netstat），不再依赖 WSL interop；
- 容器访问 Windows 宿主（Blocks RPC/TCP、GPU 服务）的地址由 `start` 时的
  `host_probe.py` 实测选定（host.docker.internal 或 vEthernet IPv4，需同时
  打通 RPC TCP 与两服务 /health/ready），写入 `topology.json`；
- planner 参数文件中的 `__HOST__` 占位符由 run.py 替换后生成
  `<output>/planner.yaml`，compute 与 proxy 都加载该生成副本；
- 起点偏离：任务书起点 (-10,-10,-8) 为 AirSim NED（z 负=上）。空中投放会在
  解锁前自由落体碰地，故按 SPF 已验证模式落地为**地面 teleport (-10,-10) +
  原生起飞 1.5 m**；评分目标 OrangeBall NED (34.25,33.10,-3.68) 换算为 odom
  ENU (34.25,33.10,+3.68)；
- waypoint 段判定口径（跑前锁定）：每探索循环发送一个 FollowTimedPath goal
  （revision+1），goal 内时间化点即 15°/5m/2m 分解段；门 = 前 revision 个
  cycle 事件的 steps 合计 ≥3 且无 segment_failed，并以
  `/drone0/follow_timed_path/_action/status` 的 SUCCEEDED 计数交叉验证。

## 运行（PowerShell，仓库根目录）

```powershell
python deployment/airsim/demos/citynav_blocks/run.py prepare --output log/citynav/pc-e2e/build-XXXX
python deployment/airsim/demos/citynav_blocks/run.py start   --output log/citynav/pc-e2e/run-XXXX [--observe]
python deployment/airsim/demos/citynav_blocks/run.py health  --output log/citynav/pc-e2e/run-XXXX
python deployment/airsim/demos/citynav_blocks/run.py mission --output log/citynav/pc-e2e/run-XXXX `
    --case formal1 --instruction "fly toward the orange ball and stop near it"
python deployment/airsim/demos/citynav_blocks/run.py stop    --output log/citynav/pc-e2e/run-XXXX
```

智谱 key 从 `src/tool/navigation/planner/citynav/legacy_stub/.env` 读取并经
`docker -e` 注入 proxy 容器，不落库。GPU 推理服务由主任务在宿主运行，
`start` 的 host_probe 与 `health` 会检查其可达性；本 Demo 不启动也不停止它们。

预留开关（本轮不启用）：`--cancel-after N`（goal 后 N 秒注入 `citynav/cancel`）、
`--use-memory 0|1`（goal payload 记忆开关，跨任务 exploit 复用轮使用）。

## 已知运行约束（沿用 SPF 实测结论）

- 每个 Blocks 会话仅第一个 mission 可靠 arm（teleport 后 PX4 GPS drift
  preflight 拒锁数分钟）——每个用例独立 start→mission→stop；
- 该 Blocks 构建无 `simGetCameraPose`/`resetLastCollision` RPC；碰撞检测用
  `time_stamp` 比对法，触地/粘滞标志不计入导航期碰撞门控；
- 观察视图复用 `deployment/swarm/` 基础设施；`--once` 健康检查仅证明相机
  连通与摆放，随动成功只以任务期 `overhead/track.jsonl` 质心序列移动为准；
- 相机名两侧包默认值不同（planner `LeftCamera*` vs proxy `CityLeft*`），
  本 Demo 在 planner.yaml 统一为 City*（`test_config_contract.py` 防漂移）；
- `start` 后须预热 ≥3 分钟再 `health`（UE4 首帧 shader 编译期 RPC 3 s 超时，
  run d 教训）；容器→宿主地址由 `host_probe.py` 实测 IPv4 优先
  （host.docker.internal 解析 IPv6，PX4 C++ TCP 无 v4 回退，run a 教训）。

## 运行前提（宿主 GPU 服务）

- 宿主 grounded_sam 服务必须以 **`GS_EMPTY_CACHE=1`（默认开）** 运行
  （warmup 与每次推理后 `torch.cuda.empty_cache()`，reserved 稳态
  ~3.7 GB）；否则 UE4 Blocks 在推理突发中被 WDDM 驱逐显存崩溃
  （2026-11-17 run c/f/g/h/k/l 实证）。clip 服务已转 CPU。两服务由主
  任务维护，本 Demo 只消费与健康检查，不启动、不停止、不重启。
- GS_EMPTY_CACHE 的代价：空闲期后首个 grounding 推理 ~78 s（allocator
  重扩容，实测 `log/citynav/pc-e2e/gs_probe.py`），故 planner.yaml 将
  `service_timeout_s=120`、`retry_backoff_s=20`。
- Blocks 窗口固定 `-ResX=1024 -ResY=1024 -ExecCmds=r.Streaming.PoolSize 128`：
  窗口任一边 <1024 时与 `--observe` 俯视相机 1024×1024 捕获互斥，UE4 4.27
  scene RT 每帧重分配触发 BitArray 断言崩溃（run g/h/i/j 启动期即崩实证）。

## 验收状态

**formal1 通过（最小端到端验收，2026-11-18，`run-20261118g/formal1`）**：
1 指令 ×1 完整实例，10 门全 PASS——readiness/teleport/takeoff、
waypoints（2 cycles、1 goal SUCCEEDED、12 时间化步 ≥3）、revision_match、
stop_state、final_reason=task_complete、land、disarm、collision（0 事件，
时间戳比对法）；`--observe` 质心随动证据 track.jsonl 293 条 + 移动 49 m +
28 俯视帧。记录项：NE 33.6→11.8 m（ratio 0.65，无门槛）、144.6 s。
详见 [validation.json](validation.json)（含 11 次尝试全链与 5 个真 bug
修复记录）。**口径为部分验证通过（最小端到端验收），不声明"复现成功"**：
跨任务记忆 exploit 复用（见下 BLOCKED 归因）、全矩阵、板端部署未通过/
未执行；AirVLN 基准范围外（路线 B 裁决）。

起点偏离（Demo 层适配，门槛未动）：默认 spawn 由任务书建议 (-10,-10)
改为 **(10,10,yaw45)**——(-10,-10) 距橙球 62 m 超契约 `0<d<50 m` 深度门，
球像素全被滤除，grounding 误分割 50 m 内墙体 → waypoint 系统性穿墙 →
真实撞墙（`run-20261118f` collision FAIL 实证）；(10,10) 球 33.5 m 深度
有效。指令与目标真值不变。

**cancel1 通过（M6 取消收口，2026-11-18，`run-20261118n/cancel1`）**：
事件驱动注入（首个 FollowTimedPath goal 发出后 20 s，`fault-injection.json`
带 `seconds_after_first_goal_sent`/`revision_at_injection` 证据）→ planner
STOPPING → stop 服务 → 速度 <0.2 m/s 持续 1 s → STOPPED
reason=CANCELED detail=cancelled stop_confirmed=true，**未触发 10 s
FAULT 分支**；behavior 侧 goal 终态 ABORTED(6) 于 `_action/status`；
land/disarm/无碰撞全 PASS。前置适配：mission 起飞后悬停稳定门（8 s 滑窗
0.15 m，起飞 z 极限环 ±0.4 m 教训 h/j/l/m）；behavior 拒绝日志补数值诊断
（`src/behavior/follow_path`，重建 overlay，属诊断改进非门槛放宽）。

**exploit1 阻塞（决策 6 跨任务记忆复用，2026-11-18，见 validation.json
exploit1 节）**：链条已实证部分——memory_loaded(nodes=2, run g 图注入) →
find_closest_node 命中既往 node0（until=floor 修复生效）→ CLIP CPU 服务
真实评分 2.9 s → LM-Nav find_optimal_route 执行 → 退化 walk 收口干净
（`run-20261118o`）。阻塞层：①感知模型——CLIP 对既有两节点分数差 0.003
且方向反（node0 0.1965 > node1 0.1935，`log/citynav/pc-e2e/clip_diag.py`）
→ argmax=start → walk 无执行段；②飞控——起飞后 z 极限环使所有"先飞出去
建新节点"的扩展任务首个 goal 被连续性门拒绝（q/s/u 三轮，诊断数值
actual z=1.064 vs front z=1.463）或触地（q/s）；③仿真器——Blocks 启动期
死亡频率上升（i/r/t）+ 63 m teleport EKF 不收敛（p×2）。按批准预案投入
6 个 Blocks 会话走"先扩节点再复用"路径未突破；**门槛未放宽，本轮记
BLOCKED 而非通过**。

**z 悬停摆动归因（2026-11-18 选项 A 裁决，validation.json z_limcycle 节 +
`skill/temp/citynav-migration/2026-11-18-z-limit-cycle.md`）**：双源采样
证明摆动在 **PX4 EKF 高度估计侧**（~0.1 Hz、真值 p2p 0.9-1.9 m、恒定
+0.63..0.67 m 偏置；AS2 指令恒定时机体仍摆，kp 1.0→0.05 幅值不变），
非 AS2 增益/相机负载/planner。Demo 层处置：controller.yaml 保持 SPF
验证值（归因写入注释）；mission 起飞 2.5 m + 速度判据悬停门；planner
发送前 `_wait_quiet()` + 新鲜位姿重锚路径起点（直接修复 q/s/u/y 的
连续性拒绝根因——旧代码用推理前 60 s 的决策位姿作首点）。验证
（run-20261118z）：4 goal 全接受、3 SUCCEEDED、33 步执行、无碰撞。
根治需 flight_board 镜像 PX4 参数专项（团队级，超 Demo 权限）。

## 已知限制

- 路线 B：不复现 AirVLN 基准，Blocks 场景语义（orange ball 等）替代城市
  landmark；论文 SR/OSR/NE/SPL/SDTD 不可声明。
- 替代云端模型 glm-4v-flash（D11 同源）：is_found/landmark 抽取稳定性受其
  影响；缓存禁用（M-1 对照）下每帧真实调用。
- 五目阵 10 流 @5Hz 为契约 §4 的增量负载，实测频率/延迟以 health.json 计数
  与 sensor timing 为准（阶段 6 首验）。
- 单机；多机外溢声明见 `src/tool/navigation/planner/citynav/README.md` §6。
