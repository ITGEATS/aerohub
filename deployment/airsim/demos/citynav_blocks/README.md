# CityNav 语言导航 Demo（citynav_blocks）

算法组件：`src/tool/navigation/planner/citynav/`（契约与状态见其 README 与
`log/citynav/contract/result-20261117.md`）。迁移方案为本地工作文档
（`.trae/documents/`，不随仓库分发）；接口、偏离与验收边界以本 README、
Planner README 和 [validation.json](validation.json) 为准。

## 拓扑（四方，Windows 宿主 + Docker Desktop 适配）

| 角色 | 实际运行 |
| --- | --- |
| 仿真器 | Windows 宿主 Blocks.exe（本 Demo settings 变体写入 Documents\AirSim，原文件自动备份；经 schtasks 计划任务启动） |
| 飞控板 | Docker Desktop：flight-board v0.1.1 `vehicle` 容器（PX4 SITL + Agent，传感器关闭）+ `clock` 容器 |
| 传感器桥 | flight-board 镜像独立 `sensor` 容器：City* 五目阵 10 路图像（5×rgb + 5×depth_planar，256×256@5Hz）+ camera_info + 静态 TF |
| 算力板 | onboard-computer v0.1.1 `compute` 容器（AS2 + FollowTimedPath + `uav_citynav_planner`，任务入口 = 共享 `NavigateToSemanticGoal` Action `to_semantic_goal`） |
| Inference | onboard-computer v0.1.1 `inference` 容器：提供 VLM 单图和多图 Action，共用逻辑模型 `vlm` 和一个云 worker；通过 ROS 服务代理 Grounded SAM 与 CLIP 宿主 worker。 |
| 工作站 | `mission.py` 一次性容器（osrf/ros humble，经 Action 调用算力板 Tool 函数）+ `--observe` 时的 observer 容器 |
| 推理服务 | Windows 宿主 GPU：grounded_sam :8100 / clip :8101（主任务维护，本 Demo 只消费并健康检查，不停止） |

与 WSL 版 spf_language_nav 骨架的偏离（记录于
`skill/temp/citynav-migration/2026-11-17-phase6-demo.md`）：

- 全部编排在 Windows 宿主原生运行（docker CLI → Docker Desktop 引擎；
  powershell → schtasks/netstat），不再依赖 WSL interop；
- 容器访问 Windows 宿主（Blocks RPC/TCP、GPU 服务）的地址由 `start` 时的
  `host_probe.py` 实测选定（host.docker.internal 或 vEthernet IPv4，需同时
  打通 RPC TCP 与两服务 /health/ready），写入 `topology.json`；
- `run.py` 只在 `<output>/inference.yaml` 中填入宿主 worker 地址。Planner 通过
  ROS 调用 Inference，不持有 worker URL；云 VLM 与 SPF 共用 `models.vlm.cloud`
  配置和 `deployment/inference/vision_language_run.py` 启动入口；
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

`prepare` 使用 Vision Language 模型的锁文件同步其目录下的 `.venv`，SPF 和 CityNav 从同一个环境启动推理 worker。

运行会话从资源表解析 `airsim_blocks_windows`，指向本机 Blocks `WindowsNoEditor` 目录。临时覆盖可设 `CITYNAV_BLOCKS_PATH`。
云模型、地址和凭据从本地 `config/inference.cloud.local.yaml` 的 `models.vlm.cloud`
取得；配置引用的环境变量按名称转发，不从 legacy stub 读取。GPU 推理服务仍在宿主运行，
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
- 相机名包默认值（planner `LeftCamera*`）与 Blocks settings 的 City* 阵
  不同，本 Demo 在 planner.yaml `view_cameras` 统一为 City*（Tool 化后
  Inference 节点不订阅相机，帧由 planner 随请求提供，命名仅一处配置）；
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

**fault1 通过（M6 LLM 失效路径，2026-11-18，`run-20261118gap/fault1`）**：
持久故障注入——会话 `planner.yaml` 代理节 `base_url` 指向容器内丢弃端口
`http://127.0.0.1:9/v4` + `restart-compute`（任务前生效、全程不可恢复）→
起飞 2.5 m 悬停 → goal → landmark 抽取连续 3 次 `error_code=http`
（代理日志 3×`Connection refused`，间隔 = retry_backoff_s=20）→ planner
FAULT reason=EXECUTION_FAILED detail=llm_failures，**revision=0、
FollowTimedPath goal 从未发出**（无执行段可坠机）→ 悬停保持
（airborne_at_fault z>1 m PASS）→ 原生 Land + disarm + 碰撞 0 全 PASS，
总 149.2 s。fault1 门为 `--expect-fault` 专用口径（waypoints/revision_match/
final_reason 记 NOT-EXECUTED，不静默通过）。证据注记见
`run-20261118gap/fault1/NOTES.md`（fault-injection.json 计数器缺陷已修
mission.py，原始证据不改写）。

**缺口补齐轮（2026-11-18，用户指示"先补全 3 个小验证"）**：
- 容器内 pytest：planner **45 passed + 1 skip**（设计内 node 占位）于
  onboard-computer v0.1.1 容器（networkx 对照库经宿主 wheel 只读挂载，
  镜像未动）；proxy **34 passed** 于工作站镜像——与宿主基线一致
  （`log/citynav/ros/container-pytest-20261118/`）。
- 真实 DDS 生命周期：planner 独立容器 + 工作站检查器 **6/6 PASS**
  （节点发现、3 publish + 21 subscribe + 6 service 契约、IDLE 心跳、
  cancel 幂等、goal→not_ready 收口×2 可重入）+ SIGINT 优雅退出 exit=0
  （`log/citynav/ros/dds-lifecycle-20261118/`，脚本
  `src/tool/navigation/planner/citynav/validation/dds_lifecycle_check.py`）。
- **M-1 代理 vs SDK 同帧对照已执行**：20 组 Blocks 抓帧，strict（is_found+逐
  视角 object）一致率 70%，未达**原**预锁定 90% 门槛；SDK 同路径自对照仅
  80%，证明原门槛与模型随机性混淆。**用户裁决（方案追加裁决二，
  2026-11-18）修订判据**（与自对照差 ≤15pp + is_found ≥90% + 差异全归类），
  按修订判据归档证据判**通过**（Δ=10pp、is_found 100%、差异均为视角重叠区
  抖动与占位符回显两类）；原门槛与 70% 实测结果保留于 validation.json，
  不追溯改判。证据 `log/citynav/llm/m1-20261118/REPORT.md`。

**stuck2 通过（M6 卡死检测注入，2026-11-18，`run-20261118stk/stuck2`）**：
注入尝试先暴露真缺陷——**上游 L534 卡死判据是死代码**
（`calculate_movement_steps` 对 A==B 仍返回 ≥1 的 yaw 占位项，
`sum(steps)==0` 永不可达；单测已锁定该发现）。按迁移方案"保留并升级
FAULT 收口"授权，本移植改喂位移进展位（`build_timed_path` 返回 None 记
零进展），`is_stuck` 函数体不变。注入 = grounding 指向恒
`seg_success=false` 的 host-network 桩容器 + 6-landmark 指令：4 个零进展
循环 → FAULT detail=stuck，goal 从未发出，悬停保持 → Land/disarm/碰撞 0
全 PASS（首轮 stuck1 因 Windows 防火墙拦宿主端口未达注入目标，但顺带
实证了 grounding 不可达 → service FAULT 收口路径，证据保留）。
容器内单测升至 **46 passed + 1 skip**；overlay 已重建。

**exploit 重试轮（2026-11-18，用户批准，B2 口径不改判别逻辑）= 仍阻塞 → 降级为后续项**：
"先扩节点再复用"投入 3 个会话——expand2（Blocks 31 s 死亡）、expand3
（(20,20) 直线走廊穿楼碰撞）、expand4（(10,10) 安全走廊 9 循环 24 步真飞，
但"绕球"语义漂移到 85.9 m 后 segment_failed，未完成即不落盘记忆图）。
**用户裁决（方案追加裁决二，2026-11-18）**：决策 6 的 executed-walk 由
验收必演项**降级为后续项**；executed-walk 证据状态如实保持"未完成"，
本 Demo 与能力卡片持续显式声明"**跨任务记忆复用未获飞行验收**"（建图/
落盘/加载/命中/LM-Nav 搜索各环节均有飞行实证，唯执行末环没有）；判别
信号增强仍列后续项不删除（validation.json exploit_retry_20261118）。

**Tool 化改造轮（2026-11-18/19，追加裁决三"被复现工作的形式是 Tool"）**：
任务入口改为共享 Navigation Tool 函数 `to_semantic_goal`
（`NavigateToSemanticGoal` Action，`max_steps`/`use_memory` 降为部署参数），
云 LLM 改走共享 Inference Function `InferVisionLanguageFrames`（私有
`llm_proxy` 包删除，prompt 逐字迁入 planner `prompts.py` + AST 防漂移；
偏离 D25/D26/D27 见 `ros2_adapter/README.md`）。已闭环：容器单测
**53 passed**、新契约真实 DDS 生命周期 **6/6 PASS**
（`log/citynav/ros/dds-lifecycle-20261118-toolified/`）。**新契约下 formal1
飞行重验通过（run-20261118tool12，2026-11-19，12/12 门全 PASS）**：5 循环 /
3 goal 全 SUCCEEDED / 15 时间化步、无碰撞、`STOPPED task_complete`、
action result `SUCCEEDED + landmark_sequence_complete_semantic_only_no_positional_verification`
（D25 语义 live 验证）、NE 33.5→8.3 m（仅记录）、随动证据 342 条 + 32 帧。
尝试史（tool4–12：4 次 Blocks 宿主死亡、1 次 RPC 停顿已修、3 次穿墙决策族
碰撞 FAIL）与系统性归因见 [validation.json](validation.json)
`toolification_20261118` 节——穿墙分支为供应商侧 VLM 决策随机性 + Blocks
走廊几何（已知 run f/expand3 族），**非 Tool 化回归**，且通过依赖尝试
（4 次飞行尝试 3 次落穿墙分支）已如实记录为残余风险。门槛未放宽。
**cancel1/fault1 新契约重验（2026-11-19 轮，`cancel1_fault1_new_contract_20261119`
节）**：fault1 **通过**（run-20261119f2：inference 容器停机注入 → 3×NO_SERVER
→ FAULT detail=llm_failures，零 goal 发出、悬停 2.54 m 收口、
Land/disarm/碰撞 0 全 PASS）；cancel1 **未通过**——c8/c9/c10 三次、以及
**宿主完整重启后**的 c11 第四次，均为干净飞行后空中取消触发 10 s 停止门
宽限超时（悬停 z 极限环使 |v|<0.2 m/s 静窗 <1 s；契约的 STOP_UNCONFIRMED
路径每次都按规范工作）。重启排除了驱动状态退化假说：阻塞是 flight_board
v0.1.1 配置的系统性 EKF 摆动，待 PX4 调参团队工单落地后重验，门槛不放宽。
本轮另证实：mission 前用真实
帧预热 grounding（gs_warm.py，~2-4 s）可消除冷启动显存扩容坠机族。
stuck2 路径未变，引用旧证据加注。

## 已知限制

- 路线 B：不复现 AirVLN 基准，Blocks 场景语义（orange ball 等）替代城市
  landmark；论文 SR/OSR/NE/SPL/SDTD 不可声明。
- 替代云端模型 glm-4v-flash（D11 同源）：is_found/landmark 抽取稳定性受其
  影响；缓存禁用（M-1 对照）下每帧真实调用。
- 五目阵 10 流 @5Hz 为契约 §4 的增量负载，实测频率/延迟以 health.json 计数
  与 sensor timing 为准（阶段 6 首验）。
- 单机；多机外溢声明见 `src/tool/navigation/planner/citynav/README.md` §6。
