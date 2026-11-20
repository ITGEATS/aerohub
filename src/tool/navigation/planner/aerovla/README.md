# AeroVLA — 架构分析与迁移契约（阶段 0）

上游：https://github.com/XuPeng23/AeroVLA @ `b87aaa7c9338ae878d39300a1ef930c753e1ff90`（main，Apache-2.0，纯引用不修改；子模块锁定待可用网络）。论文：AeroVLA（原名 AerialVLA），ECCV 2026。分析基线为 raw 抓取源码 `log/aerovla/upstream-src/` 与 tar 快照 `log/aerovla/upstream-snapshot/`（均 gitignored，事实以锁定提交为准）。

## 1. 能力本质（源码核实）

端到端**策略型 Planner**：OpenVLA-7B（SigLIP+DINOv2 视觉 + Llama-2 7B，bf16）+ PeftModel LoRA adapter（`aerovla_wrapper_ui.py` 加载后不 merge）；每导航步一次自回归推理（greedy、`max_new_tokens=20`、无采样），**纯反应式**（无历史缓冲）。输出连续 3-DoF 位移 + 内禀落地信号，无中间航点、无外部检测器。

### 输入契约

| 项 | 事实 | 源码 |
| --- | --- | --- |
| 视觉 | 5 目装置 `RGB_FOLDER=['front','left','right','rear','down']`；仅取 [0]前视+[4]下视，各 256×256 FOV90 → bicubic 224×224 → 垂直 mosaic 224×448 → openvla image processor | `aerovla_wrapper_ui.py` L86-143 |
| 语言 | `<image>\nFly {hint}and find the target. {obj_desc}\nAction: `；`obj_desc` 从数据集 instruction 按 `"degrees from you."`/`" Please control"` 切分（强依赖 TravelUAV instruction 模板） | 同上 L93,L146-150 |
| 模糊方位 hint | **GT 目标坐标** + 机体位姿 → 机体系 atan2 → 7 档（±15/±60/±120/±180°）；水平距离 <0.01 m → 空串。"消除 oracle"仅指不逐步给指令；目标坐标仍是运行时输入 | 同上 L46-76 |

### 输出契约

- 文本尾 3 个整数 bin → 反量化 `bin/98*(max-min)+min`：forward∈[0,5] m、down∈[−5,5] m、yaw∈**[−1.1,1.1] rad（代码值；论文 ±π 不符，以代码为准）**。
- 落地（双条件）：`"LAND"`/`"<LAND>"` 出现在文本，或 fwd<0.01 且 |down|、|yaw|<0.01。

### 执行映射（`AirVLNSimulatorClientTool_AeroVLA.py` `move_path` L301-395）

世界系位移 (fwd·cos(ψ_cur+Δψ), fwd·sin, down)；|Δψ|≥0.25 rad 或零位移 → `rotateToYawAsync`+`moveToZAsync(v=2)`（**大转向步不做水平位移**，分支逐字保真）；|Δψ|<0.25 → `moveByVelocityAsync`（<1 m：v=d/1 s；≥1 m：单位向量×1.0 m/s 巡航，绝对 yaw_mode）；步末 0.5 s 悬停；返回同一末状态×5。SimpleFlight+LockStep+ClockSpeed 10。

### 评测判据（`closeloop_util.py`/`env_uav.py`/`utils/env_utils_uav.py`）

`maxWaypoints=200`、batchSize=1；成功=停止时距目标 ≤20 m（早停但 >20 m 记 early_end；途中曾入 20 m 记 oracle_success）；碰撞=深度检测（Assist）或 xy 位移 <0.05 m 连续 >15 步（stuck）或目标距离单调增 10 帧；指标 SR/OSR/NE/SPL（`scripts/metric.sh`）。

## 2. aerohub 职责映射（按 2026-11-19 裁决）

| 上游部件 | aerohub 归属 | 说明 |
| --- | --- | --- |
| openvla-7b+LoRA 推理 | **板端 RoboPi** `src/basic_function/inference/models/aerovla/houmo/`（复用已调通 HMM 产物，model-migration-houmo 口径） | 推理角色在算力板；bf16 数值基线不可执行（无可用 GPU 主机），以 golden 复跑为数值门槛，逐张量对照保留为缺口 |
| mosaic/hint/prompt/反量化/落地判别 | `src/tool/navigation/planner/aerovla/ros2_adapter/`（Tool/Planner 层） | prompt 逐字入 `prompts.py`，AST 防漂移 |
| 每步位移执行 | `TimedTrajectorySegment` → FollowTimedPath（决策 3） | 上游速度环 vs PX4 位置控制：动力学偏离记录 |
| 前视/下视 RGB + 位姿 | 传感器桥扩展（Blocks settings 增 FrontCamera/DownCamera 256×256 FOV90） | citynav 多目装置经验复用 |
| TravelUAV UE4 环境 + 场景服务器 | **v2 裁决不使用**：场景=本机 Blocks 内置；目标 GT 坐标由 mission 提供 | 策略 OOD 风险声明见迁移计划裁决 2 |
| 任务入口 | "半语义"Tool：description + **目标坐标**（决策 4） | 卡片如实声明 GT 依赖 |

## 3. 待核查条件（进入阶段 1 前必须闭环）

1. 板端 HMM 服务的输入输出接口形态（此前调试轮的 runner/服务入口、固定张量对照能力）。
2. bf16 基线可运行性：服务器 conda py3.10 环境、openvla-7b+LoRA 权重完整性、flash_attn 与 4090 兼容。
3. TravelUAV town05 环境与 `dataset_raw`/`merged_data.json` 在服务器上的就绪状态（历史 tmux 记录显示已跑过）。
4. 上游基线闭环的推理通路：eval 循环替换 wrapper 为板端客户端后的时延包络（板端 NPU 单步推理时间）。
5. 跨机 DDS（服务器容器 ↔ 板端 ROS adapter）发现与 QoS 实测（SPF 轮 WSL2 风险结论不适用于原生 Linux，需重新核查）。
6. `PhysiceEngineName=ExternalPhysicsEngine` 在服务器环境的实际生效性（settings 拼写即上游原文）。

许可证：Apache-2.0，prompt/逻辑逐字迁移合规。训练复现不在本期范围（评测复现）。
