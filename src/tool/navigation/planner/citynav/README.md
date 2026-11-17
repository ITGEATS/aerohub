# CityNavAgent Planner

上游：https://github.com/EmbodiedCity/CityNavAgent.code （ACL 2025，LLM 驱动的空中 VLN：
层级语义规划 HSPM + 全局记忆拓扑图；README 另指向镜像 WeichenZh/CityNavAgent）
子模块锁定提交：`4b6e926c3325995483576523b18c9d0de5eb0710`（main tip，2026-11-17 锁定；
四项关键事实抽查与 gh-proxy zip 快照全部一致，且 `git diff --no-index` 快照↔锁定提交
仅差 `.git` 文件；teleport=setPoses、gpt-4o 字面量 L112、FOV=90 见 L101、步长 5/2/15
见 airsim_settings.py L89-92）

> **⚠️ 许可证记录（如实）**：上游 README 徽章声明 Code License = MIT 并链接
> `CODE_LICENSE`，但**仓库内不存在 CODE_LICENSE 文件（断链）**，也无其他 LICENSE 文件。
> MIT 主张仅来自徽章文本，无许可正文可核查。本仓库以 git 子模块引用（仅 gitlink，
> 不复制源码入库）；适配层若逐字复制 prompt/算法逻辑，依据为徽章声明的 MIT 意图，
> 该缺口已记入过程记录，对外分发前需向上游确认。

> **⚠️ 本期范围（用户裁决 2026-11-17，路线 B）**：**不复现 AirVLN 基准**——不下载
> AirVLN UE4 仿真器与 Google Drive 数据（AirVLN-S/E、作者记忆图 pkl、landmark 标注），
> 全链路在标准 AirSim Blocks + PX4 SITL 验证。结论上限为"管线复现 + 行为定性一致 +
> AS2 链路接入验收通过（部分验证通过）"，不声明、也不能声明论文 SR/OSR/NE/SPL/SDTD。
> 迁移方案与验收矩阵见 `.trae/documents/citynavagent-migration-plan.md`；
> 过程记录在 `skill/temp/citynav-migration/`。

## 1. 上游真实管线（源码追踪，非论文宣称）

启动入口 `python SimRun.py`（`__main__` L679-688：env_id=3、split=val_seen、
max_step_size=60、vlm_name="sam"）。每 episode 主循环（`CityNavAgent()` L324-570）：

| 步骤 | 能力 | 输入 | 输出 | 源码位置 |
| --- | --- | --- | --- | --- |
| 全景取图 | 8 视角 RGB-D，**teleport 旋转取图**（每步 3×TURN_RIGHT=45°），取 6 视角使用 | AirVLN 场景服务器 + airsim RPC `front_0` | 512×512 rgb/depth ndarray + 各视角位姿 | `utils/env_utils.py` L649-675、`SimRun.py` L423-443 |
| 场景描述 | 云端 gpt-4o captioner（base64 图逐视角） | 6 视角图 + `landmark_caption_prompt_builder` | 观测物体集合文本 | `SimRun.py` L150-157 |
| landmark 级 | **直接消费数据集标注** `instruction["landmarks"]`（作者离线产物，入口不做 LLM 抽取） | `data/gt_by_env/{env}/{split}_landmk.json` | 地标序列 | `SimRun.py` L383 |
| object 级 | 云端 gpt-4o 多图路线查询（5 视角带标签图 + 指令 + 地标序列 → 各视角目标物 + is_found） | `route_planning_prompt_builder` | 视角→object 映射 | `SimRun.py` L253-255、`src/llm/query_llm.py` `query_viewpoint_api` |
| 语义点云 | 本地 GroundingDINO SwinT + SAM vit_h `greedy_mask_predict` → 深度反投影（**FOV=90 硬编码**）→ 统计滤波 | RGB + metric 深度 + mask | 机体系→世界系 3D 语义点云 | `SimRun.py` L260-290 |
| motion 级 | 点云类均值 → alpha=0.6 向当前位置收缩 → 高度下限（sam 支 z=5 m，dino 支 z=2 m）→ 动作分解 | 语义点云 + 当前位姿 | 下一 waypoint 位姿 + 步数 | `SimRun.py` L292-321 |
| 记忆 exploit | `find_closest_node`（阈值 20 m）命中 → 加顶点连边 → LM-Nav `full_pipeline`（alpha=1e-4）图上搜索剩余地标 → 动作图最短路转动作序列 → teleport 终点并 **break** | 作者 pkl 记忆图（本期不使用，改在线建图） | 动作轨迹 | `SimRun.py` L452-492、`external/lm_nav/` |
| 执行 | **运动学 teleport**：`tool.setPoses` 直接设位姿，无物理无控制器（起飞亦为 5×GO_UP teleport） | waypoint 位姿 | 仿真器位姿 | `SimRun.py` L488/L520、L413-416 |

关键既定行为（复现时勿误判为 bug）：

- **全程 teleport**：论文基准数字不含动力学；上游"动作"只是位姿增量约定。AS2 段以
  FollowTimedPath 时间化真实执行（偏离 D5）。
- 动作码 8 个（含 MOVE_LEFT/RIGHT），实际使用 6 个；步长 FORWARD=5 m、UP_DOWN=2 m、
  TURN=15°（`airsim_settings.py` L88-94）。`utils/utils.py` L195-277
  `calculate_movement_steps` 分解顺序固定为"15° 步进偏航 → 5 m 水平步进 → 2 m 垂直
  步进"，分解顺序即执行时间顺序。
- 深度反投影 FOV=90 硬编码（`SimRun.py` L101、L275）；scene 1/7 用 `getImageResponses`、
  其余 `_v2`，两种深度编码不同（replay 处 ×300 缩放 L624）。
- 卡死判据：最近 4 步位移和为 0 → 失败退出（L534）；步数预算 max_step_size；
  landmark 全部达成即结束（L530）——**无显式 Stop 收口**，AS2 段补生命周期收口。
- LLM：create 调用硬编码字面量 `model="gpt-4o"`（`query_llm.py` L112 等）；API key
  占位写死 `SimRun.py` L341；异常 sleep 10 s **无限递归重试**（L346）；缓存
  `llm/cache/{model}/{name}.json`。v1 类含作者个人 Azure endpoint（L74，不迁移）。
- 仿真器双层协议：msgpackrpc 场景服务器（127.0.0.1:30000，`ping`/`reopen_scenes`，
  MAX_SCENE_NUM=8）+ 每场景 airsim 客户端（`AirVLNSimulatorClientTool.py` L96-120）。
  本期以最小桩对接标准 Blocks AirSim（偏离 D2）。

## 2. 职责映射与目录归属（路线 B 裁决后）

| CityNavAgent 部分 | aerohub 层归属 | 目录 |
| --- | --- | --- |
| HSPM 决策 + 语义点云 + 记忆图 | Planner（Navigation Tool 实现基础） | `src/tool/navigation/planner/citynav/ros2_adapter/`（`uav_citynav_planner`，onboard-computer 容器） |
| 云端 gpt-4o → glm-4v-flash 调用 | 外部推理依赖，代理节点承载 | `llm_proxy/uav_citynav_llm_proxy/`（工作站侧） |
| GroundingDINO + SAM 本地推理 | Inference Robotic Function | `src/basic_function/inference/models/grounded_sam/`（uv 隔离，model-migration-cuda） |
| 轨迹执行 | 原生扩展 Behavior（复用，不新建） | `src/behavior/follow_path/` FollowTimedPath（共享 `uav_planner_interfaces`） |
| 起飞/降落 | 原生 AS2 Behavior | TakeoffBehavior / LandBehavior |
| RGB-D/位姿/时钟供给 | 平台与飞控板角色 | flight_board 传感器桥 / 时钟桥 / AS2 self_localization |

不设 `ros1_proxy/`：agent 本体无任何 ROS 进程（requirements 中 ROS1 pip 包属上游
仿真器侧产物，裁剪，偏离 D12）。
landmark 抽取（决策 7 代决）：Blocks 无标注可消费，landmark 序列由 LLM 从指令抽取
（论文方法本身，提示词逐字取 `src/llm/prompt_builder.py`），偏离 D3。
记忆图（决策 6 代决）：不用作者 pkl（坐标绑定 AirVLN 场景），改**在线建图 + 跨任务
复用**（任务 1 落盘、任务 2 命中 exploit），偏离 D4。

## 3. 最小数据/调用链（AS2 段目标形态）

```
mission.py(工作站) --citynav/goal{instruction}--> uav_citynav_planner(算力容器)
uav_citynav_planner --landmark/viewpoint/route 查询--> uav_citynav_llm_proxy(工作站)
uav_citynav_llm_proxy <--文本/JSON-- 云端 glm-4v-flash API
uav_citynav_planner --greedy_mask_predict(image,text)--> inference grounded_sam(PC GPU)
uav_citynav_planner --TimedPath(偏航段→水平段→垂直段)--> FollowTimedPath(AS2)
FollowTimedPath --Motion Reference--> AS2 控制器 --> PX4(flight_board) --> AirSim Blocks
```

图像数据流止步于算力容器/PC GPU；公网访问仅代理出板。

## 4. 复用决策

| 部分 | 决策 | 理由 |
| --- | --- | --- |
| FollowTimedPath + 修订/确认时序 | 直接复用 | 动作分解（15°/5 m/2 m）可完整时间化表达，不丢中间语义 |
| 共享 `uav_planner_interfaces` | 直接复用 | 与 SPF/YOPO/SUPER 同一 FollowTimedPath 契约与收口码表 |
| 上游 `calculate_movement_steps` / 反投影 / 统计滤波 | 语义逐字迁移到 adapter | 输入从 airsim RPC 换 ROS 话题，输出从 setPoses 换 TimedPath |
| 上游 prompt（caption/route/viewpoint/landmark 抽取） | 逐字复制进代理 | 复现保真；复制原因在文件头声明 |
| `external/lm_nav` 图搜索 | 逻辑移植进 `memory.py` | 纯 networkx/scipy，无平台耦合 |
| AirVLNSimulatorClientTool / 场景服务器 | 阶段 1 用最小桩，AS2 段不迁移 | 平台能力由传感器桥替代 |
| openai SDK | 不迁移，纯 stdlib HTTP 等价实现 | 与 SPF D2 同理；容器内也避免 Rust 轮子依赖 |
| torch（agent 进程内） | 不迁移 | grounding 推理归 Inference 服务，planner 不加载模型 |

## 5. 接口契约（阶段 2 实测后锁定，当前为设计值）

- 输入：`<vehicle_ns>/self_localization/pose|twist`；`sensor_measurements/FrontCamera/rgb/image`
  + `camera_info` + 深度（量程/尺度阶段 2 锁定）；`citynav/goal`（String JSON
  `{instruction, max_steps, use_memory}`）；`citynav/cancel`（Trigger）。
- 输出：`follow_timed_path` Action；`citynav/llm/query|result`；`citynav/status|events`。
- 时间：LLM 结果绑定请求时图像 stamp；`max_result_age_s` 配置；`use_sim_time` 经 `/clock`。
- 坐标系：AirSim NED ↔ odom ENU 固化；`cam2ego/ego2world` 组合与 TF 链核对；
  quaternion 分量序重排逐字保留（`SimRun.py` L397）。
- 全景：原地偏航旋转取图（偏离 D8），采集耗时计入周期预算。

## 6. 多机外溢声明

`citynav/goal` 与 LLM/感知接口按 `<vehicle_namespace>` 复制即可逐机独立；但代理缓存
键、记忆图文件路径与 request_id 命名空间须按 vehicle 隔离（本期不实现）。
**本期仅单机验收，不声称多机能力**；机群级语言任务编排属 Mission 层，不在本 Planner 范围。

## 7. 待核查条件（进入阶段 1/2 前必须闭环）

1. 许可证缺口：CODE_LICENSE 文件缺失（徽章断链），对外分发前向上游确认；组内使用
   按 MIT 意图执行。[记录 2026-11-17]
2. ~~权重直链可达性~~ [已闭环 2026-11-17：SAM 直链 200（2564550879 B）、DINO GitHub
   release 直连失败走 gh-proxy.com 成功，均落
   `src/basic_function/inference/models/grounded_sam/weights/`]。
   **新增 D17（后端替换）**：本机 RTX 5060（sm_120）与上游 torch 2.0.1+cu118 不兼容，
   且原版 GroundingDINO CUDA 算子无工具链不可编译、无 CPU 路径——原版实现本机不可
   运行。grounding 后端固定为 HF transformers GroundingDINO（权重同源）+ 官方
   segment_anything，现代 torch cu128 真实 GPU；legacy 段经服务调用（D18），阶段
   4 前置于阶段 1（D19）。与原版数值对照不可行，边界如实声明。
3. ~~glm-4v-flash 多图上限~~ [已实测 2026-11-17：单请求上限 5 图（第 6 图报 1210），
   恰好覆盖 viewpoint 路线查询需求；captioner 逐图调用不受限，
   `log/citynav/llm/multiview-test-20261117.json`]。
4. 场景服务器最小桩协议：`ping`/`reopen_scenes` 返回值形状以
   `AirVLNSimulatorClientTool.py` 消费侧为准逐字段核对。
5. Blocks settings.json `front_0` 相机命名与分辨率（对齐上游 512×512 或记录偏离）。
6. Blocks 指令集 2-3 条起草（建筑/车辆/橙色球语义），用户确认。
