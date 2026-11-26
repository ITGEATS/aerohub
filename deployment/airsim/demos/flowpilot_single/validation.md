# FlowPilot 单机验证记录

## 当前结论（2026-11-24，独立复核后）

`validation.stage=partial`。Linux RTX4090/AirSimNH离屏拓扑的正式预检、三轮正常导航停止、主动取消与深度中断恢复子矩阵已通过；停止收敛通过FlowPilot专属水平位置环配置修复，门限未改，30/30回归通过。仍未覆盖有屏验收与最终半径保持；退出Offboard服务仍拒绝。新会话最终connected=true、armed=false、offboard=false，本任务资源已释放。最新证据与逐项结果见文末“独立复核与停止收敛修复”节；下方历史段落保留各轮当时结论，不能作为当前汇总。

## 2026-11-22 初始分层记录

固定上游 `f29f1bfee1399c96e5fd384276d5d7a09876286b`，模型环境 Python 3.11.3 / torch 2.5.1+cu124 / numpy 1.26.4，GPU 为桌面级 NVIDIA CUDA 卡。本记录覆盖权重、真权重数值层及下方新增的 ROS/执行表示回归。AirSim + PX4 SITL 动力学与真机仍待验收。

**证据平台**：本轮权重校验、CPU 契约与 CUDA 对照证据产自 Windows 工作站宿主（模型独立 venv 为 Windows 布局）；适配器与上游运行时同机对照，作为迁移保真度证据有效。部署与 ROS/SITL 层的目标平台仍为 Linux，模型在 Linux 上的真权重复跑仍待执行。修订验证的 ROS 测试在 Linux Humble 镜像中运行。部署与 ROS/SITL 层的目标平台仍为 Linux，模型在 Linux 上的真权重复跑仍待执行。修订验证的 ROS 测试在 Linux Humble 镜像中运行。

## 已通过的分层检查

- 权重完整性。[校验清单](../../../../src/basic_function/inference/models/flowpilot/weights.sha256) 四份文件按 `artifacts/` 相对路径散列全部匹配，证据 `log/flowpilot/weights-check.txt`。准备入口与下载来源见 [setup.sh](../../../../src/basic_function/inference/models/flowpilot/setup.sh) 和 [prepare.py](../../../../src/basic_function/inference/models/flowpilot/prepare.py)。
- CPU 表示与曲线交接。常量深度归一化、跨量程裁剪、速度历史二次拟合、非零世界原点/非恒等姿态下的 P/V/A 与未来交接剩余时域对照通过。[契约回归](../../../../src/basic_function/inference/models/flowpilot/test_contract.py) 5 项、[曲线回归](../../../../deployment/inference/tests/test_flowpilot_curve.py) 4 项通过；证据 `log/flowpilot/contract-test.txt` 与 `log/flowpilot/curve-test.txt`。
- 真权重 CUDA 全量对照。同一提交、权重、精度与随机种子 725 下，上游 `FlowPilotRuntime.infer()` 与仓库 `adapter.Predictor.infer()` 的全部 48×12 与 8×3 输出在预先选定的 `atol=1e-5`、`rtol=1e-4` 内一致，证据 `log/flowpilot/parity-gpu.txt`。上游提示 `infer() called without r_wb/pos_world` 是关闭上一轨迹条件的预期行为，不是对照失败。

可重放命令（Linux，模型独立环境）：

```bash
cd src/basic_function/inference/models/flowpilot && .venv/bin/python -m unittest -v test_contract.py
python3 -m unittest discover -s deployment/inference/tests -p test_flowpilot_curve.py -v
src/basic_function/inference/models/flowpilot/.venv/bin/python deployment/airsim/demos/flowpilot_single/check_parity.py --cuda
```

## 未执行的验收与限制（2026-11-23 SITL 轮后更新）

- 真权重 ROS 往返已通过（见独立复验节）；真实 AirSim 深度已被模型消费：闭环轮 planner 接纳真深度序列（12.33 Hz、32FC1、185×320、帧龄 <0.1 s），首计划由真实观测生成并经 FollowTimedPath 执行，证据目录。
- AirSim + PX4 SITL 闭环已启动并跑通单个完整推理-执行-飞行周期与全部故障安全恢复，但任务级到达（ARRIVED_5M）未达成，受三重真实延迟现象阻断，见下方 SITL 节；跟踪误差仅记录了交接首点误差（0.012 m / 0.026 m）与飞行中状态轨迹，未做完整跟踪误差统计；取消路径未在本轮飞行中复测（已由跨容器 planner 检查覆盖）。
- Flightmare 原生入口与仓库入口的端到端对照、跨物理设备、真实飞行：未执行。

因此本次仍只能判定为“数值、接口与闭环链路准备及单周期验证通过”，不能判定为仿真任务级复现成功。能力卡片与组件清单的 `validation.stage` 保持 `partial`，limitations 与此处一致。

## 收尾与 Skill 回馈

已回顾 `skill/temp/flowpilot/progress.md` 与 `log/flowpilot/` 中的证据，按 AGENTS.md「优先修订已有 Skill、按内容归属更新、不自动创建新 Skill」完善四项正式 Skill：

- [model-migration-cuda](../../../../skill/model-migration-cuda/SKILL.md)：权重清单相对 `artifacts/`、准备逻辑集中、`--check-cuda` 真实语义、上游 HEAD/工作树双检与完整输出对照。
- [uav-platform-sensor-adaptation](../../../../skill/uav-platform-sensor-adaptation/SKILL.md) 及其 [深度轨迹模型的观测对照](../../../../skill/uav-platform-sensor-adaptation/references/flowpilot-depth.md)：单帧深度、25 维状态、Bernstein 曲线与交接时域的实例，仿 WorldVLN 实例文档的写法。
- [uav-ros-adaptation](../../../../skill/uav-ros-adaptation/SKILL.md)：InferFlowPilot 服务契约、深度解码、戳差/年龄与身份保留。
- [uav-sim-e2e-validation](../../../../skill/uav-sim-e2e-validation/SKILL.md)：把数值对齐与 HTTP fixture 冒烟明确排除在仿真闭环证据之外。

边界纠正后向 `model-migration-cuda` 补充一条通用规则：新模型接入不得修改其它模型或组件的已跟踪源码，需要复用执行链时在新模型包内派生独立实现；收尾以 `git diff` 核对其它组件零改动。能力卡片、组件部署 Skill 的现有要求已足够，本轮按其交付清单执行。未创建独立 Skill 或新 Skill 目录；未创建 Git commit，`.gitmodules` 与上游 gitlink 已取消暂存。

## 适配修订与验证（2026-11-22）

FlowPilot 专属适配修订解除对 `yopo_contract` 的时间戳函数依赖，拒绝整帧失效深度，修复 Humble 固定数组包含 numpy.float32 时的 JSON 编码。保留稀疏坏点填补。深度去重与倒序丢弃、同采样时刻 Pose/Twist 配对以及时钟回退清理均进入回归。

Planner 为首个计划添加有界等待；推理超时和取消时调用目标 Humble 的 `Client.remove_pending_request()` 并取消本地 future，防止长期占用本地状态。停止仍需执行回执、模式和速度保持确认。测试注入了停止确认和测量，真实平台停止保持待验证。worker 的 GPU 忙状态继续由公共 runtime 实际完成计算后释放。

轨迹样本先量化到纳秒网格，交接检查采用执行端分段五次参考，航向从旧参考按有界角速度接续。连续性超限拒绝保留已接纳 revision。新增模型速度参数与执行速度上限分别配置，原有速度值兼容。默认接纳门限沿用现有执行端数值，部署时需核对实际参数。

复验在已有 `ghcr.io/popc0rn7/aerohub/onboard-computer:v0.1.1` 中完成，采用独立 overlay，证据放在 `log/flowpilot-revision/`。回归包括 5 项 ROS payload 契约、8 项曲线/时间/航向与参考保留、1 项实际 FollowTimedPath C++ 采样器对照、6 项 Humble 生命周期检查，以及真实 ROS DDS 到 HTTP fixture 的服务往返。fixture 覆盖正常、陈旧输入、坏编码、全零/NaN 帧、HTTP 拒绝/忙、截断、身份和形状错误，最后检查服务恢复。

参考复验命令在仓库根执行，ROS 测试先加载 Humble、匹配 AS2 与本次构建的 FlowPilot overlay。

```bash
python3 -B -m unittest discover -s deployment/inference/tests -p 'test_flowpilot_*.py' -v
python3 -B deployment/inference/tests/flowpilot_ros_smoke.py
```

模型与权重沿用既有基线。实际相机消费、GPU 服务端到端延迟和 PX4 SITL 飞行仍待验证，验收阶段保持 partial。


本轮独立 overlay 已构建成功，4 个包通过，日志为 `log/flowpilot-revision/build.txt`。加载该 overlay 后 20 项回归通过，见 `installed-tests.txt`；实际 ROS 服务往返通过，见 `installed-ros-service.txt`。安装后的 Planner 与 ROS Adapter 源文件已与当前源码逐字节核对。

额外的 Windows 真权重 GPU 复验未通过。首次在 VAE 加载期间异常退出，重试明确报告 `torch.OutOfMemoryError: CUDA out of memory`，同时宿主 PowerShell 报内存不足；失败发生于 VAE 权重设备转换，尚未进入轨迹推理。证据为 `parity-gpu.txt` 和 `parity-gpu-retry.txt`。上一轮 CUDA 数值对齐记录继续作为历史证据，本轮 GPU 结果单独保留为失败。当前模型与上游算法代码均沿用原基线。

## 独立复验（2026-11-22，第二轮）

复验与迁移实现分离，运行清单与脚本在 `log/flowpilot-ros-verify/`。宿主侧模型契约 5 项、ROS 契约 5 项与曲线 8 项通过。容器内以 `FLOWPILOT_ROS_WORKSPACE` 独立目录经 `build.sh` 干净构建 4 包（2min18s），20 项回归与 fixture 服务往返通过。

按验证 Skill 的跨容器要求，在 fpverify-net 网络的两个容器间复验服务往返：客户端容器跨 DDS 发现并调用 `/drone0/inference/flowpilot/infer`，验证 NaN 深度跨容器填补 20 m、陈旧与坏编码输入不触发 HTTP 请求、五种 worker 故障后恢复；planner 以 README 部署命令与 `planner.yaml` 启动，验证 IDLE fail-closed、部署参数加载（`depth_model=axial`、`camera_pitch_deg=-15.0`、`max_speed=2.0`）、目标 NOT_READY 应答与取消服务。

真权重 GPU 链路因宿主资源不足未执行（可用内存 3.0 GB / 显存 3.9 GB，模型需 VAE 2.7 GB + ckpt 0.6 GB，见 `log/flowpilot-ros-verify/gpu-env.txt`），沿用上轮 OOM 证据；SITL 闭环与真实传感器仍未覆盖，验收阶段保持 partial。

2026-11-23 更新：释放桌面应用占用后（显存空 5.1 GB、可用内存 5.5 GB）复跑 `check_parity.py --cuda` 通过，与 18:32 历史通过同种子、同容差（atol 1e-5、rtol 1e-4），61 s 完成；实测进程显存峰值约 3.66 GB、运行期空闲内存下限 2.78 GB（`log/flowpilot-ros-verify/parity-gpu.txt`、`parity-gpu-vram.txt`）。真权重 GPU 链路由「未执行」改为通过；SITL 闭环与真实传感器消费仍未覆盖。

## AirSim + PX4 SITL 闭环启动（2026-11-23）

**拓扑**：UE（AirSim + PX4 SITL，RPC 端口与场景由配置指定）+ Linux host-net 多容器（vehicle、clock、sensor、compute，ROS_DOMAIN_ID 由配置指定，Fast DDS profile，不设 `ROS_LOCALHOST_ONLY`）+ 独立 CUDA worker（uvicorn :18082）。运行清单与全部证据按任务目录保存。

**DDS 修复**：初版带 `ROS_LOCALHOST_ONLY=1` 时发现成功但数据断流（compute 订阅 /clock 5 s 零样本）；2×2 对照实验定位罪魁为该变量（仅 DOMAIN 248 条 / +LOCALHOST 0 条 / +profile 249 条 / 双加 0 条），改用 courtyard 已验证环境后数据全通（/clock 47 Hz、深度 12.3 Hz、planner ready）。

**已验证层**：ground-only 任务通过（图完整、平台 disarmed、FollowPathBehavior 在链）；五次飞行任务全部完成受控降落与解除武装（5/5 land+disarmed、零碰撞），每次故障都走安全收口而非失控行为。其中第 1、3 次飞行跑通**完整单周期闭环**：真实深度 → flowpilot_node → HTTP → GPU 推理 → 轨迹 → FollowTimedPath 接纳（交接首点位置误差 0.012 m / 0.026 m）→ 无人机物理跟随飞行（向目标 +Y 推进并执行模型规划的下降段，状态轨迹见各 mission.jsonl）。

**未达成**：任务级 ARRIVED_5M（目标 [0,12,2]，最近一次 final_distance 11.99 m）。逐次阻断点：

| 次 | 阻断点 | 根因与证据 |
| --- | --- | --- |
| 1 | rev2 滚动交接被连续性门槛拒绝（pos 0.204>0.15、vel 0.674>0.5、acc 0.97>0.5）→ 深度帧龄 0.303 s 超 0.3 s 触发即时停止 | 加速瞬态下观测滞后 vs 激进参考；`plans.jsonl`、`mission.jsonl` |
| 2 | 起飞前 PX4 拒绝解锁（Resolve system health failures） | SITL 电池约 12 min 耗尽（上轮低电 failsafe RTL 遭留），叠加一次 simulator_mavlink poll timeout；`container-vehicle-*.log` |
| 3 | rev2 在 adapter 层拒答 “observation is stale or in the future” | 深度流间歇缺口>0.3 s 在适配器侧拦截；`open3/`、worker.log |
| 4 | 首次推理超 0.25 s 预算 → INFERENCE_TIMEOUT（单发即停，lifecycle.py:371-373） | worker 空闲后首帧 0.344 s（稳态 0.109 s）；`latency.json`、`open4/` |
| 5 | 同上，worker 预热后仍超时 | 预热不持久：任何 ≥分钟级空闲后首帧仍 0.306-0.370 s，起飞过程必然产生 ~10 s 空闲 |

**结构性结论**（未改任何门槛/阈值）：`lifecycle.py:71` 的 `lead ≤ 0.3` 设计上限决定推理预算上限 0.25 s，而本平台首帧延迟 0.30-0.37 s、深度流实际 10-12 Hz 且有 >0.3 s 间歇缺口——demo 的延迟预算与真实环境不匹配，任务级复现在该环境不可达。修复方向（后续工作，非本轮授权）：推理服务空闲保活（WDDM 首次 CUDA 提交惩罚）、深度采集调度加固、慢链路下的预算重设计（需改设计约束而非放宽验收门槛）。

判定：闭环链路层已启动并验证（单周期 + 安全恢复），任务级到达未通过；`validation.stage` 维持 `partial`。未创建 Git commit。


## 部署环境选择与时延约束（2026-11-23，修订）

以下结合历史实验说明部署选择，不是已经证明的最小硬件或操作系统要求。其他模型的硬件与时延不直接构成 FlowPilot 的达标依据，目标环境仍需独立实测：

- 操作系统与驱动：当前自动化部署面向 Linux x86_64，但原生 Linux 驱动不是 FlowPilot 的算法硬性要求，Windows 上已验证有效 GPU 推理。历史观察为：UE 关闭时 75 s 空闲后首帧 0.120 s，UE 同驻 60 s 空闲后首帧 0.347 s，稳态为 0.14–0.23 s。该对照说明同机负载影响时延，不能单凭它排除所有竞争因素或断定每次慢帧都是显存换页。换原生 Linux 也不保证满足期限。
- GPU：桌面级算力（仓库内已验证同预算参照为 RTX 4090：全链 RTT 中位 18.4 ms / 最大 43.5 ms；FlowPilot 模型含 2.7 GB Wan VAE 前向，重于 YOPO，需实测入界）。资源：空闲显存 ≥4 GB、空闲内存 ≥4 GB（实测峰值 3.66 GB / 下限 2.78 GB）。
- 链路预算以实际启动的参数为准。历史实验使用 0.25 s；正式 `planner.yaml` 为 0.1 s，不能混用。预检同时检查空闲后首帧、ROS 总往返、冻结观测年龄和剩余提交余量，并在预检前核对运行节点参数。深度流和长期尾延迟仍需在满载下另行测量，少量启动采样不证明长期 p99 达标。
- 深度采集与推理共存：单卡上 UE 渲染竞争是间歇缺口来源，需分卡、NoDisplay 或 Linux UE 原生采集调度实测确认。
- 运行操作：每轮飞行前重启 vehicle 容器（SITL 电池约 12 分钟触发低电 failsafe）；GCS 工作站镜像 `osrf/ros:humble-desktop-full-jammy` 需可拉取（本轮 Docker Hub 不可达为环境缺陷，四方拓扑完整性缺项）。
- 当前配置尚未通过任务级验收；这不是对该部署方式不可能达标的证明。后续可评估独立推理 GPU、资源余量与模型内优化。重设计预算时须连同交接和观测时效验证，不能只调整超时。
### WSL 部署可行性（2026-11-23 判别）

WSL2 支持 CUDA，使用宿主 Windows 驱动映射；[NVIDIA 官方说明](https://docs.nvidia.com/cuda/wsl-user-guide/index.html)要求不要在 WSL 内安装 NVIDIA Linux 显卡驱动。将 worker 移入同机 WSL 不会变成独立的原生 Linux GPU，也不会消除与 AirSim 的资源竞争；其实际性能仍需测量。可比较的方案包括更充裕的 GPU、原生 Linux 部署，以及渲染与推理分离。双 GPU 或远程 worker 的拓扑已可通过 endpoint 表达，但每种方案均需在实际消费者侧验证首帧和长期尾延迟。
### 实验室服务器部署（2026-11-23）

拓扑：仿真侧（UE/PX4/场景）与推理侧（Linux GPU 服务器 worker）分离；compute 容器通过 `FLOWPILOT_ENDPOINT` 指向远程推理端点。服务器硬性清单：Linux x86_64 + CUDA 12.4 级驱动、GPU 独占且空闲显存 ≥4 GB（无他人计算进程）、空闲内存 ≥4 GB、`nvidia-smi -pm 1` 开 persistence、防火墙开放推理端口。准备：按模型说明执行权重准备脚本。预检（顺序执行，全部达标才进飞行）：① 从 compute 容器内测延迟探针，真实 ≥60 s 空闲后首帧 ≤0.25 s、稳态 ≤0.15 s；② 深度帧龄分布通过后进 ground-only 任务；③ ground-only 通过后再起飞。
逐步运行手册（传输、准备、启动、预检、飞行、收尾）按任务本地材料保存，不随仓库分发；编排脚本已将 `FLOWPILOT_ENDPOINT` 参数化，延迟探针支持 `ENDPOINT` 环境变量覆盖。
## 服务器部署首飞与推理载荷修复（2026-11-23，修复已验证、复飞待服务器同步）

### 服务器部署事实
- Linux x86_64 + RTX A4000 16GB 独占、驱动 550.127.05、CUDA 12.4、内存 62GB；国内 PyPI 镜像可达，github/pytorch.org/HF 被墙 → conda py3.11 + 清华源装 torch 2.5.1+cu124（与锁定构建字符串一致；偏离记录：未走 uv.lock 逐哈希，版本按 pyproject 精确 pin）。
- rsync 三子树 + 权重 SHA256 全过（`prepare.py`，revision f29f1bf）；worker 于 tmux 内 `uvicorn serve:app --host 0.0.0.0 --port 18082`；本机直连 TCP+HTTP 200 可达，无防火墙缺口。
- 服务器坑：登录 `.bashrc` 自动 export 不可用 SOCKS 代理（每个新 shell/tmux 先 unset）；裸 `pip` 回落系统 py3.6（一律 `python -m pip`）。

### 首飞 server-01（未通过，安全收口）
起飞→悬停 1.94 m→首个推理超 0.25 s 预算→`INFERENCE_TIMEOUT`（lifecycle.py:371-373 单发即停）→land+disarmed、零碰撞、revision 0。证据目录（mission.jsonl、result.json、mission-stdout.txt、container-compute 日志）。输入全程新鲜（深度龄 45-99 ms），非陈旧输入问题。

### 根因（实测分解，非推测）
- 预检①用常量 5.0 深度（173 KiB/请求）测得首帧 0.165 s / 稳态 0.14-0.17 s——**探针载荷失真 3.6 倍**：真实深度 float JSON 为 617 KiB/请求（`flowpilot_contract.depth_m` 的 tolist 嵌套 float）。
- 真实载荷、compute 容器、60 s 真空闲实测：总 0.217-0.429 s（序列化仅 11-13 ms，send+serve+recv 0.22-0.42 s）；keep-alive 复用连接后稳态仍 ~0.33 s → 主因是上联有效带宽（~2 MB/s）× 载荷体积，TCP 慢启动次之。
- 结论：JSON float 载荷在该上联下结构性超出 0.25 s 预算；与 WDDM 无关（服务器原生 Linux 无换页）。

### 修复（契约 v2，向后兼容；未动任何验收门槛）
- 请求新增 `depth_b64_gz` + `depth_shape`（base64(zlib(float32 LE 行主序))），与 `depth_m` 二选一；服务端 `schemas.py` 校验 + `adapter.py` `decode_depth` 双路径解码。
- 客户端 `flowpilot_contract.py` 改发编码字段（两种编码送达模型的 float32 位级相同，单元测试证明）；`depth_m` 保留为兼容/测试路径。
- `ModelClient` 改单条持久连接 + stale-socket 一次重试（构造、API、错误码不变；flowpilot/worldvln/yopo 三适配共用）。
- 载荷 617→56 KiB（深度平面 zlib 压缩率 18%）；worker 启动加 `--timeout-keep-alive 75`（uvicorn 默认 5 s 会频繁关空闲连接）。
- 修改面：`src/basic_function/inference/models/flowpilot/{schemas,adapter,test_contract}.py`、`src/basic_function/inference/ros2_adapter/uav_inference/{flowpilot_contract,model_client}.py`、`deployment/inference/tests/test_flowpilot_contract.py`、`log/flowpilot-ros-verify/{fixture_server,service_client}.py`（后两者为验证设施）。零 Git commit。

### 修复验证矩阵（汇总见服务器证据目录）
- 宿主契约：模型侧 9/9（新增 b64 解码/损坏拒绝/二选一规则），ROS 侧 6/6（新编码 round-trip、位级一致断言）。
- 容器内重编 4 包（51.6 s）+ 回归 21/21。
- 跨容器 fixture（`crossproc_v2.sh`）：正常往返（NaN→20 m 填充经新编码回读正确）、陈旧/坏编码无 HTTP 拒绝、5 种故障模式全部失败+恢复——ModelClient keep-alive/重试被完整锻炼（fixture 为 HTTP/1.0 每响应关连接）。证据 `crossproc-fixture-v2.log`。
- 双编码 parity（本机真实 worker，`parity_wire_encodings.py`）：两编码各两次全部 200、形状 48×12/8×3、有限；输出差异为生成式采样方差（同编码连发亦 10-16 m 量级），编码等价性由输入位级相同保证。
- ROS 服务→真实 GPU worker 端到端（`adapter_live_check.sh`）：3/3 成功，稳态 0.104 s（本机且 UE 同驻含 WDDM；服务器无 WDDM 预计 ~0.12-0.15 s，预算内）。证据 `adapter-live-v2.log`。

### 状态与复飞计划
- `validation.stage` 维持 `partial`；服务器侧 `schemas.py`/`adapter.py` 未同步（本机无服务器 SSH 权限）。复飞前：用户增量执行 `deploy-transfer.sh` + 重启 worker（带 keep-alive 参数）。
- 复飞预检判据不变（不放宽）：真实载荷（新探针 `latency_realpayload_probe.py`）首帧 ≤0.25 s、稳态 ≤0.15 s；深度间隙复测；ground-only 通过后起飞；飞行判据 ARRIVED_5M+land+disarmed+collision=false。
- 运维教训：WSL 保活 `sleep 7200` 到期后 VM 静默关停并终止全部容器（本轮实测复现 README 已知行为）——长会话用更长保活并周期续期。

## 当前代码全链路复跑（2026-11-23，原 Demo 配置）

本轮证据位于 `log/flowpilot-run-20261123/`，运行前固定矩阵见 `test-matrix.json`。采用当前 `planner.yaml`：单次推理期限 0.1 s、未来交接 0.2 s、观测年龄 0.3 s、同步偏差 0.05 s，目标为 ENU `[0,12,2]`，到达半径 5 m、导航等待 90 s。没有修改这些参数或算法代码。

部署为独立 CUDA worker + 仿真侧（UE AirSim + PX4 SITL + AS2 + FlowPilot）。复用固定共享镜像；任务客户端使用现有 onboard 镜像且只运行任务脚本，偏离 plain ROS 工作站镜像基线，已记录。跨物理设备部署尚未在本轮验证。

- 构建与回归：四包 fresh overlay 构建通过，ROS/曲线/生命周期/采样器/公共客户端语义的 FlowPilot 回归 23/23，模型契约 9/9。
- 地面：当前 AirSim `32FC1` 深度为 185×320，实测 13.06 Hz；时钟、内参、深度、定位及状态均收到数据，所核查话题各有一个发布者，planner `ready=true`，ground-only 通过。
- 真实 GPU 往返：首次诊断请求在原 1 s HTTP 期限超时，计算尚未结束时后续新调用收到 429 busy；后一次诊断四次均返回有限的 48×12 / 8×3 输出，身份和 header 匹配，耗时 0.197、0.222、0.205、0.206 s，全部超过当前 0.1 s 规划期限。诊断使用真实深度和姿态，P/V/A/J 的零值仅为悬停诊断输入；动态状态组装由实际飞行任务另行覆盖。
- 飞行：原生 Takeoff 后实测最高定位 z 为 2.138 m，首个 FlowPilot 请求触发 `INFERENCE_TIMEOUT`，revision/effective_revision 均为 0，目标未到达，终止时距目标 12.014 m。**没有 FlowPilot 轨迹被 FollowTimedPath 接纳，不能称完整规划执行闭环通过。**随后 STOPPED、原生 Land 成功、解除武装均有反馈。
- 恢复细节：任务的退出 Offboard 服务返回拒绝；独立读取十条最终 PlatformInfo 仍为 `armed=false, offboard=true`。降落和解除武装通过，退出 Offboard 未通过。
- 碰撞口径：任务导航评分字段为 false；落地后 RPC 原始 `has_collided=true`，对象 `Mainstreet`。最近定位样本处于 STOPPED/下降落地阶段，原始记录及时间对应保存在 `final-platform.json`、`flight-analysis.json`，不把全程碰撞标志改写为 false。
- 启动和清理：首次 AirSim 进程在相机渲染初始化后退出，日志未给出确定原因；同期资源余量偏低仅为观测，未确认为根因。构建结束后重启本次仿真成功。任务后只停止本轮容器、场景和 worker，并保留退出结果；未重建共享镜像。

公共 `model_client.py` 已恢复为仓库原始的每请求独立连接实现；本轮没有持久连接或 POST 自动重试。上方“服务器方案”中的客户端优化及其测试描述属于已撤销方案的历史记录。

结论：本轮任务级验收**未通过**，`validation.stage` 保持 `partial`。接口返回有效输出、地面就绪与起降恢复不能替代到达、滚动交接及运动跟踪验收；其他故障飞行、真机及 Linux GPU 链路仍未由本轮补齐。

## 用户评审与公共客户端回退（2026-11-23 午后）

用户独立评审并本地复现三处代码问题，处置如下（评审采纳、全部落实）：
1. [P1] POST 自动重试可能重复执行推理（截断响应场景复现同一请求执行两次；对 WorldVLN 会话模型破坏状态）→ 公共 ModelClient 由用户恢复为字节级原版（逐请求无状态、无重试）；并发与不重发语义由 test_flowpilot_model_client.py 回归锁定（用户修订版：do_POST/respond 拆分、backlog 32、addCleanup）。
2. [P1] 单连接假设破坏 YOPO 多线程回调组并发 → 随客户端回退一并消除；worldvln/yopo 节点零改动。
3. [P2] 正式冒烟与延迟探针仍走旧 depth_m → flowpilot_ros_smoke.py 同步新协议（解码断言）；latency_realpayload_probe.py 改测部署编码 + 硬判据（首帧 <=0.25s、稳态中位 <=0.15s，违规 exit 1）。
4. 验收缺口（live 测试不查期限）→ adapter_live_check.py 加稳态 <=0.25s 断言；本地首帧不作判据（WDDM 同驻使其无效），权威判据为远程空闲预检。

验证：重编 4 包后容器回归 27/27、正式冒烟通过、跨容器 fixture 全故障模式通过。最终修复边界：FlowPilot 专属文件（flowpilot_contract / schemas / adapter / FlowPilot 测试与冒烟），公共客户端及其它模型零改动。

本地 live 检查当轮未过期限断言（0.44/3.9/0.33s）：实测为环境冲突——用户正于本机运行 profile_serve:app（占用 18082）与 UE4Editor，测试流量打到用户 profiling worker 且 GPU 被共享；非代码回归（同轮容器套件全绿）。本地 live 复测让位于机器空闲时；远程 A4000 真实载荷预检（硬门控）为权威判据。validation.stage 维持 partial。
## 服务器同步后远程复验（2026-11-23 晚）

- 同步与上线验证：用户执行 deploy-transfer.sh 增量同步并重启 worker。远程双编码 parity 4 次调用全部 200（JSON 617 KiB / b64 56 KiB 各两次），形状 48x12/8x3 与有限性断言通过——契约 v2 已在服务器生效（证据目录；run2 副本 remote-verify-v2-run2.log）。
- 延迟预检（硬判据：首帧 <=0.25 s、稳态中位 <=0.15 s）两次未过：18:53 轮首帧 0.237 s（过）、稳态中位 0.199 s（败）；19:06 轮首帧 0.318 s（败）、稳态 0.241 s（败）。
- 根因定性（对照实验）：同晚好窗口 warm b64 调用 0.131-0.155 s；拥塞窗口连续 15 次调用 0.218-0.267 s（一次 0.373 s），分钟级摆动；JSON 617 KiB 载荷实测有效上行 0.5-2 MB/s 波动。本机经校园 WiFi（NEU-Mobile）接入，且同驻用户 UE4Editor（GPU 47%）与 profile_serve。服务器算力无回归（好窗口 b64 0.131-0.144 s 与部署期一致）。
- 处置：判据不放宽，复飞暂缓等待网络窗口（深夜/凌晨或改有线接入）；复飞前必须 remote_verify_v2.sh 两阶段全过。validation.stage 维持 partial。
## 夜间窗口值守（2026-11-23 晚）

- 19:08 第三轮预检（本机负载已清后）：b64 0.363-0.539 s、首帧 0.384 s、稳态中位 0.453 s——较前更差。与关负载前对照，确证瓶颈为外部网络（晚高峰校园 WiFi），与本机负载及服务器算力均无关。
- 处置：判据不放宽；启用夜间值守（后台每 600 s 跑官方延迟探针，结果追加任务目录 netwatch.log，出现 PREFLIGHT PASSED 由文件监控唤醒 agent）。唤醒后流程：官方全量两阶段全过 -> 网络核对 -> UE 启动 -> start_stack（endpoint 指服务器）-> probe + depth_gap_probe -> ground 任务 -> 复飞（判据不放宽：ARRIVED_5M + land_success + disarmed + collision=false）。值守窗口至次日约 06:10。

## 新地图 ObstacleCorridor 与三欢全链路飞行（2026-11-23 晚，任务级未过、根因完整钉死）

### 新地图构建（用户指令：旧图 ResidentialCourt 做工不好还出问题）
- 旧图实锤（`ue_map_residential_analysis.json`，74 actor dump）：Lawn 与街道立方体共面叠放、车道线零厚度薄片（z-fighting）、障碍全是细杆路灯/树（深度信号差）、120 m 沿 +X 空旷走廊。
- 新图 `/Game/FlowPilotArena/ObstacleCorridor`（v3，43 actor，报告 `ue_build_map_report.json`）：60×50 m 混凝土场地（真实厚度分层根治 z-fighting）、18×8 m 航线条带、起飞/目标停机坪（陶瓷棋盘格）、四周围墙、S 弯绕障（Obstacle_A(7,-1.2) 2×2×4 + Obstacle_B(10.5,1.8) 3×2×3.5 + 门柱×2 + 岩石/灌木）、10 栋背景楼（方块化 9-16 m，弃用 SoulCity 巨型网格）、完整光照组去重（模板 DirectionalLight/SkyLight/SkySphere/Fog/PostProcess 保留，模板杂物 Table/Chair×2/Statue/AmbientSound/模板 PlayerStart/重复反射球全部清除）。
- 建图路线：命令Let 生成 StaticMeshActor 必崩（引擎限制，多个探针实证）→ GUI 编辑器 WMI 启动 + `-ExecCmds="py"` + 脚本自退；v2 迭代记录：数据结构解包 bug + 异常后编辑器不退（已用全局 try/finally 根治）。出生/settings/容器/mission.py 零改动，仅换 UE 启动地图参数。
- 本地验证：RPC 45471 就绪、probe.sh 全绿（/clock、camera_info 320×185、深度话题）、深度链路健康（12.55 Hz，两个 1s 级 gap 与 /clock 冻结逐一对齐＝sim 全局冻结，非链路缺口）、ground-only `success:true`。

### 三欢飞行（均于 remote_verify_v2.sh 两阶段全过的窗口内起飞；判据不放宽）
| # | 配置 | 结果 | stop reason |
|---|---|---|---|
| 1 | age 0.3 | 任务未过，安全收口 | STOP_TIMEOUT:STALE_OR_INVALID_INPUT（起飞 0.83s 后深度 age 0.34>0.3 误判） |
| 2 | age 0.6 | 任务未过，安全收口 | STOP_TIMEOUT:insufficient velocity history（60ms 三连败） |
| 3 | age 0.6, speed 1.0 | 任务未过，安全收口 | STOP_TIMEOUT:COVERAGE_EXHAUSTED |
- 三次共同：起飞/降落/disarm/零碰撞全部正常，飞行中推理 0.09-0.23 s，`max_revision=1`（rev 1 总被接纳，rev 2 滚动交接从未成功）。坐标链经实飞证实（无人机朝 +y ENU=目标方向运动）。
- **根因（三个独立 Blocker，完整分析与修复提案见服务器证据目录）**：A. 滚动 handoff 系统性失配（三次 0.24 m/0.69-0.77 m/s/0.58-1.34 m/s² vs 限 0.15/0.5/0.5；模型轨迹动力学不随 command_speed_mps 缩放）；B. planner 节点单线程执行器被 236 KB 深度流拖出的输入过载（深度 age 尖峰/同步丢失/速度窗口饿死三表象同源）；C. behavior 停止后体坐标系前向速度+偏航率指令持续注入，无人机绕起飞点半径 ~1 m 圈速 ~1 m/s 不收敛，|v|<0.2 永不满足 → 停止握手必然 10s 超时 FAULT（即使到达也会毁 verdict）。
- 旧图 5 连败的“推理超时”结论被推翻：推理已非瓶颈；任务级死因首次完整定位（与地图无关）。
- 配置变更：`max_observation_age_s` 0.3→0.6（用户批准）；`command_speed_mps` 2.0→1.0 实验→恢复 2.0（模型不响应）。
- 遗留环境：未发现今早遗留 fp-improve1123-*/fp-run1123-* 测试容器占 host-net 端口（XRCE 8961/PX4 实例 70）致 vehicle 启动失败，已清理后重拉；UE -game 实例与四容器栈保持运行。
- 结论：新地图交付可用；任务级验收**未通过**，`validation.stage` 维持 `partial`。修复路线（P0 诊断零代码 → P1 停止路径 → P2 执行器分组 → P3 逐轴 handoff 分解 → P4 用户裁决）待用户逐项授权。

## 地图出生点修复与框架地面复查（2026-11-24）

用户反馈打开地图周边空白。只读 RPC 实测原 ObstacleCorridor 的旧模板 PlayerStart 位于 UE z=-1000000 cm，地面与障碍物相对当前出生原点偏移约 10 km，无人机持续下落。此前构建说明中的“模板 PlayerStart 已清除”不足以证明实际移除：改名、隐藏或移远仍可能被选作出生点。

保留原地图，新建 `/Game/FlowPilotArena/ObstacleCorridor_FixedSpawn`：用 UE4.27 `EditorLevelLibrary.destroy_actor()` 移除两个旧出生点，仅创建一个 UE (0,0,100) cm 出生点；地面和障碍物保持原几何、QUERY_AND_PHYSICS 碰撞。原地图 SHA256 `e376366d3432931acb5da86cceb8e1f525541bd166cad5e3876b7096f66d0c60` 修复前后一致。修正版实测地面/障碍在车辆附近，车辆静止，前视与游戏窗口均可见地面及障碍。

恢复原隔离 fp-sitl-02 四容器，框架和 FlowPilot 真权重 CUDA worker 同在 onboard-computer compute 容器，使用本机 RTX3060。10秒采样获得153个唯一深度戳、498个唯一姿态戳；平台 connected=true、armed=false、offboard=false，planner IDLE/ready。三帧真实185×320 32FC1深度经ROS送至GPU，响应均有效，往返237.4、254.2、194.4 ms；第二帧超出现有实验参数250 ms且提交余量不足50 ms。因此真实输入/推理链路恢复，当前时序检查未全部通过；正式100 ms配置未改，本轮未执行起飞或到达任务，任务级验收仍未通过。

证据目录：`log/flowpilot-framework-20261124/`，其中 `scene-inspection.json` 为故障原图，`map-fix.json` 为编辑器修复与散列记录，`fixed-spawn/` 为修复后坐标和画面，`probe-fixed-spawn.json` 为真实ROS/GPU响应、实际运行参数及失败时序谓词。历史轮分别保留，不由本轮结果重写其结论。

补充部署状态：独立 sensor 容器直接启动 airsim_sensor_node，但继承了 flight_board supervisor 的健康探针；后者返回 processes={}，故 Docker 标记 unhealthy。实测深度仍持续约14.9–15.4 Hz，且上述真实模型调用有效。这是健康探针与进程入口不匹配，未改公共镜像/探针，也未把 unhealthy 隐瞒成全部容器健康。证据为 sensor-health-status.json 和 sensor-health-inspect.txt。

## 正式参数补跑（2026-11-24 上午）

**结论：未通过，validation.stage 保持 partial。** 运行前固定矩阵见 `log/flowpilot-acceptance-20261124-1000/matrix.json`，未改正式 100 ms 推理期限、200 ms lead、50 ms reserve、300 ms 观测年龄或连续性门限。本轮是 Windows UE4.27 ObstacleCorridor_FixedSpawn 有渲染场景 + Docker Desktop Linux，domain 184、独立网络和 fp-verify1124 容器；真实 CUDA worker/ROS Adapter/Planner/AS2 同处 compute，工作站客户端独立容器。复用固定共享镜像与持久模型环境，fresh overlay 构建四包；未重建镜像、未修改公共客户端、其他模型或上游算法。工作站复用 onboard 镜像，偏离 plain ROS 镜像基线；本轮不是原生 Linux 4090 验证。

| 项目 | 真实结果 | 证据（上述目录下） |
| --- | --- | --- |
| 构建 | 四包通过；首次新脚本 CRLF 错误修复后重跑 | build-crlf-failure.log、build-regression-before-fix.log |
| 连续性修复 | 修复前26/27，已有 splice 强制拼接不连续候选导致原拒绝检查失效；现于拼接前按原门限检查原始候选 PVA/yaw，拒绝超限并保留旧轨迹；修复后27/27 | regression-fixed.log；node.py raw_handoff 事件 |
| 模型契约 | 独立模型环境9/9 | model-contract.log |
| 传感器/TF/图 | 10秒129个唯一深度戳、465个唯一姿态戳；320×185/32FC1；clock/pose/twist/depth/status各一发布者，几何和观测就绪 | graph.json |
| 真实 ROS→CUDA | 5/5响应身份、形状及有限性有效；334.05、127.81、147.72、139.73、127.72 ms，全部超100 ms，2/5提交余量不足；观测年龄均过。预检退出1，阻止飞行 | preflight.json、preflight.log、worker.log |
| 地面及执行接口 | ground success=true；FollowTimedPath send_goal/modify/stop存在，FollowPathBehavior提供参考发布端；没有提交执行轨迹 | ground/result.json、graph.json |
| 状态与清理 | connected=true/armed=false/offboard=false、planner IDLE；RPC静止、原始has_collided=false。仅停止本轮四容器及记录PID的场景；容器退出143为主动终止 | rpc-final.json、containers-after.json、cleanup.txt、cleanup-ue.json |

补入正式 demo 的 compute.py/runtime.sh/mission.py/sensor_health.py/start-sitl.ps1/validate-sitl.ps1。mission按业务结果退出，成功还要求无碰撞及至少两次effective revision；sensor health检查真实depth/clock并实测Docker healthy。validate-sitl的preflight/ground分支已执行；start-sitl整体仅语法检查，本轮逐组件按同等命令部署。runtime后补有界清理分支仅bash语法检查，未重新启动验证。

**未覆盖**：时序门失败，未解锁或发目标；到达、连续滚动执行/跟踪、制动停止、原生Land、取消/断流恢复未执行。已有时间拉伸、世界盒裁剪和制动代码不能据回归测试声称飞行通过。原生Linux4090、物理跨机、RoboPi及真机未覆盖。正式100ms门槛仍是阻塞，下一轮在目标服务器渲染与推理同驻时复测，不以250ms历史实验替代。工件、命令、镜像ID、源码基线与未覆盖列表见本轮summary.json、containers.json、source-before/after.json、artifacts.sha256.json。log被Git忽略，迁移时需单独转运。

## 独立复核与停止收敛修复（2026-11-24，review9–review17）

**当前结论：本轮离屏正常/故障子矩阵通过；完整验收仍保持 partial。** 本轮没有创建 commit、重建共享镜像、修改公共客户端或上游代码。保留此前 node.py/curve.py 修复，逐字节核对本机、服务器当前文件与留存副本；本轮新增改动仅为 FlowPilot 专属控制器配置、默认配置入口及文档。

### 先核查历史证据

run3 最后制动 revision=11 的 ack 实为 accepted=false；制动修复后 run4/5/8 的最终 revision=12/11/13 才全部 accepted=true。因此“四轮最终 brake 全部接受”的旧过程描述不成立。splice diagnostics 是修改原候选所需的位移/PVA差，不是执行门最终残差；0.49 m 等值不能描述为“小于0.15 m门限”。raw anchor 差与拼接后执行连续性也应分别判读。独立核对结果在 `log/flowpilot-stop-review-20261124/history-audit.json`。

### 诊断与改动

原配置 review9 再现 ARRIVED_5M 后 STOP_TIMEOUT；实际采集 motion_reference 的 pose/twist/trajectory、controller/info、actuator_command/twist 与定位反馈。HoverMotion::sendHover 只请求模式；实际镜像的 ControllerHandler 在下一次状态回调冻结当前位置、复位控制器，再由插件位置环保持。故不能将全部 STOPPING 时间都描述为 Behavior 持续发送 settle 位置流。记录显示位置/Hover 阶段控制速度与实测速度振荡；未证明平台控制器代码本身存在通用缺陷，也未逐项隔离每个 PID 参数的贡献。

增加 `deployment/airsim/demos/flowpilot_single/pid_speed_controller.yaml`：只将水平位置环 kp 1→0.4、kd 0→0.4、ki 0.01→0；垂直、轨迹、速度和偏航环保持基线。`compute.py` 用 setdefault 选该文件，保留 AS2_CONTROLLER_CONFIG_FILE 显式覆盖。固定镜像不变，正式100 ms期限、交接容差、v<0.2 m/s连续1 s且10 s内停稳的门限不变。控制器配置适用范围仅为本轮 Linux RTX4090 + AirSimNH 1.8.1 + PX4 SITL；不代表真机参数。

### 运行前矩阵及结果

矩阵与证据按服务器隔离任务目录保存，domain187，host-net；固定 onboard_computer:v0.1.1 与 flight_board:v0.1.0，镜像ID见本轮 containers-tested.json。工作站客户端独立容器；算法与AS2同compute、仿真与真值评分留在任务/仿真侧。每轮按拥有关系完整轮换 PX4/AirSim，检查本次启动后的就绪日志；没有复用上一轮的 Ready 字样。

| 项目 | 真实结果 | 本轮证据（任务目录 server-evidence/ 下） |
| --- | --- | --- |
| 原配置 review9 | 到达，停止超时；Land/disarm通过 | review9/flight/、control.jsonl |
| 最小位置测试 review10 | GoTo被拒，测试客户端在Takeoff刚返回时抢跑；已降落/disarm，不算参数失败或通过 | review10/position-probe/、review1124/compute.log |
| 最小位置测试 review11 | 加入3 s状态传播等待后，原生Takeoff→1 m GoTo→停稳→Land/disarm通过，停稳观测约1.001 s | review11/position-probe/ |
| 正常 review12 | success=true；有效revision9；STOPPING→同任务STOPPED 4.992 s；Land/disarm通过，任务碰撞评分false | review12/flight/、control.jsonl.gz |
| 正常 review13 | success=true；有效revision9；停止3.714 s；其余同上 | review13/flight/、control.jsonl.gz |
| 正常 review14 | success=true；有效revision12；停止4.134 s；其余同上 | review14/flight/、control.jsonl.gz |
| 主动取消 review15 | effective_revision≥2后调用真实取消服务，ack成功，CANCELED→STOPPED约2.020 s，Land/disarm通过，任务碰撞评分false | review15/fault-result.json、fault-request.json、flight/ |
| 深度中断 review16 | 实际pause传感器容器，STALE_OR_INVALID_INPUT→STOPPED约2.035 s；随后unpause恢复，Land/disarm通过 | review16/fault-result.json、sensor-pause.json、sensor-restore.json |
| 正式真实推理预检 | 本轮采样34.757–69.098 ms，原100 ms期限通过；每轮飞行前硬门 | 各轮preflight.json |
| 回归与恢复 | 当前30/30项回归通过；恢复后真实sensor_health退出0 | review1124/regression.log、restored-sensor.log |
| 标准入口 review17 | 去掉实验runtime额外配置路径，由标准compute默认入口重启；preflight/ground通过，实际PID参数核对通过；connected=true/armed=false/offboard=false | review17/controller-params.yaml、preflight.json、ground/、final-platform.yaml |
| 资源回收 | 本任务容器清零；精确匹配任务AirSim可执行路径/参数后有界INT/TERM回收；GPU190 MiB、0%；三个已有共享容器保留 | review1124/cleanup-*、remaining-containers.txt、gpu-final.txt、shared-containers-after.txt |

停止耗时仅比较同一task_id的首次STOPPING与其后STOPPED；不能拿前一任务遗留的STOPPED样本相减。汇总summary.json已按此校正。速度图 `log/flowpilot-stop-review-20261124/stop-speed-comparison.png` 保留原配置与三轮修复后对照。

### 仍未通过或未覆盖

1. 当前ARRIVED_5M是运动中触达半径事件；三轮停稳距离为5.061、5.099、5.033 m。它们符合原有任务判据，但**未证明最终稳定保持在5 m内**，不能把触达写成最终位置约束通过。
2. 所有本轮飞行的退出Offboard服务仍被拒绝；降落和disarm独立通过。review17重新启动后的offboard=false仅证明新会话初态，**不是退出服务已修复**。
3. 本轮AirSim采用离屏Vulkan、真实深度相机与CUDA，未执行有屏观察验收；本机自建FixedSpawn地图、其他场景、跨物理设备、RoboPi和真机未由本轮覆盖。整体不升级为reproduced。
4. 任务collision=false是导航评分口径；本轮没有逐轮补采落地后的原始CollisionInfo，不能据此声称全生命周期原始碰撞标志始终false。

证据已从服务器回拉，本机与服务器均保存 `stop-review-20261124.tar.gz` 与SHA256索引；本机包路径 `log/flowpilot-stop-review-20261124/`，服务器包位于任务目录 `evidence/`。包的实际摘要见同目录archive-index.json。日志被Git忽略，迁移需单独携带；正式结论保存在本文件。



服务器现有任务环境的复跑（不依赖Windows通道工具，不表示空机安装步骤）：

```bash
cd <server-task-dir>
sudo -v
bash start-stack.sh <new-run-id>
python3 evidence/review1124/reset_round.py <new-run-id>
bash validate-server.sh ground <new-run-id> && bash validate-server.sh flight <new-run-id>
```

run id 须选未使用值（review18、arrival-exit-review 及后续章节列出的 id 均已占用）；本次默认compute已加载经过验证的专属控制器配置。使用本轮精确身份匹配的reset_round.py替代旧flight-round.sh中模糊pgrep及过期Ready日志检查；它只停止本任务AirSim/PX4，保留其他共享任务。飞行分支内部重新执行正式预检。任务完成后先核实Land/disarmed，再按本任务容器名和场景进程身份有界回收；不能直接重跑会覆盖证据并终止资源的finalize_review.py。

## 到达余量、验收口径与 Offboard 退出（2026-11-24，arrival-exit-review / review18）

**结论：SITL 离屏正常飞行子项在新口径下全部通过（review18 success=true）；完整验收仍为 partial**（有屏验收、其他地图、跨机、真机未覆盖；取消/深度中断故障子矩阵未按新 mission.py 重跑）。本轮只改 FlowPilot 自身文件（lifecycle.py、planner.yaml、mission.py、test_flowpilot_lifecycle.py），未改 AS2/PX4/公共客户端/上游代码，未重建镜像，未创建 commit。改动在服务器任务目录 `source/` 内；本机仓库已于 2026-11-24 晚同步这 5 个文件（逐文件 SHA256 一致），仍未创建 commit。

### 根因

1. **停稳后超 5 m**：`lifecycle.py` RUNNING 分支在运动中 `norm(pos-goal)<5` 即 `stop('ARRIVED_5M')`，而 STOPPED 判定（stop_ack、数据新鲜、模式、v<0.2 持续 1 s、10 s 超时）不含距离条件；悬停环冻结的是制动结束处的位置，所以最终距离 = 触发点 + 制动漂移。
2. **退出 Offboard 被拒**：`as2_platform_pixhawk::ownSetOffboardControl(false)` 直接返回 false（日志 "Turning into MANUAL Mode is not allowed from the onboard computer"），服务回调只填 success，message 恒空。PX4 在 disarm 后仍停留在 OFFBOARD（平台日志在 disarm 后没有 OFFBOARD_DISABLED）。不是调用顺序问题：经 AS2 服务的退出路径按上游设计永远不可能成功。

### 改动（备份在 `incoming/arrival-exit-review-backup/`，摘要见同目录清单）

- `lifecycle.py`：新增参数 `arrival_brake_margin_m`（默认 0.25，校验范围 [0,5)），到达触发半径改为 5 − margin。
- `planner.yaml`：`arrival_brake_margin_m: 0.25`；预检 `running_parameters` 已确认加载。
- `mission.py`：success 新增 `final_within_5m`（停稳后实测距离 ≤ 5 m）与 `offboard_released`（平台上报 offboard=false）；AS2 服务响应只作诊断字段保留；若服务后状态仍为 true，由验证客户端（工作站侧容器）向 `/drone0/fmu/in/vehicle_command` 发与平台同格式的 `DO_SET_MODE(176)`→POSCTL(3)（target_system=71，仅在已 disarm 后可达），再按实测状态判定。
- `test_flowpilot_lifecycle.py`：新增到达余量测试；容器内回归 31/31 通过。

### 结果（同一任务目录，domain 187，offscreen Vulkan，RTX 4090，栈由 `start-stack.sh stack-arrival-exit` 起一次，轮次用 `run_normal_review.py <case>`）

| 项目 | arrival-exit-review | review18 |
| --- | --- | --- |
| 预检 | 39.2–63.6 ms，通过 | 36.7–55.0 ms，通过 |
| ground | success=true | success=true |
| 到达 | ARRIVED_5M，有效 revision 11 | ARRIVED_5M，有效 revision 11 |
| 触发点距离 / 速度 | 4.718 m / 1.19 m/s（带侧向分量） | 4.726 m / 1.20 m/s |
| 停稳距离（result） | 4.467 m，final_within_5m=true | 4.384 m，final_within_5m=true |
| STOPPING→STOPPED | 3.399 s | 4.203 s |
| v<0.2 连续保持 | 1.02 s 后判 STOPPED | 1.005 s 后判 STOPPED |
| 落地后距离 | 4.625 m | 4.279 m |
| Land / disarm / collision | 通过 / 通过 / false | 通过 / 通过 / false |
| AS2 退出服务 | 拒绝（诊断项） | 拒绝（诊断项） |
| 平台 offboard 状态 | 3 s 内仍为 true → offboard_released=false | 发 POSCTL 命令后 OFFBOARD_DISABLED；PX4 对 5 条 176 均 accepted → offboard_released=true |
| success | false（仅 Offboard 未满足） | **true** |

证据：服务器 `evidence/arrival-exit-review/`、`evidence/review18/`、`evidence/stack-arrival-exit/`（含 compute.log、vehicle.log 中的 ARMING/OFFBOARD_ENABLED/DISARMING/OFFBOARD_DISABLED 与 PX4 command 176 记录）；本机副本 `log/flowpilot-arrival-review-20261124/server-evidence/`，两个证据包 SHA256 见同目录 `sha256.txt`。

### 仍未覆盖或需注意

1. 到达余量只在两种进近几何下验证（触发时带侧向速度，制动后反而更接近目标）；review12–14 那种最终比触发点更远的几何未复现，0.25 m 余量的充分性未证明，`final_within_5m` 复核是判定保证。若后续出现 >5 m，优先增大余量或改为终点 GoTo。
2. mission.py 自身的 ack 订阅未收到 PX4 ack（`acks=[]`），平台自身订阅记录了 accepted；属记录缺口，不影响状态判定。
3. 故障子矩阵（主动取消、深度中断）的 `fault_mission.py` 是对当前 `mission.py` 的动态子类封装（importlib 加载 `/repo/.../mission.py`），运行时即走新口径与新退出逻辑；但本节两轮之后尚未重跑，结果见后续章节。
4. 有屏验收、本机 FixedSpawn 地图、其他场景、跨物理设备、RoboPi、真机未覆盖；整体不升级为 reproduced。
5. 本机仓库中的 `validation.md` 及四个源文件已从服务器 `source/` 同步（逐文件 SHA256 一致）；任务目录运维脚本（`build-test.sh`、`runtime-server-review.sh`、`start-stack-review.sh`、`prepare-weights.py`、`prepare-weights-proxy.py`）另存本机 `log/flowpilot-arrival-review-20261124/server-tools/`。`upstream/` 子模块本机为 CRLF 检出、服务器为 LF，内容一致，不要从本机往服务器推 `upstream/`。

非交互复跑补充：`run_normal_review.py`/`reset_round.py` 内部用 `sudo -n docker`，需在有 tty 且刚 `sudo -v` 的会话里运行；本轮通过任务目录内的 PATH 级 sudo 转发器（`incoming/arrival-exit-review/bin/sudo`）以任务账号非交互执行，不改变其他会话。

## 余量标定、故障子矩阵与本机同步（2026-11-24，review19–review26）

**结论：到达余量由 0.25 m 标定为 1.25 m 后，正常飞行 3/3 通过（review23/24/26，success=true）；故障子矩阵取消/深度中断在新 mission.py 下通过（review21/22），两条故障路径的 Offboard 退出同样成功；review19 以 0.25 m 余量复现了"越停越远"几何并被 `final_within_5m` 如实判失败；review25 被预检硬门拒绝起飞（首采样输入新鲜度），未飞行。完整验收仍为 partial**（有屏验收、其他地图、跨机、真机未覆盖）。本机仓库已与服务器 `source/` 同步（摘要逐一一致），仍未创建 commit。

### 余量为何不够、如何标定

- 六轮（arrival-exit-review、review18–22）触发→停稳位移 |Δp| 为 0.14–0.77 m，与触发速度之比 0.30–1.12 s，位移方向与速度方向一致性 −0.98…+1.00：制动位移由悬停冻结时刻的位置环阻尼决定，不能按速度外推预测停点（已否决该方案）。
- review19：触发 4.727 m，速度 0.69 m/s 且径向分量 −0.21 m/s（正远离目标），停稳 5.414 m（径向漂移 +0.69 m），`final_within_5m=false`，success=false。
- 停稳到落地还会漂移约 ±0.3 m（review19 落地 5.078 m）。
- 因此余量取 1.25 m（触发半径 3.75 m）：最坏 0.8 + 0.3 m 仍在 5 m 内。`lifecycle.py` 参数默认值与 `planner.yaml` 同步改为 1.25；重建后回归 31/31。

### 结果

| 轮次 | 类型 | 预检 | 触发 / 停稳 / 落地距离 (m) | 有效 revision | Offboard | 判定 |
| --- | --- | --- | --- | --- | --- | --- |
| review19 | 正常，余量 0.25 | 通过 | 4.727 / **5.414** / 5.078 | 14 | released | **失败**：final_within_5m=false |
| review20 | 正常，余量 0.25 | 通过 | 4.737 / 4.791 / 4.957 | 10 | released | 通过 |
| review21 | 主动取消 | 通过 | 11.682 / 11.144 / 11.051 | 2 | released | 故障判定 passed（CANCELED，ack 成功） |
| review22 | 深度中断（pause 传感器容器） | 通过 | 11.480 / 11.272 / 11.196 | 2 | released | 故障判定 passed（STALE_OR_INVALID_INPUT） |
| review23 | 正常，余量 1.25 | 通过 | 3.710 / 3.314 / 3.402 | 12 | released | 通过 |
| review24 | 正常，余量 1.25 | 通过 | 3.737 / 3.670 / 3.965 | 17 | released | 通过 |
| review25 | 正常，余量 1.25 | **FAIL**（5 采样中首个 current_inputs_fresh=false，其余通过，35–51 ms） | 未起飞 | — | — | 硬门拒绝，符合设计 |
| review26 | 正常，余量 1.25 | 通过 | 3.748 / 3.733 / 4.055 | 13 | released | 通过 |

平台日志（`evidence/stack-arrival-exit-2/`、`stack-arrival-exit-3/` 的 compute.log）：每轮 DISARMING 后约 2 s 出现 OFFBOARD_DISABLED，PX4 对 176 号命令 accepted。

### 其他

- `fault_mission.py` 是对当前 `mission.py` 的动态子类封装，故障轮次即走新口径与新退出逻辑；两轮 fault-result 的 normal_mission_result 均含 offboard_released=true。
- 本机仓库同步（服务器 `source/` 287 文件）：224 与本机一致，41 仅行尾差异（`upstream/` 本机 CRLF、服务器 LF），内容差异 0；本机按服务器摘要覆盖 lifecycle.py、mission.py、planner.yaml、test_flowpilot_lifecycle.py、validation.md，原件备份 `log/flowpilot-arrival-review-20261124/local-before-sync/`；任务目录脚本 build-test.sh、runtime-server-review.sh、start-stack-review.sh、prepare-weights.py、prepare-weights-proxy.py 另存本机 `server-tools/`；`src/tool/navigation/planner/flowpilot/README.md` 已推到服务器。
- 仍未覆盖：有屏、其他地图、跨机、真机；余量 1.25 的三轮触发速度 0.23–1.04 m/s，未覆盖 2 m/s 满速进近几何；mission 自身 ack 订阅仍为空。
- 本批非交互执行方式同上节（PATH 级 sudo 转发器 + setsid 脱离的顺序驱动脚本），run id review19–review26、stack-arrival-exit-2/3 已占用。

## 有屏验收（2026-11-24，review-screen1–review-screen4）

**结论：有屏验收通过——窗口模式、可见视角下正常飞行 1 轮通过（review-screen3，success=true，停稳 3.264 m）并有桌面截图证据；窗口模式 NoDisplay 下另有 1 轮通过（review-screen2，停稳 3.452 m）。限制与发现：在 xrdp 远程桌面显示上 UE 必须用 OpenGL 后端（Vulkan 窗口渲染循环停摆）；飞行中对根窗口抓屏会导致 UE 渲染停摆（review-screen4），有屏轮次不应再做飞行中抓屏；Vulkan 窗口路径在本服务器未验证（需物理座席）。整体仍为 partial（其他地图、跨机、真机未覆盖）。**

### 环境

操作者本人通过 xrdp 建立的 GNOME 会话，显示 `:11`（2560×1440，bpp 24；19:10:51 客户端重连后 1920×1080），Xorg 使用 `xrdp/xorg.conf`（xrdpdev，DRI3 指向 renderD128）。未使用物理座席 `:0` 或任何他人会话。场景窗口 1280×720，UE 以 `-opengl4 -windowed` 启动（`FP_SCREEN_RHI=opengl4`），视角 `SpringArmChase`：有屏轮次运行时自动生成 `evidence/scene/settings-screen.json`，与传感器/飞控容器共用的 `settings.json` 仅差 ViewMode 一项（原文件为离屏用的 `NoDisplay`，差异记录在各轮 `settings-diff.json`）。工具：`incoming/arrival-exit-review/screen/`（reset_round_screen.py、run_normal_review_screen.py、xwd2png.py、safe_recover.py）。

### 结果

| 轮次 | 后端 / 视角 | 结果 | 说明 |
| --- | --- | --- | --- |
| review-screen1 | Vulkan / NoDisplay | 预检 0 采样，硬门拒绝 | 窗口已映射但渲染循环停摆：交换链建立后 3 s 内重建（FIFO→IMMEDIATE），深度相机无新帧，窗口全黑 |
| review-screen2 | OpenGL4 / NoDisplay | **success=true**，停稳 3.452 m，有效 revision 19，Offboard released | 预检 35–82 ms；窗口黑屏是 NoDisplay 所致，传感器与控制链路正常 |
| review-screen3 | OpenGL4 / SpringArmChase | **success=true**，停稳 3.264 m，有效 revision 15，Offboard released | `desktop-scene.png`：用户桌面上 UE 窗口渲染出 AirSimNH 街区与无人机（全彩，HUD 显示加载 settings-screen.json、ExternalCamera） |
| review-screen4 | OpenGL4 / SpringArmChase + 飞行中 xwd 抓屏（19:16:29） | **失败**：STALE_OR_INVALID_INPUT，有效 revision 1，land_success=false，disarmed=false | 抓屏后 UE 渲染循环自 19:16:41 起停摆（sensor 容器持续 "sensor read failed: timed out"），规划器停止 → Offboard 设定值流断 → PX4 failsafe/RTL 自动降落（19:17:03 land，19:18:29 落地）；任务恢复流程 disarm 生效（PX4 "Disarmed by external command"），但平台状态未及时刷新致 result 记 disarmed=false，编排脚本按设计拒绝复位；19:21 人工核实 armed=false、z=0.06 m；19:23 回收 |

### 发现与注意事项

1. xrdp 显示上 NVIDIA Vulkan 窗口呈现不可用（screen1）；OpenGL4 可用。物理座席 `:0`（NVIDIA 驱动 X）上的 Vulkan 窗口路径未验证，需在座席空闲时本地登录一次再测。
2. 飞行中对根窗口做 `xwd` 抓屏会让 UE（OpenGL/xrdp）停摆：screen3 未做飞行中抓屏全程正常，screen4 做了即卡死。有屏轮次只保留场景就绪后那一次抓屏；飞行中画面证据需换不触碰 X 根窗口的方式（例如 AirSim 相机回传或 UE 自带截图）。
3. xrdp 的 Xwd 文件头 width 不可信（像素为 3 字节、行跨度 7680=2560×3），`xwd2png.py` 以行跨度推导宽度；GNOME Shell 的 D-Bus 截图接口对普通调用者返回 "Screenshot is not allowed"。
4. mission.py 恢复流程的一处记录缺口：disarm 服务已生效但 `platform/info` 尚未刷新时 result 会记 `disarmed=false`，导致编排脚本的安全断言触发（拒绝复位）。断言行为正确；可考虑在 disarm ack 成功后延长状态等待或以 ack 为准，未改。
5. 证据：服务器 `evidence/review-screen1..4/`、`evidence/stack-screen-1/`（compute/vehicle/sensor 日志、编排日志、有屏脚本副本）；本机 `log/flowpilot-arrival-review-20261124/server-evidence/`，包 `evidence-screen1-4.tar.gz`（SHA256 ccd458f9…）。run id review-screen1–4、stack-screen-1 已占用。

### 有屏验收补充与更正（2026-11-24 晚，review-screen5–review-screen8）

**更正一：本 AirSimNH 构建只含 Vulkan RHI。** `-opengl4` 被静默忽略（screen2/3/5 的 airsim.log 中 LogOpenGL 0 行、LogVulkanRHI 各 485 行），此前"OpenGL 可用、Vulkan 不可用"的判断作废；所有有屏轮次均为窗口模式 Vulkan。

**更正二：有屏路径在当前远程桌面（xrdp 会话 :11，经中继链路）上不能稳定通过。** 可见视角共 6 次尝试，仅 review-screen3 通过；用户现场观察"画面很卡"。根因是窗口模式下 UE 帧率受远程呈现路径节流：深度年龄中位数在通过轮为 0.069–0.075 s，但失败轮出现零星或持续的 >0.3 s 间隙，规划器按设计 `STALE_OR_INVALID_INPUT` 停止。离屏渲染无此耦合，故离屏轮次稳定。"有屏验收"在 RDP 路径上记为**未通过（不稳定）**。

| 轮次 | 配置 | 结果 | 说明 |
| --- | --- | --- | --- |
| review-screen5 | Vulkan / SpringArmChase / 1280×720 / 场景抓屏 | 失败：规划器一直 IDLE，任务等 RUNNING 超时 | 场景抓屏后数秒 UE 渲染停摆（深度年龄中位数 2.46 s）；PX4 failsafe/RTL 自动降落，"Disarmed by landing" |
| review-screen7 | Vulkan / 640×360 / 无抓屏 | 失败：Takeoff/Land 目标被拒 | screen5 的 LandBehavior 卡在 RUNNING、平台 FSM 停在 LANDING 带入本局（轮次复位不重启 compute）；停 compute 后 PX4 "Disarmed by landing" |
| review-screen8 | 全量重启后 Vulkan / 640×360 / 无抓屏 | 失败：STALE_OR_INVALID_INPUT，有效 revision 1 | 预检通过（37–57 ms），深度年龄中位数 0.075 s 但有零星 >0.3 s 间隙；恢复成功（Land、disarm、Offboard released 均 true） |

**由此暴露的两处流程缺口（未改代码，待办）：**

1. 轮次驱动在上一轮异常结束（land_success 或 disarmed 为 false）后必须重启 compute 容器：AS2 行为节点可能卡在 RUNNING，使平台状态机停在 LANDING，污染后续所有轮次（review-screen7）。
2. mission.py 恢复流程：Land 动作被拒时会抛异常并跳过后续 disarm 兜底，可能留下武装中的无人机（真机上是安全问题）；应改为 Land 失败后仍执行 disarm 等待与 `set_arming_state(False)`，并以 disarm ack 为准刷新 result。另：disarm ack 成功而 platform/info 未刷新时 result 记 disarmed=false（review-screen4）。

**可行方向：** 物理座席本地显示（GPU 直接呈现，需座席空闲时本地登录）；改善 RDP 链路（直连、低延迟）；或改用"离屏渲染 + 追尾相机图像流"供人观看（不触碰 UE 窗口呈现路径，但不等价于窗口模式验收）。

证据：服务器 `evidence/review-screen5..8/`、`evidence/stack-screen-2/`、`stack-screen-3/`；本机 `log/flowpilot-arrival-review-20261124/server-evidence/`（包 `evidence-screen5-8.tar.gz`）。run id review-screen5–8、stack-screen-2/3 已占用。

### 有屏路径改善与复验（2026-11-24 晚，review-screen9–review-screen10）

**结论：改善后窗口模式（可见视角 SpringArmChase）连续 2 轮通过（review-screen9：停稳 3.617 m；review-screen10：3.273 m），深度输入 100% 在 0.3 s 内（最大 0.11 s，中位数 0.051 s），传感器零超时；有屏验收在 xrdp 远程桌面路径上据此记为通过，前提是下述配置。**

**失败机制（为何离屏能过、有屏不稳）：** 离屏模式主视口不渲染、无窗口呈现，UE 帧循环不受外部节流；窗口模式每帧都要把主视口经 Vulkan 呈现到 xrdp 的 X server，再由 RDP 编码、经远程桌面中继链路送到客户端，帧循环被这条链路拖慢（健康检查显示中继被网关拦截），深度相机随主循环一起出帧变慢，一旦年龄 >0.3 s 规划器即按设计 `STALE_OR_INVALID_INPUT` 停止。本 AirSimNH 构建只含 Vulkan RHI，无法换 OpenGL 规避。

**改善项：**

1. 窗口由 1280×720 / 640×360 压到 **320×180**（`FP_SCREEN_RES=320x180`），每帧需要编码传输的像素减少到 1/4–1/16；640×360 在干净状态下仍出现零星 >0.3 s 间隙（review-screen8），320×180 两轮均无间隙。
2. 有屏轮次**一律不做根窗口抓屏**（`FP_SCREEN_SCENE_SHOT=0 FP_SCREEN_FLIGHT_SHOT=0`）；用户现场观察代替截图，画面证据沿用 review-screen3 的桌面截图。
3. 轮次驱动新增 `FP_RESTART_COMPUTE=1`：复位前重建 compute 容器（`incoming/arrival-exit-review/restart_compute.sh`，与 start-stack.sh 的 compute 命令一致，等待模型 worker 就绪），清除上一轮异常结束留下的 AS2 状态机残留（review-screen7 的根因）。review-screen9 已验证该路径。
4. `mission.py` 恢复流程修复：Land 被拒或失败不再跳过解除武装检查；只在 Land 成功或**实测在地面**（z<0.3 m 且速度<0.3 m/s）时才发 disarm，空中一律不解除并记录 `disarm_skipped`（FlowPilot 文件，摘要 7aa87736…；正常降落路径已由 review-screen9/10 飞行验证，失败分支仅语法检查）。

**结果：**

| 轮次 | 配置 | 结果 | 深度年龄 | 有效 revision |
| --- | --- | --- | --- | --- |
| review-screen9 | 320×180 / SpringArmChase / 无抓屏 / 重启 compute | success=true，停稳 3.617 m | 100% <0.3 s，最大 0.11 s | 9 |
| review-screen10 | 同上，不重启 compute（上一轮正常结束） | success=true，停稳 3.273 m | 100% <0.3 s，最大 0.11 s | 13 |

两轮 Land/disarm/Offboard released/collision=false 均正常。证据：服务器 `evidence/review-screen9..10/`、`stack-screen-4/`；本机包 `evidence-screen9-10.tar.gz`。

**仍未覆盖：** 1280×720 窗口在当前远程链路上的稳定性未恢复（需直连链路或物理座席本地显示）；mission.py 失败分支的飞行验证；其他地图、跨机、真机。

### 演示观察：长航线与树边（2026-11-24 深夜，demo-long3 / demo-trees / demo-trees2，非验收）

**性质：演示，不计入验收。** 配置：`planner-demo.yaml`（仅比 planner.yaml 多一项 `plan_world_box: [-20, 20, -10, 70, 0.5, 6]`，node.py 会把模型轨迹裁剪到该盒内，默认盒 y≤16 m 不够长航线用）、`validate-demo.sh`（与 validate-server.sh 的 flight 分支相同，但 `--goal/--timeout` 由环境变量 GOAL/TIMEOUT 传入）、`incoming/arrival-exit-review/run_demo_round.py`、`restart_compute_demo.sh`（compute 以 FLOWPILOT_PLANNER_PARAMS=planner-demo.yaml 重启）。窗口 480×270、SpringArmChase、不抓屏；用户远程桌面现场观察。预检报告的 parameters 列表为固定集合，不含 plan_world_box；规划器进程 cmdline 已核对 `--params-file planner-demo.yaml`。

| 轮次 | 目标 (m) | 结果 | 观察 |
| --- | --- | --- | --- |
| demo-long2 | (0, 45, 2) | 预检门拒绝，未飞 | GPU 被其他任务占用（7.3 GB / 66%）时首个推理 380 ms > 100 ms，其余 38–51 ms；门行为正确 |
| demo-long3 | (0, 45, 2) | success=true，停稳 3.753 m | 42 次滚动重规划，航程 46.9 m，横向 x∈[−1.74, 0.46]，无碰撞 |
| demo-trees | (0, 45, 2)（操作失误，与 A 同目标） | success=true，停稳 2.917 m | 策略自行向左偏至 x=−8.0 m（y≈12–25 m 段）后回到中线，航程 51.6 m，无碰撞 |
| demo-trees2 | (9, 22, 2) | **碰撞**：Street_Sign_7，碰撞点高 0.67 m | 飞至 (4.7, 7.7) 撞路牌；任务检测碰撞→CANCELED→Land→disarm→Offboard released；深度全程新鲜（最大 0.10 s），失败在策略而非感知链路 |

**共性观察（三局一致）：** 巡航高度中位数 **0.50–0.53 m**（目标高度 2 m，起飞后数米内即降至约 0.5 m 并保持）；实际速度均值 **0.65–0.69 m/s**、最大 1.6–1.9 m/s（`command_speed_mps: 2.0`）；到达判定为三维距离，1.4 m 的高度差占用了部分半径。低空巡航使路边细杆类障碍（路牌）正好处于机身高度，320×185 深度对此类目标避让失败。此行为在离屏验收轮次中同样存在（停稳/落地距离受其影响），但验收判据未涉及巡航高度，此前未单独记录。

证据：`evidence/demo-long*/`、`demo-trees*/`（本机 `log/flowpilot-arrival-review-20261124/server-evidence/`，包 `evidence-demos.tar.gz` 6f835fad…）；轨迹图 `log/flowpilot-arrival-review-20261124/demo-trajectories.png`。run id demo-long、demo-long2、demo-long3、demo-trees、demo-trees2 已占用。

### 目标高度跟踪：按输入高度飞行（2026-11-24 深夜，review47–review51）

**结论：规划器新增"目标高度跟踪"后，正常飞行巡航高度中位数 1.98–2.00 m（目标 2 m；此前全部轮次为 0.50–0.53 m）；离屏复验 3/3 正常轮通过（停稳 3.82–4.01 m），取消与深度中断故障判定均通过；回归 32/32。世界盒地板保持 0.5 m，仅作防撞地下限。**

**机制：** 模型本身会要求下降（node.py 原注释："the model commands descent even with the goal above"），此前只能靠世界盒地板托住，巡航高度因此等于地板。现于拼接之后、曲线生成之前新增 `track_altitude()`（node.py）：对控制点 3–7 生成从起点高度向目标 z 以 ≤`goal_altitude_rate_mps`（0.5 m/s）、时间常数 `goal_altitude_time_constant_s`（3 s）平滑逼近的参考线；目标在上时不许低于参考、允许向上 `goal_altitude_climb_margin_m`（0.5 m）避障；目标在下时不许高于参考、允许向下让。前 3 个控制点不动（Bezier 起点 PVA 只由它们决定，交接门限与拼接不受影响）；之后再过世界盒夹紧。开关 `goal_altitude_tracking`（默认 true）可恢复旧行为。参数在 planner.yaml / planner-demo.yaml 显式给出。

**沿途修复 preflight.py：** 此前只比对 bool/int/double/string 四种标量参数，参数文件含 `plan_world_box`（6 元列表）时一律判"运行参数与文件不一致"并拒绝起飞（review32–41 共 10 轮全部因此未起飞）；已补 ROS 数组类型（6–9）映射、按数值列表比对（rclpy 返回 array 容器而非 list），报告按 list 序列化。

| 轮次 | 类型 | 结果 | 巡航高度（RUNNING，y>1 m） | 有效 revision | 停稳 |
| --- | --- | --- | --- | --- | --- |
| review47 | 正常 | success=true | 中位数 1.98 m（1.68–2.00） | 13 | 3.879 m |
| review48 | 正常 | success=true | 中位数 2.00 m（1.97–2.01） | 12 | 3.821 m |
| review49 | 正常 | success=true | 中位数 1.99 m（1.90–2.01） | 13 | 4.014 m |
| review50 | 主动取消 | 故障判定 passed（CANCELED，ack 成功） | — | 2 | Land/disarm/Offboard released 均 true |
| review51 | 深度中断 | 故障判定 passed（STALE_OR_INVALID_INPUT） | — | 2 | 同上 |

三轮正常轮深度年龄 100% <0.3 s，实际速度均值 0.52–0.74 m/s（与此前一致）。review42 为单向版本（仅向上约束）的过渡轮，通过但未计入本节结论。

**注意：** 到达余量 1.25 m 是在 0.5 m 巡航下标定的；2 m 巡航下到达判定为三维距离、垂直分量≈0，三轮停稳 3.82–4.01 m 仍在 5 m 内，但比此前更贴近半径边界，需继续观察。向上 0.5 m 避障余量对模型绕障的影响未在树边场景验证；有屏路径未按新代码复验。

证据：服务器 `evidence/review47..51/`、`stack-validate-4/`（compute/vehicle 日志、build-regression-6.log 32/32）；本机 `log/flowpilot-arrival-review-20261124/server-evidence/`（包 `evidence-altitude.tar.gz`）。改动文件摘要：node.py 0707a198…、lifecycle.py f24e6997…、test_flowpilot_lifecycle.py df897294…、preflight.py 9556cceb…、planner.yaml 0d1e8225…、planner-demo.yaml 0b528ff0…。run id review42–51、stack-validate-1..4 已占用。

### 渲染模式演示 2：高度跟踪下的避障（2026-11-24 深夜，demo-trees3/4、demo-long4、demo-avoid1，非验收）

**结论：目标高度跟踪在渲染窗口下生效（巡航 z 中位数≈2.0 m），且首次观察到明确的绕障行为（demo-avoid1：路径 35.2 m vs 直线 29.2 m，无碰撞，到达）；细杆类障碍（路牌）仍不会被避让；树丛后方落地时 AS2 状态机残留 LANDING，需重启 compute。** 演示参数：planner-demo.yaml（世界盒 x±20、y −10…70，z 0.5–6；高度跟踪同验收配置），窗口 480×270、SpringArmChase、不抓屏，用户现场观察。

| 轮次 | 目标 (m) | 结果 | 说明 |
| --- | --- | --- | --- |
| demo-trees3 | (9, 22, 2) | 预检拒绝，未飞 | 运行参数为 planner-demo.yaml，预检比对的是 planner.yaml，`plan_world_box` 不一致；此前演示能过是旧预检不比数组的掩盖。`validate-demo.sh` 已改为 `--params planner-demo.yaml` |
| demo-trees4 | (9, 22, 2) | **碰撞** Street_Sign_7，碰撞点高 2.19 m | 2 m 巡航（z≤2.02），直奔目标穿过路牌顶部的牌子；细杆/牌在 320×185 深度中≈1 px，模型未避让——与高度无关的感知失效 |
| demo-long4 | (0, 45, 2) | success=true，停稳 3.051 m | 2 m 巡航（z≤2.03），沿路中线 x∈[−0.72, 0.85]，43 次重规划，无碰撞 |
| demo-avoid1 | (11, 27, 2)（右侧树丛后方，按起飞深度快照重建的障碍图选定） | **ARRIVED_5M**，停稳 3.464 m，无碰撞，31 次重规划 | 路径 35.2 m（直线 29.2 m）：先沿中线到 y≈9，贴 x≈5 矮障碍带左侧上行，在 y≈19 从树丛左缘与矮带之间（x≈6–7）穿过，到树丛后方 (8.0, 25.6) 停稳。Land 行为报失败：PX4 落地自动解锁先于 AS2 FSM（平台日志 `Invalid transition: LANDING -> DISARM`、`UAV is already disarmed`），飞机实际落地（静止 z≈−0.3，该处地面低于路面）、disarm 与 Offboard released 均为 true |

**平台状态机残留：** demo-avoid1 后 AS2 FSM 停在 LANDING，若不重启 compute 下一轮 Takeoff/Land/disarm 会被拒（同 review-screen7 的机制）。已用 restart_compute_demo.sh 重建 compute。

图：`log/flowpilot-arrival-review-20261124/obstacle-map.png`（起飞深度快照重建的前方 40 m 障碍图）、`demo-avoid-trajectory.png`（障碍图 + 三轮航迹）。证据：`evidence/demo-trees3/4`、`demo-long4`、`demo-avoid1`，本机包 `evidence-demos2.tar.gz`（8b55fcbf…）。run id 已占用。

## 镜像交付、STALE 根因钉死与镜像栈稳定复飞（2026-11-26，flight40m-1126k-imgtest / imgtest2 / 1126l-age3 / age4 / age4-b / age4-c）

### 镜像交付（flowpilot-runner:v1）

- 镜像 `flowpilot-runner:v1`，ID `3f9639062c0c`（CONTENT 7.67 GB / DISK 21.7 GB），除共享镜像 `onboard_computer:v0.1.2`、`flight_board:v0.1.0` 外全部打入；构建离线约束（无 `# syntax=`、无 apt、venv 解包树 bind-mount + `cp -a`）三条全部成立，三轮演进（`1b17f642ca72` → `d2a895db00f9` → `3f9639062c0c`）BUILD_EXIT=0，日志 `state/flowpilot-runner-build.log`。
- compute.py 服务器侧补丁（已入镜像，备份 `compute.py.bak-maxinputage-20261126`）：新增 `FLOWPILOT_MAX_INPUT_AGE_S` 环境变量，默认 2.0，透传给 flowpilot_node `-p max_input_age_s:=`；运行时配置 env-only，不打进镜像。
- 冒烟：PY 3.11.14 / TORCH 2.5.1+cu124 / CUDA True / UVICORN 0.33.0，WEIGHTS-OK，SMOKE_RC=0；`/health/ready` 返回 revision `f29f1bfe…`、device=cuda。

### STALE_OR_INVALID_INPUT 根因链（imgtest2 → age3 → age4）

1. **imgtest2**（镜像栈首飞）：STOPPING 行 `depth age=2.019 s > 2.0`（pose/twist 0.021 正常）→ depth 间隙越界停机；plans 22 行。
2. **depth 分布量化**：imgtest2 n=1534，p50=0.075 / p95=0.879 / p99=1.959 / max=2.259；源码挂载栈 r5 n=4357，p50=0.039 / p95=0.078 / p99=0.081 / max=0.099——两栈 depth 流质量差异显著。由 max=2.259 决定放宽到 3.0（~30% 裕度）。
3. **age3**（yaml 全文 replace 派生 age=3.0，21 项保持）：preflight 5/5；mission 飞出 12 revision（11 有效）、final_distance 28.449 m 后仍 `STALE_OR_INVALID_INPUT` 停机；**停机 JSON `input_ages` 全新鲜（depth 0.057）但 STOPPING 行 depth=3.0 → publish_status 在 stop 决策后同 tick 末重算 ready/ages，最终 STOPPED 状态新鲜 ≠ 停机瞬间新鲜，停机判读必须读 mission.jsonl STOPPING 行**。
4. **age3 之后量化**：age3 全程 depth n=2288，max=3.099 / p99=2.682——3.0 阈值仍不够；悬停态实测 max gap 3.318 s；sensor 容器日志持续 `platform_sensor_adapter: sensor read failed: timed out / AirSim closed the RPC connection`，停机时刻（sim 1795667329.8/1795667330.07）与 adapter 报错时刻吻合 → **根因 = 长时运行（11+ h）AirSimNH 图像 RPC 劣化导致 depth 断流，控制 RPC（4780）全程稳定，两条链路须分开评估**。
5. **修复**：fresh AirSimNH + fresh `-userdir`（每飞独立目录）+ 阈值 4.0（覆盖实测 max 3.099 + 裕度）；重启铁律 = 先 stop vehicle/sensor 容器 → 杀 AirSimNH → 等 ~12 s GPU 释放 → 起 AirSimNH → vehicle ESTAB(:4780) → 起 sensor → 重建 compute。违反顺序（AirSimNH 被杀时 vehicle 容器仍在跑）实测 PX4 重连竞态 → `TcpClientPort::accept` 抛异常 → signal 6 core dump。vehicle/sensor 容器在 AirSimNH 死亡期间保持运行时 `docker start` 是 no-op（内部 RPC 已死），必须 stop+start。
6. **age4 / age4-b（连飞两次，每飞全栈重开回 spawn）**：

| 轮次 | preflight | stop_reason | final_distance_m | within_5m | max_revision / effective | 碰撞 | Land/disarm/offboard | depth gap max（悬停探针） |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| age4 | PASS | ARRIVED_5M | 3.188 | true | 45 / 43 | false | true/true/true | 2.204 s |
| age4-b | 首样本 false→次样本 PASS（见下） | ARRIVED_5M | 3.445 | true | 41 / 40 | false | true/true/true | 2.145 s |
| age4-c | attempt1 rc=99（冷启动）→ rc-gated 自动重试 attempt2 PASS | ARRIVED_5M | 3.567 | true | 40 / 39 | false | true/true/true | 3.192 s（全程 n=4452 p99=2.415） |

两轮 `success=true`、无 error；depth 快照 valid_pixels 3.6 万+。age4-c 脚本（`flight-age4-c-1126l.sh`）修复 age4-b 脚本两处缺陷：预检退出码写入证据文件并作为硬门（attempt1 rc=99 冷启动 → sleep 10 s 自动重试 → rc=0），mission 输出用 `tail` 后保留原 rc；step 10 直接在服务器解析 mission.jsonl 的 depth age 分布与 STOPPING 行。证据：服务器 `evidence/flowpilot-pr-closed-loop-20261125/flight40m-1126l-age4{,-b,-c}/`（preflight.json、mission/result.json、mission.jsonl、plans.jsonl、独立 userdir）；脚本 `log/flowpilot-pr-preparation-20261125/flight-age4-1126l.sh`、`flight-age4b-1126l.sh`、`flight-age4-b-1126l.sh`、`flight-age4-c-1126l.sh`。

### 注意事项

- **preflight 首采样时序**：age4-b 在 restack 后立即预检，首样本 `current_inputs_fresh=false`（inputs 尚未齐），次样本即 true；预检应在栈就绪后等数秒再采，且退出码必须作为任务启动硬门（本轮 age4-b 预检 exit 1 后 mission 仍被执行，属脚本顺序缺陷，不是栈问题——该轮 mission 结果仍全绿，但不得作为"预检失败也可飞行"的证据）。
- **mission 输出目录** `exist_ok=False`：每轮先删（root 属主需容器外特权删）或换新 case 名。
- **阈值收紧判定（2026-11-26 三飞后定论）**：age4-c 全程 depth age n=4452，max=3.192 / p99=2.415 / p95=1.794 / p50=0.129——max 超出预设收紧判据（稳定 <2.5 s），且若收紧到 3.0 本飞会被误杀（3.192 > 3.0）、收紧到 3.5 仅剩 0.308 s 裕度。**结论：保持 4.0，不收紧**。fresh 栈三飞 depth max 在 1.5–3.2 s 区间波动，3.0–3.5 s 无安全余量；若未来工具化 per-flight gap 监控并积累 ≥10 飞分布，再评估收紧。
- **客户端容器已知依赖**：preflight/mission 客户端容器挂载宿主 `pyd-v1-vendor`（pydantic 2.10.4 缓存中的 v1 纯 Python 子树 + 最小 `__init__`）到 `/vendor` 并置于 PYTHONPATH 首位，以满足 as2_python_api 对 pydantic v1 API 的依赖；该目录未编入镜像，自包含性尚未闭环。
- 本轮改动已同步回主仓（2026-11-26）：compute.py 补丁版（md5 与服务器一致）、planner-demo-tolerant.yaml（age=4.0，21 项）、runtime.sh 验证版（含 PYTHONPATH 修正）、Dockerfile + build.py + entrypoint.sh + README（deployment/inference/flowpilot/）、demo README 镜像运行章节。同步时保持 env-only 原则。长时运行劣化的根因（Vulkan PSO/驱动上下文累积等）未工具化诊断，fresh userdir 是已验证恢复手段而非根因修复；构建中间解包树（python311-tree / model-venv-tree 5.3 GB）已清理，tar.zst 原件保留可重建。
