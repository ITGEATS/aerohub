# FlowPilot 新 dev 适配与验证边界

本次以 `c5bc3dbbd265e67131f158842b9987055568f68f` 为开发基线。上游固定为 `f29f1bfee1399c96e5fd384276d5d7a09876286b`，模型与权重语义不因目录调整而改写。**当前状态：新交付版本已在 Linux RTX4090 + AirSimNH + PX4 SITL 全栈完成三轮端到端仿真飞行验收（2026-11-26，见下方"全栈端到端三飞验收"）；真机与跨设备仍未验证。**

## 已调整的实现

- 源码进入 `src/basic_function/inference/` 和 `src/tool/navigation/planner/flowpilot/`；保留共享主线实现，登记上游 gitlink。
- 通过公共 NavigateToPose / AS2 NavigationModule 调用原有规划与停止链；首期仅支持 COMPLETE 点目标，明确拒绝目标流、HOLD、终端朝向和部署速度以外的请求。
- 公共成功要求原 STOPPED、正确原因、新鲜最终位姿及原始精度距离 ≤5m；工作站另外核对 Land、disarm、Offboard 和实际 revision。
- 增加自包含组件 Dockerfile、构建计划检查、原生 Linux 角色启停/预检入口、Card v1 与导航 Tool 清单。权重按清单校验后通过 BuildKit 命名上下文编入镜像，运行只挂独立证据和场景配置。
- 观察窗口使用场景原生 `SpringArmChase` 第三人称追随视角；动态俯视视图及真实窗口鼠标行为仍待实测。

## 本轮本地检查（2026-11-25）

| 检查 | 当前结果 | 证据边界 |
| --- | --- | --- |
| FlowPilot payload 契约 | 6 项通过 | CPU 编码、时间与字段检查，非真实相机 |
| 曲线/制动回归 | 9 项通过 | CPU 曲线和参考生命周期，非机体动力学 |
| 公共接口及结果边界 | 9 项通过 | 从实际类 AST 执行状态机，ROS 绑定用测试边界替身；不证明真实 DDS/Action |
| 交付计划、权重及故障判据 | 6 项通过 | dry-run、配置拒绝、数据挂载及校验和反例，不证明 Docker build |
| 公共 HTTP 客户端 | 2 项通过 | 实际本地 HTTP 并发与不重放测试；非真实 GPU |
| 实际 C++ 执行采样器 | 1 项通过 | 默认权限无法启动编译器，沙箱外实际编译运行后通过；非飞行 |
| 启动脚本及计划 | 4 份 Bash 语法检查通过，构建/取消/复位 dry-run 通过 | 不执行脚本、不启动容器；镜像/uv/权重占位值仍需现场替换 |
| Card 官方验证器 | 全仓检查失败；本次两张 Card 未增加错误 | 与 HEAD 对照确认原注册表重复 ID 和 CityNav 字段/生命周期错误已存在，未越界修改其他能力 |
| Docker/真实 Humble | 后续兼容环境诊断通过，正式交付镜像未执行 | 见下方本机实跑；诊断基于已有 v0.1.1，不能冒充目标交付镜像 |
| CUDA 完整数值对照 | 后续 Windows 宿主对照通过，交付镜像未执行 | 见下方本机实跑；复用既有模型环境，不等于新环境重建 |
| 有屏 AirSim/PX4 正常及故障 | 未执行 | 未连接或改变用户现有服务器/仿真 |

可复跑的正式测试位于 `deployment/inference/tests/test_flowpilot_*.py`。Humble 生命周期、preflight 和 ROS smoke 必须在编译生成接口并加载相同安装环境后执行；缺 ROS 不能将跳过项当通过。

## 本机实跑（2026-11-25）

用户随后授权在本机尝试运行。以下为新路径代码的实际诊断，不能继承为完整交付、真实传感器或飞行通过。证据按任务保存在 `dev/flowpilot/local-run-20261125/`，含预声明计划、源文件散列、环境与镜像身份、原始输出和资源采样。

| 实际执行 | 结果 | 范围与证据文件 |
| --- | --- | --- |
| 真权重 CUDA 完整数值对照 | 通过，约25秒完成整个进程 | 桌面级 GPU 6GB；复用既有 Python3.11.3/torch2.5.1+cu124 环境，执行新 adapter 和锁定上游；固定合成输入、全部输出按 atol=1e-5/rtol=1e-4 比较；`cuda-result.json`、`cuda-parity.log` |
| 真实 GPU HTTP 服务 | 功能通过，期限未全部通过 | 新 runtime/adapter，6次有效请求均返回正确身份、完整有限输出；非法速度422后恢复。空闲15秒后首请求469.6ms，后续5次为120.6/103.9/92.7/83.0/105.3ms；4/6超过100ms。`http-result.json` 里的 passed 仅为功能检查，期限字段另为false；不是 ROS preflight |
| 当前 ROS 包重新构建 | 4包通过 | 仅使用本地既有 onboard v0.1.1 固定 image ID 构建 `flowpilot-ros-diagnostic:local1125`，代码在构建阶段复制，运行 mounts为空；没有模型环境/权重，不能称完整组件镜像；`ros-build.log`、`ros-source-manifest.json`、`ros-image.json` |
| 真实 Humble 回归 | 20项通过 | 契约6、生命周期10、preflight判据4；生命周期包含测试替身，不证明物理停止。另有真实 DDS→ROS Adapter→HTTP fixture 往返通过；`ros-tests.log` |
| 公共 Navigation Action | 拒绝/未就绪分支通过 | 实际节点与生成类型，真实 Action 返回 INVALID_GOAL、UNSUPPORTED、NOT_READY，私有 cancel 拒绝；未注入运动状态、未启动执行服务器，成功/运动中取消/超时仍待平台测试；`public-action.log` |
| Windows 正式启动入口 | 拒绝，退出2 | 原生 Linux 入口在 Windows 按设计拒绝；未为了本次试跑去掉系统边界；`windows-entrypoint.json` |
| WSL AirSimNH Vulkan 前置条件 | 未满足 | Ubuntu22.04 WSL2 的 `vulkaninfo --summary` 只有 llvmpipe CPU 设备，未列出 NVIDIA 渲染设备；容器 nvidia-smi 可用不能替代图形验证；`wsl-vulkan.json` |
| 真实传感器、FollowTimedPath运动和飞行 | 未执行 | 本轮未启动AirSim/PX4，没有正常/取消/故障恢复飞行结果，未执行正式同驻preflight；不能把 HTTP 诊断当成其替代 |

诊断基础镜像缺少 msgpack，完整交付 Dockerfile 已声明该依赖，尚未实际安装/构建验证。首次 ROS 诊断构建误将本地 image ID 当作 FROM 引用而失败；保留失败日志，再核对本地标签对应同一 ID 后重建通过，没有替换基础镜像。诊断镜像仅供接口复验，不作为 PC 交付拓扑。

宿主 GPU 数值与 HTTP 功能已实际运行，更新了前述“未执行”状态；完整 Linux 模型环境重建、自包含 GPU 镜像、真实 GPU ROS 消费及下列平台矩阵仍未完成。模型进程及本轮容器的清理单列取证，不改写为正常运行自行退出。

## 必须重新运行的矩阵

2026-11-25后续目标服务器资源核查：本地确实存在onboard_computer:v0.1.2（摘要ad41f6143b06…）和flight_board:v0.1.0（摘要1eff3190ca78…），共享AirSimNH主程序与其manifest匹配。已用固定摘要的onboard镜像无挂载、无网络调用Humble/AS2导入及容器GPU查询，退出0并回收。完整身份、检查命令及边界保留在dev/flowpilot/server-shared-20261125。这解决了此前“目标机是否已有基础镜像和场景”的不确定性；远端镜像可拉取性、新组件构建、模型GPU及飞行矩阵仍待执行，未改动原任务或共享安装。

运行前固定源码/镜像/权重/配置身份，按 [Demo](../deployment/airsim/demos/flowpilot_single/README.md) 准备。正常次数和故障阈值在执行前确定，不能将历史轮次自动补入新矩阵。

1. 核实团队当前 onboard v0.1.2、flight_board v0.1.0 及可获取摘要；确认 Linux uv 镜像与 BuildKit 命名上下文支持，校验并编入已有权重，构建组件镜像，记录源码 dirty diff、上游、锁、image ID/digest。
2. 无宿主源码/venv/overlay 挂载启动镜像，验证模型权重、CUDA、全部样本和控制点数值，以及真实 ROS 客户端→Adapter→worker 消费。
3. 验证公共 Action 的成功、取消、超时、忙、错误输入、不支持约束和停止未确认；再运行实际传感器同驻预检，100ms期限仍保持。
4. 有屏 AirSimNH + PX4 SITL 正常导航：连续 revision 生效、目标高度、停稳后距离、速度保持、Land/disarm/Offboard 独立判断，记录完整碰撞窗口。
5. 取消、真实深度中断/恢复、推理故障等适用路径；保持原停止门限，恢复与清理分别取证。
6. 实测动态质心俯视及点击后鼠标可出窗，确认不同负载下深度新鲜度；结束只清理本次资源。

## 全栈端到端三飞验收（2026-11-26）

在 Linux RTX4090 + AirSimNH + PX4 SITL 全栈完成三轮端到端仿真飞行验收。组件镜像按离线约束构建（无 `# syntax=`、无 apt 网络访问、venv 解包树 bind-mount + `cp -a`），冒烟确认 PY 3.11.14 / torch 2.5.1+cu124 / CUDA 可用，`/health/ready` 返回上游 revision `f29f1bfe…` 与 `device=cuda`。compute 容器以默认角色自启 worker + 推理节点 + planner，飞行由 `mission.py --goal` 唯一触发。验收镜像内的运行时源码与本 PR 对应文件逐字节一致（preflight/mission/规划器/模型适配/测试均经哈希核对）。两点边界须如实说明：① 验收镜像由任务内离线配方构建（不做 uv sync、不联网 apt，直接搬入已验证的 CPython 3.11.14 与模型 venv），与本仓库 Dockerfile 的 uv 锁文件路径不同，按仓库文件的正式重建尚未执行；② 验收时 preflight/mission 客户端容器额外挂载了宿主提供的 pydantic v1 兼容目录（`/vendor`）以满足 as2_python_api 依赖，该目录未编入镜像，镜像自包含性尚未闭环。

| 轮次 | preflight | stop_reason | 停稳距离 (m) | max/effective revision | 碰撞 | Land/disarm/offboard |
| --- | --- | --- | --- | --- | --- | --- |
| age4 | PASS | ARRIVED_5M | 3.188 | 45/43 | false | true/true/true |
| age4-b | 次样本 PASS | ARRIVED_5M | 3.445 | 41/40 | false | true/true/true |
| age4-c | rc=99 冷启动→自动重试 PASS | ARRIVED_5M | 3.567 | 40/39 | false | true/true/true |

三轮 `success=true`、无 error、无碰撞。期间钉死并闭环 STALE_OR_INVALID_INPUT 根因：长时运行（11+ h）后 AirSimNH 图像 RPC 劣化导致 depth 断流（控制 RPC 全程稳定，两条链路分开评估），恢复手段为 fresh AirSimNH + 每飞独立 `-userdir` + 重启顺序铁律；age4-c 全程 depth age n=4452、max 3.192 s / p99 2.415 s，`max_observation_age_s` 判定保持 4.0 不收紧（收紧到 3.0/3.5 会误杀实测尾部）。preflight 首采样时序缺陷已在脚本层修复：退出码作为任务启动硬门，冷启动 rc=99 自动重试。

前述矩阵第 4 项（有屏正常导航）由三飞覆盖；第 5 项故障注入与第 6 项俯视实测仍待执行。完整轮次数据、STOPPING 行判读规则与阈值量化见 [Demo validation.md](../deployment/airsim/demos/flowpilot_single/validation.md)。验证停在仿真层级，真机未执行。

## 历史依据与限制

旧 Linux RTX4090/AirSimNH/PX4 记录包含交接修复、停止 PID 调整、最终距离/退出判据，以及后续正常、取消、深度故障和低分辨率有屏分项。高度跟踪后的三次正常历史停稳距离为 3.879/3.821/4.014m；两次故障分别报告 CANCELED 和 STALE_OR_INVALID_INPUT。另有演示碰撞，不能把演示等同验收。

这些结果对应旧源码布局、镜像、私有任务入口及不同配置轮次，**不证明本次公共 Action/自包含镜像已通过**。原始完整历史及包索引按任务保留在本地 dev 材料，不能把个人任务目录当正式复跑入口；共享报告不包含私人 SSH、绝对主机路径或代理。

当前完整 Linux 启动、故障注入和清理入口已提供实现，但尚未真实运行。新规范与 releases/README 的 v0.1.2 发布状态描述存在差异，构建前必须核实真实可用镜像，而非自行假定发布摘要。跨物理设备、其他地图、RoboPi和真机均未由本轮覆盖。
