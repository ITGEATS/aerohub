# FlowPilot：观测、模型输出与规划交接契约

本参考挂靠既有平台传感器适配 Skill，用于接入或复核 FlowPilot。它定义模型实例的输入语义，并追踪这些语义如何进入 Planner；模型安装、ROS 传输、执行和验收分别遵循对应通用 Skill。不是独立的新 Skill，也不授权修改其他模型或共享执行层。

适用上游为 `flyingbitac/FlowPilot` 的 `f29f1bfee1399c96e5fd384276d5d7a09876286b`。以下实现事实于 2026-11-25 按本地源码核对；更换版本、配置或部署位置后重新核对。当前服务器实际状态未经本页编写过程在线验证。

## 1. 来源与责任边界

| 环节 | 正式入口 | 责任与核查结果 |
| --- | --- | --- |
| 算法 | `src/tool/navigation/planner/flowpilot/upstream/` | 锁定上游；不把网络复制到 Inference |
| 模型准备 | [README](../../../src/basic_function/inference/models/flowpilot/README.md)、[setup.sh](../../../src/basic_function/inference/models/flowpilot/setup.sh)、[prepare.py](../../../src/basic_function/inference/models/flowpilot/prepare.py)、[weights.sha256](../../../src/basic_function/inference/models/flowpilot/weights.sha256) | 独立环境、四份权重、来源与散列；清单文件名相对 artifacts |
| 模型调用 | [adapter.py](../../../src/basic_function/inference/models/flowpilot/adapter.py)、[schemas.py](../../../src/basic_function/inference/models/flowpilot/schemas.py) | 深度解码/预处理、上游运行时、模型输出 |
| 对外服务 | [InferFlowPilot.srv](../../../src/basic_function/inference/interfaces/srv/InferFlowPilot.srv)、[flowpilot_node.py](../../../src/basic_function/inference/ros2_adapter/uav_inference/flowpilot_node.py)、[flowpilot_contract.py](../../../src/basic_function/inference/ros2_adapter/uav_inference/flowpilot_contract.py) | ROS 请求验证和内部 HTTP；不做隐式 TF 或运动执行 |
| Planner | [node.py](../../../src/tool/navigation/planner/flowpilot/ros2_adapter/uav_flowpilot_planner/node.py)、[curve.py](../../../src/tool/navigation/planner/flowpilot/ros2_adapter/uav_flowpilot_planner/curve.py)、[lifecycle.py](../../../src/tool/navigation/planner/flowpilot/ros2_adapter/uav_flowpilot_planner/lifecycle.py) | 同步状态、导数、曲线、交接、任务代次和停止 |
| 部署配置 | [settings.json](../../../deployment/airsim/demos/flowpilot_single/settings.json)、[planner.yaml](../../../deployment/airsim/demos/flowpilot_single/planner.yaml) | 实际相机、坐标和运行参数；不能由旧日志替代 |
| 验收 | [正式流程与边界](../../uav-sim-e2e-validation/references/flowpilot-acceptance.md)、[validation.md](../../../deployment/airsim/demos/flowpilot_single/validation.md) | 分层检查、具体版本结果和未覆盖项 |

运行链为 AirSim 真实深度与 AS2 权威定位 → ROS Planner 请求 → ROS Inference → 内部 HTTP CUDA worker → 控制点 → Planner 转换 → FollowTimedPath → AS2/PX4 → 实际运动。工作站经 ROS 发任务；仿真 RPC 用于测量和评分，不能直接控制机体代替执行链。

## 2. 深度的物理和二进制契约

1. 相机为轴向 `DepthPlanar`，不是可视化深度或 `DepthPerspective` 射线距离。核对实际传感器桥发布代码、消息和 CameraInfo，不仅看配置名称。
2. `32FC1` 为米，`16UC1` 为毫米。按 `is_bigendian`、`step`、宽高与实际缓冲区长度解释；行 padding 不能当像素。当前 ROS 解码要求总长度等于 `height * step`，像素数不超过 1,048,576。
3. 在填补前，以“有限且 >0”为有效测量判断。无任何有效像素的整帧拒绝；稀疏零、负数、NaN/Inf 填成 20 m。有效比例没有因此获得质量保证，需另按场景声明，不能用少量有效像素掩盖严重传感器故障。
4. 模型侧将米制值裁到 20 m 后除以固定 20，再走上游预处理；网络网格为高 96、宽 160。常量 5 m 输入应得到归一化 0.25。上游其他入口的单位换算不能重复施加到已经米制的 ROS 输入。
5. 当前 settings 为 320×185、水平 FOV 90°、AirSim Pitch +15°；planner 的 `camera_pitch_deg=-15` 属于转换后的约定。必须联查光学 TF 和旋转方向，不能因为符号不同就自行反转。该实例相机与机体共置；非零杆臂需重新推导和验收。
6. resize 只改变网格，不修复 FOV、外参或训练几何。记录实测 CameraInfo、唯一采样戳率、间隙、年龄与图像有效性；图像发布率不等于新测量率。

当前 `geometry_ready()` 还有实际就绪门：CameraInfo 与深度尺寸一致，焦距为正，水平/垂直 FOV 分别距离 90°/60°小于 1°，主点距画幅中心各小于 1 像素，畸变系数接近零；光学外参需满足共置及配置旋转。尺寸可解码不代表能通过几何门。Pose 与 Twist 的历史配对要求两者测量 stamp 完全一致，再要求该配对与深度相差不超过配置 skew；仅三路都“新鲜”不足以就绪。更换状态发布者或相机配置时首先核对此约束。

## 3. 状态与时间

25 维 `state_goal` 按以下顺序组装：

| 切片 | 语义 |
| --- | --- |
| 0:3 | 局部机体系起点 P，当前为零 |
| 3:6 | 同观测时刻的机体系 V |
| 6:9 | 同观测时刻的机体系 A |
| 9:12 | 同观测时刻的机体系 J |
| 12:16 | 机体到世界的四元数，顺序 wxyz |
| 16:19 | 从观测位置指向目标的机体系相对 P |
| 19:22、22:25 | 目标 V/A；当前静止目标为零 |

Planner 从与深度配对的权威 Pose/Twist 获取位置、姿态和 V，将世界系导数旋转到机体系；A/J 由有界速度历史局部二次拟合。当前拟合使用观测时刻前约 0.16 s 的历史，至少 3 点、覆盖至少 0.035 s、严格递增且矩阵满秩。它是导数估计方法，不等于直接 IMU 测量；噪声、间断和急转情况下的误差需另行评估。

区分四个时刻：深度采样、状态采样、请求发送、未来执行交接。服务保留 `request_id` 与 `observation_header`；当前默认深度/状态偏差不超过 0.05 s、年龄不超过 0.3 s，测量不得在未来。Planner 的图像、状态历史和当前健康均独立检查；重复/倒序帧、TF 晚到、时钟回退按生命周期源码处理，不能用 now 或最新姿态覆盖原采样时间。

HTTP 默认超时 1 s 与 Planner 正式推理预算 0.1 s 是不同层级；`handoff_lead_s=0.2`、`submission_reserve_s=0.05` 还限制交付余量。所有值以本轮运行参数和配置校验为准，这些数值不是其他模型或 GPU 的通用建议。

## 4. HTTP 表示和模型输出

生产 ROS Adapter 使用 `depth_b64_gz` 与 `depth_shape`：先填补后的米制 float32 → 小端、C 行序 → zlib → base64。字段名含 gz，但实现是 zlib，不要按 gzip 解码。worker 解压长度必须与 `rows * cols * 4` 一致。旧 `depth_m` JSON 数组仍为兼容表示；两者应还原相同的 float32 测量，不重复填补或归一化。

当前 worker 需 CUDA，使用三步推理、关闭上一轨迹条件；改变这些开关需重新对照，不能仅沿用版本号。输出为 `wp_body: 48×12` 的 P/V/A/J 与 `ctrl_pts_body: 8×3` 的七阶 Bernstein 控制点。ROS 响应扁平化为对应数组，检查身份、形状和有限性。

名义曲线域 1.6 s，采样率 30 Hz，首点 0 s、末点 47/30 s。最后样本不是 1.6 s 理论终点。世界系位置旋转再平移，V/A/J 只旋转；记录 `command_speed_mps` 的模型速度条件与 `max_speed` 的执行限制，不能混为一项。

## 5. 三类检查与后处理必须分开

**原始锚点。** 对原生 1.6 s 控制多边形，起点 P 为 C0，V 为 `7(C1−C0)/1.6`，A 为 `42(C2−2C1+C0)/1.6²`。与请求使用的实测起点 P/V/A 对照；不要拿未来执行预测值或 warp 后速度作原始模型锚点标准。

**执行接续。** 时间拉伸 k 后时域为 `1.6k`，各阶导数按 k 的对应次幂缩放。滚动段以未来交接为新起点，splice 到已接纳计划在同一绝对时刻的预测参考 P/V/A，并接续 yaw。首次计划仍按原观测时间轴裁出剩余样本，不能将两种起点策略混用。提交时间在 ROS 纳秒网格上生成严格递增样本；验证执行器使用的分段五次插值，而不只比较离散点。

**路线改变。** 当前代码还有 splice 平移/初始控制点重设、世界盒裁剪与目标高度跟踪。滚动段保留最初三个控制点的起始 P/V/A；后续控制点变化仍改变路线、后续导数和障碍间距。时间拉伸上限及离散峰值估计也不能视为严格全域动力学保证。记录 raw、warp、splice、altitude/box 和执行门各自数据，不把 splice diagnostics 当成最终门限残差。

当前目标高度跟踪使后续 z 控制点向目标高度参考收敛，随后受世界盒约束；世界盒地板是防地下限，不是巡航高度。目标是否位于盒内、完整轨迹的动力学及避障安全必须另验。改变模型路线后的任务结果不能称为未修改模型策略的直接复现。

**最终制动。** `brake_segment` 从旧执行参考的完整 P/V/A 构造五次 Hermite，再升阶为七阶；理论终点 V/A 为零。不能把起点 A 强制为零以制造不连续；也不能因理论终点为零就断言末个实际样本或机体已经停稳。具体执行与停止指标归 [验收参考](../../uav-sim-e2e-validation/references/flowpilot-acceptance.md)。

### 深度新鲜度诊断与 RPC 劣化判定

当 Planner 报告 `STALE_OR_INVALID_INPUT` 时，不直接归因于网络抖动或真机姿态。按以下顺序排查：

- **mission.jsonl STOPPING 行优先**：`publish_status.ready/ages` 在 stop 决策后同 tick 末尾重算；STOPPED 状态 fresh ≠ 停机瞬间 fresh。停机根因必须比对 STOPPING 行各 ages（depth / pose / twist），不能只看终态。
- **sensor adapter 日志**：`platform_sensor_adapter` 出现 `sensor read failed: timed out` 或 `AirSim closed the RPC connection`，且 depth gap 与报错时刻吻合，即为图像 RPC 劣化。控制 RPC（默认 4780）与图像 RPC（默认 45471）独立；前者可通过 `ESTAB`（`ss -tnp | grep -E 'ESTAB.*:4780 '`）快速确认，后者无连接保持日志时不能证明其健康。
- **depth gap 量化**：用独立诊断脚本订阅实际话题名（Planner 通过 remap 订阅，不能只按配置名称推断），收集有效样本数 n、p50/p95/p99、max；fresh AirSimNH 首飞 depth max gap 实测 2.2 s，二飞 1.5 s；旧 AirSimNH（11+ h）实测 max=3.1 s 且持续 timeout——两者差距是根因不是阈值问题。
- **阈值标定**：阈值必须基于当前 fresh 栈的 max gap 加 30%–50% 裕度。旧栈 max gap 3.1 s、设 3.0 s 仍不足以覆盖；改为 fresh userdir + 阈值 4.0 s 后全绿。后续新栈稳定跑两次 max <2.0 s 时可收紧到 3.0–3.5 s。
- **悬停态探针**：空中悬停比航线中途更容易出现慢帧，用 `ros2 topic hz` 或自行导出的 sample 脚本在同样 fresh 栈下采集 max gap，不能把离线测试的 coverage 当飞行全程保证。
- **threshold yaml 全文 replace 规则**：planner yaml 的阈值参数不得手写子集；由原文件做字符串 replace 派生，python 验证关键字段值与总项数。

当前验证实例结论：fresh AirSimNH + fresh `-userdir` + 阈值 4.0 s 组合下，depth max gap ≤2.2 s，ARRIVED_5M 全绿；阈值组件见 `planner-demo-tolerant-1126b.yaml`，由原文件全文 replace 21 项派生，禁止手写子集。

## 6. 可复验入口和证据等级

除表中明确要求进入模型目录的命令外，均从仓库根执行。先满足模型 README 前置条件并加载对应环境，不能在缺依赖时将全 skip 当通过。

| 检查 | 正式入口 | 能证明什么 |
| --- | --- | --- |
| 准备 | `bash src/basic_function/inference/models/flowpilot/setup.sh`；另加 `--check-cuda` 做一次前向 | 权重与环境/加载检查；不代表服务或飞行通过 |
| 模型契约 | 在模型目录用独立 `.venv/bin/python -m unittest -v test_contract.py` | 模型输入输出与校验；核对实际通过和 skip 数 |
| 上游完整 GPU 对照 | `src/basic_function/inference/models/flowpilot/.venv/bin/python deployment/airsim/demos/flowpilot_single/check_parity.py --cuda` | 当前脚本以固定种子对照全部样本/控制点，atol=1e-5、rtol=1e-4；不验证全部后处理 |
| ROS payload | `python3 -m unittest discover -s deployment/inference/tests -p test_flowpilot_contract.py -v` | 单位、stride、端序、稀疏坏点、整帧失效、压缩与戳检查；无需 ROS runtime |
| 曲线 | `python3 -m unittest discover -s deployment/inference/tests -p test_flowpilot_curve.py -v` | 导数、曲线和制动等 CPU 回归 |
| Humble 请求与恢复 | [flowpilot_ros_smoke.py](../../../deployment/inference/tests/flowpilot_ros_smoke.py)、[lifecycle 回归](../../../deployment/inference/tests/test_flowpilot_lifecycle.py)、[sampler 回归](../../../deployment/inference/tests/test_flowpilot_sampler.py) | 前者使用 HTTP fixture；生命周期需要真实 ROS/AS2 消息，采样器需匹配 C++ 工具链；均不替代动力学 |

独立测试进程、跨容器 DDS、真权重服务和真实平台消费逐层记录。复验命令可输出到新建 log 目录，但不能依赖旧 log 中的脚本、模型或环境作为隐含前置。

## 7. 结果与适用范围

当前运行结果按 [validation.md](../../../deployment/airsim/demos/flowpilot_single/validation.md) 的版本与章节查阅。该文件含早期失败和后续修正，模型 README/清单也存在尚未同步的早期状态；不可只摘某一句“待验证”或某个 success=true 概括整条链路。

本参考不冻结“最新全部通过”结论。后续复跑需记录实际源码散列、镜像 ID、参数、地图/渲染方式、原始输入与执行反馈，按同一版本的完整矩阵判定。此前服务器文件、现在本地源码与正在运行的部署是否一致，需要实际核对。
