---
name: uav-repro-adapt
description: 将已完成预处理的无人机算法先适配为可验证的 PC 组件，再按需迁移到 RoboPi；区分 pc-adapt、board-adapt 和多机共享规则。
---

# 复现适配

Adapt 分为三个有明确边界的类别：

1. **`pc-adapt`**：在固定 PC/onboard-computer 基线上完成算法、ROS/AS2、传感器、Component Harness、消费者接入、组件正式化和能力卡。
2. **`board-adapt`**：只接收已完成的 PC 组件和契约，处理目标平台代码差异、交叉编译、NPU、设备操作和板端安装。
3. **`shared`**：只保存确实同时改变 PC 与板端拓扑的规则；当前仅为多机部署。

`board-adapt` 必须以 `pc-adapt` 的可重复候选和证据为输入，不在板端重新迁移算法语义、ROS/AS2 接口、传感器语义、组件契约或能力卡。板端若暴露必须修改这些契约的问题，返回 `pc-adapt` 修改、验证并形成新的 PC 基线，再继续板端迁移。

## 模式规则

- `pc-adapt` 必须读取[PC 组件适配](references/pc-component-adaptation.md)和[Component 契约](references/component-contract.md)。
- `board-adapt` 必须读取[从 PC 迁移到 RoboPi](references/robopi-adaptation.md)。
- 多机任务还必须读取 [`uav-integrate-swarm-infrastructure`](../uav-integrate-swarm-infrastructure/SKILL.md)，按实际阶段选择 `pc_simulation` 或 `hybrid_board`。
- 适配完成后进入 [`uav-repro-verify`](../uav-repro-verify/SKILL.md)，不能在本阶段自行提升成熟度。

## `pc-adapt` 路由

| 子 Skill | 触发条件 | 必需输入 | 输出 | 失败返回 |
| --- | --- | --- | --- | --- |
| [`uav-adapt-planner`](../uav-adapt-planner/SKILL.md) | 工作包含路径、轨迹、滚动、多机或策略型 Planner | Preprocess 的能力/执行边界、上游 Planner、目标执行接口 | 保留规划与执行语义的 Planner 候选 | 架构边界不明返回 Preprocess；接口或实现问题留在 pc-adapt |
| [`uav-adapt-platform-sensors`](../uav-adapt-platform-sensors/SKILL.md) | 算法消费平台传感器，或数据源、时间、坐标、标定、扫描/地磁模型不同 | 算法输入契约、PC 仿真数据源、目标采样和坐标语义 | 已适配的传感器输入与语义证据 | 数据契约不明返回 Preprocess；实现问题留在 pc-adapt |
| [`uav-adapt-ros`](../uav-adapt-ros/SKILL.md) | 存在 ROS 版本、消息、参数、QoS、时间、坐标或传输差异，包括 ROS 1→Humble | 锁定上游接口、目标 Humble 接口、消费者要求 | 原生 ROS 2 Humble 接口与迁移记录 | 能力边界不明返回 Preprocess；不得转移到 board-adapt 解决 |
| [`uav-integrate-as2-extensions`](../uav-integrate-as2-extensions/SKILL.md) | 能力需接入 AS2 Planner、Behavior、State Estimator 或 Perception 扩展点 | 架构归属、算法候选、目标 AS2 扩展契约 | 可注册、加载和调用的 AS2 扩展候选 | 归属错误返回 Preprocess；实现问题留在 pc-adapt |
| [`uav-build-pc-runtime`](../uav-build-pc-runtime/SKILL.md) | 所有 PC 复现；需要锁定源码、依赖、工具链并生成自包含产物 | 锁定源码、依赖、基础镜像、构建入口与目标架构 | 可追溯、可重复、自包含的 PC 构建产物 | 来源不明返回 Preprocess；接口/算法问题返回对应 pc-adapt Skill |
| [`uav-prepare-cuda-inference`](../uav-prepare-cuda-inference/SKILL.md) | PC 候选包含 CUDA 推理、权重或模型隔离环境 | 模型来源、权重、CUDA 基线和 Inference 接口 | 可加载的 CUDA 模型环境及模型证据 | 模型契约不明返回 Preprocess；接入或构建问题留在 pc-adapt |
| [`uav-package-pc-runtime`](../uav-package-pc-runtime/SKILL.md) | 实现需成为可安装、选择、启停和回滚的正式 PC 运行组件 | 已构建 PC 候选、运行入口、依赖与生命周期要求 | 清单、注册、生命周期、版本和回滚完整的组件候选 | 构建身份问题返回构建 Skill；接口问题返回对应 pc-adapt Skill |
| [`uav-create-capability-cards`](../uav-create-capability-cards/SKILL.md) | PC 实现已具备可声明的真实能力，或能力契约发生变化 | 实现接口、硬件/资源要求、已验证和未验证事实 | 与实现一致的机器可读能力卡 | 声明缺少实现依据时返回对应 pc-adapt Skill；不以卡片补实现 |

## `board-adapt` 路由

以下 Skill 都必须取得同一版本的 PC 候选、PC 验证证据和既有接口契约。

| 子 Skill | 触发条件 | 必需输入 | 输出 | 失败返回 |
| --- | --- | --- | --- | --- |
| [`uav-cross-compile-ros-for-ohos`](../uav-cross-compile-ros-for-ohos/SKILL.md) | PC ROS 2 组件需在 OpenHarmony/RoboPi 构建 | 已验证 PC 源码与契约、OHOS SDK/NDK、目标依赖 | 目标架构 ROS 2 产物和构建证据 | 若需改变 ROS/算法契约，返回 pc-adapt；纯平台构建问题留在 board-adapt |
| [`uav-install-robopi-runtime`](../uav-install-robopi-runtime/SKILL.md) | 目标产物需要在 RoboPi 安装、启停、卸载或回滚 | 已交叉编译产物、设备与安装布局、生命周期入口 | 可管理的板端安装和简单加载证据 | 产物错误返回交叉编译/Houmo；接口语义错误返回 pc-adapt |
| [`uav-access-robopi-with-hdc`](../uav-access-robopi-with-hdc/SKILL.md) | 需要设备发现、远端命令、传输、日志或端口转发 | 目标设备、HDC 环境和待执行操作 | 可追溯的设备操作结果或连接诊断 | 返回调用它的 board-adapt/Verify 步骤；工具成功不代表迁移成功 |
| [`uav-compile-model-for-houmo`](../uav-compile-model-for-houmo/SKILL.md) | 已验证 PC ONNX 模型需在 Houmo NPU 执行 | PC 模型与基准、固定工具链、量化输入和外部契约 | HMM 产物、量化/编译记录和板端接入候选 | 模型契约或精度基准变化返回 pc-adapt；工具链问题留在 board-adapt |

## `shared` 路由

| 子 Skill | 触发条件 | 必需输入 | 输出 | 失败返回 |
| --- | --- | --- | --- | --- |
| [`uav-integrate-swarm-infrastructure`](../uav-integrate-swarm-infrastructure/SKILL.md) | 工作需要多机 PC 仿真或 PC+RoboPi 混合拓扑 | 车辆集合、角色映射、PC/板端候选和目标部署模式 | `pc_simulation` 或 `hybrid_board` 拓扑、每机实例与协同链路 | 单组件问题返回对应 pc/board-adapt；拓扑问题留在 shared |

能力卡在 `pc-adapt` 中建立并随 PC 实现更新；Verify 只核对真实结果和成熟度，Demo 消费卡片。`board-adapt` 继承并核对该契约，不重新定义接口。

## 退出产物

`pc-adapt` 交付可构建、可调用、可部署和可验证的正式 PC 组件候选。`board-adapt` 另交付目标架构产物、安装/卸载/回滚入口和与 PC 基线可比较的板端候选。两者均须保留未完成项，不能把编译、安装或进程存活写成复现成功。
