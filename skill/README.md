# AeroHub Skills

本目录保存无人机工作复现的正式 Skill。本文只负责组织、选择和维护边界；执行时仍须完整读取所选 `SKILL.md` 及其明确要求的 references。

## 调用层级

```text
uav-repro
├── uav-repro-preprocess
├── uav-repro-adapt
│   ├── pc-adapt
│   ├── board-adapt
│   └── shared
├── uav-repro-verify
└── uav-repro-demo（可选）
```

正式部署由 Agent 调用 `uav-repro`。开发时可直接调用四个阶段父 Skill，必要时可按明确路径调用任务/工具 Skill。实现目录保持扁平；父 Skill 必须列出可用子 Skill、触发条件、必需输入、输出和失败返回，以减少选择噪声。

`.agents/skills/` 只启用顶层 Skill 和四个父 Skill。它只保存指向本目录的相对软链接，不复制实现，也不用于表达多重归属。

## 划分轴

Skill 同时从四个轴描述：

1. **编排层级**：顶层入口、阶段父 Skill、任务/工具 Skill、reference/script。
2. **生命周期**：Preprocess、Adapt、Verify、可选 Demo。
3. **适配目标**：PC、RoboPi 板端、跨平台共享。
4. **能力性质**：领域任务、工程工具、验证或展示构建。

Skill 是否成立取决于它是否对应可复用、边界明确、具有独立输入输出或失败语义的实际步骤，不取决于篇幅。HDC 等工程工具可以是 Skill。复杂 Skill 使用简洁入口加条件化 references；简单工具不为形式强行拆分。规则级消融用于判断 Skill 内规则的贡献，不用于删除实际复现阶段。

## Skill 目录

| 阶段/类别 | Skill | 触发与作用 | 主要输入 → 输出 | 调用方/平台 |
| --- | --- | --- | --- | --- |
| 顶层 | `uav-repro` | 编排完整复现及失败恢复 | 复现目标 → 分阶段产物、证据与结论 | 正式 Agent / 全流程 |
| Preprocess 父 | `uav-repro-preprocess` | 锁定来源、边界、基线和验证范围 | 用户目标、上游候选 → 复现契约 | 顶层或开发者 / 通用 |
| Preprocess | `uav-analyze-repro-architecture` | 确定能力、进程、所有权与复用边界 | 上游与目标能力 → 架构归属 | Preprocess / 通用 |
| Preprocess | `uav-build-projectairsim-scene` | 构建、烘焙和启动 ProjectAirSim 场景/HUD | 场景来源与环境 → 可启动场景 | Preprocess / PC |
| Adapt 父 | `uav-repro-adapt` | 路由 PC、板端和多机共享适配 | 复现契约 → 待验证候选 | 顶层或开发者 / 通用 |
| PC Adapt | `uav-adapt-planner` | 保留路径、轨迹、滚动、多机和策略执行语义 | Planner 与执行契约 → Planner 候选 | Adapt / PC |
| PC Adapt | `uav-adapt-platform-sensors` | 适配数据源、时间、坐标、标定、扫描和地磁模型 | 输入契约与仿真源 → 传感器接入 | Adapt / PC |
| PC Adapt | `uav-adapt-ros` | 适配 ROS 消息、参数、QoS、时间、坐标、传输及 ROS 1→Humble | 上游接口与消费者 → Humble 接口 | Adapt / PC |
| PC Adapt | `uav-integrate-as2-extensions` | 接入 AS2 扩展点并核对注册、加载和执行 | 算法候选与 AS2 契约 → AS2 扩展 | Adapt / PC |
| PC Adapt | `uav-build-pc-runtime` | 锁定来源、依赖和镜像，构建自包含产物 | 源码与工具链 → 可重复 PC 产物 | Adapt / PC |
| PC Adapt | `uav-prepare-cuda-inference` | 准备 CUDA 模型、权重和隔离环境 | 模型与 CUDA 基线 → 可加载推理环境 | Adapt / PC |
| PC Adapt | `uav-package-pc-runtime` | 固化清单、注册、生命周期、版本和回滚 | PC 候选与运行入口 → 正式组件候选 | Adapt / PC |
| PC Adapt | `uav-create-capability-cards` | 为工作生成并维护能力、接口、硬件和资源契约 | 真实实现事实 → 机器可读能力卡 | Adapt / PC |
| Board Adapt | `uav-cross-compile-ros-for-ohos` | 处理平台差异并交叉编译 ROS 2 组件 | 已验证 PC 源码与 OHOS 工具链 → 目标产物 | Adapt / RoboPi |
| Board Adapt | `uav-install-robopi-runtime` | 安装、启停、卸载和回滚目标产物 | 目标产物与设备布局 → 可管理安装 | Adapt / RoboPi |
| Board Adapt | `uav-access-robopi-with-hdc` | 设备发现、命令、传输、日志和端口转发 | 设备与操作 → 设备操作/诊断结果 | Adapt、Verify / RoboPi |
| Board Adapt | `uav-compile-model-for-houmo` | 将 PC ONNX 量化编译为 Houmo HMM | 已验证模型与工具链 → NPU 候选 | Adapt / RoboPi |
| Shared Adapt | `uav-integrate-swarm-infrastructure` | 提供 PC 多机与混合板端的完整多机基础设施 | 车辆/角色/候选 → 实例和协同链路 | Adapt / PC+RoboPi |
| Verify 父 | `uav-repro-verify` | 路由平台验证并维护共同证据/成熟度规则 | 平台候选与预声明矩阵 → 平台结论 | 顶层或开发者 / 通用 |
| Verify | `uav-validate-pc-runtime` | 有屏 AirSim + PX4 SITL 完整验收 | PC 候选与矩阵 → PC 证据和结论 | Verify / PC |
| Verify | `uav-validate-robopi-runtime` | 固定输入对照、真实链路、资源和恢复验收 | PC 基线与板端候选 → RoboPi 证据和结论 | Verify / RoboPi |
| Demo 父 | `uav-repro-demo` | 路由可选展示材料与本地整站 | 已验证版本与证据 → Demo/Gallery 产物 | 顶层或开发者 / 通用 |
| Demo | `uav-create-gallery-entry` | 生成单项目 Gallery 正文、图示、素材和配置 | 实现、卡片与证据 → 项目条目 | Demo / 内容 |
| Demo | `uav-build-gallery-site` | 登记、同步、检查并构建本地 Gallery 整站 | 项目条目与站点配置 → 本地站点 | Demo / 网站 |

Board Adapt 的四个 Skill 都以同一版本且已经完成验证的 PC 候选为前提。若板端限制迫使算法、ROS/AS2、传感器或组件契约变化，必须返回 PC Adapt 建立并验证新基线。当前只有多机基础设施属于 shared。

Demo 是可选阶段，不决定复现成功。当前没有生产 Gallery 发布 Skill；`uav-build-gallery-site` 的边界止于本地构建和检查。

## 命名规范

任务/工具 Skill 使用 `<domain>-<verb>-<specific object>[-target]`。名称应直接说明动作和对象，保留 ROS、AS2、HDC、CUDA、Houmo、AirSim、Gallery 等有辨识度的领域名词，避免用 `component`、`deployment` 等低信息量词作为主要对象。

仓库没有按旧 Skill 名直接调用的稳定接口，因此重命名不保留空兼容入口。旧名称只存在于 Git 历史，不写入当前状态文档。

## 无损整理与自迭代

1. 可缩写、重排和合并语义等价表述，但必须保留适用条件、强制程度、禁止项、例外、验证要求和已知边界。
2. 每段原文必须映射到至少一条凝练规则；合并时分别保留稳定来源映射，语义覆盖率为 100%。无法证明等价时不合并。
3. 入口必须明确路由到条件适用的 reference；内容存在但不可发现仍视为信息损失。
4. 删除语义、改变范围或调整强度，必须等待规则级消融结果和人工确认。
5. 重复规则只保留一个权威位置；不保留没有独立规则和兼容必要的空 Skill。

每次复现后按 `AGENTS.md` 和贡献指导回顾实际过程。只把已验证、可复用且边界明确的经验写入正式 Skill；失败尝试和待验证方案必须标明状态。新增 Skill 前先判断能否归入现有边界，并由用户确认。本文随组织结构更新，Skill 名称、分类或调用契约变化时必须同步维护。
