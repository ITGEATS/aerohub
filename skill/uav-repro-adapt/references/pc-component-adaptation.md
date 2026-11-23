# PC 组件适配

本规则承接已拆分的跨阶段迁移流程中的适配部分。输入必须是
[`uav-repro-preprocess`](../../uav-repro-preprocess/SKILL.md) 产出的锁定上游与基线。

## 选择迁移机制

优先目标环境原生实现；只有依赖隔离、移植成本或性能证据支持时才保留旧进程与桥接。ROS 版本或接口迁移读取 [`uav-adapt-ros`](../../uav-adapt-ros/SKILL.md)；涉及 ROS 1 时还必须读取其 [ROS 1→Humble 专项规则](../../uav-adapt-ros/references/ros1-to-humble.md)。CUDA 模型与独立环境读取 [`uav-prepare-cuda-inference`](../../uav-prepare-cuda-inference/SKILL.md)。

明确源码归属：锁定参考放 `upstream/`，活动实现和必要补丁进入项目源码；构建侧兼容补丁可重放并绑定目标版本。算法语义修改和编译兼容修改分别记录。

模型调用采用云端服务时，也按 Inference Function 接入：Tool 选择 `model_name`，认证、供应商协议与模型执行留在 Function；任务 prompt、结果解释和运动决策留在 Tool。CUDA/Houmo 的张量后端与 Cloud 的内部协议可以不同，不以插件存在推断模型跨后端可用。云端接入的现有入口与私有配置方式见 [`vision_language`](../../../src/basic_function/inference/models/vision_language/README.md)。

**完成门：** 机制选择有证据；核心、适配壳、模型 worker 和桥接进程的边界确定；补丁归属无歧义。

## 建立可复现 PC 构建

优先复用项目固定工具镜像，在外部工作区补编译并显式加载 overlay。记录镜像 ID、OS/ROS、架构、编译器、完整依赖锁、构建输入和安装前缀。源码迭代不自动升级基础镜像；接口或 ABI 变化时连同消费者重编译。

先独立构建并调用核心，再构建外层包。干净上下文中检查包发现、动态库、Python ABI、模型文件及实际入口，避免依赖开发机残留或旧安装。

只读源码挂载下，先验证本地 Python 包构建是否写 `egg-info`；需要写入时将本项目 runtime 的构建输入复制到容器临时目录，保持相对依赖路径，在目标 Python ABI 与固定环境路径中生成外部 venv。准备成功还需从最终环境加载运行，不能只验证宿主临时 venv。

缺少少量系统依赖时，可锁定发行版包版本与校验和，在一次性构建容器中解包并输出外部 overlay；共享镜像保持原 ID。运行前核查实际宿主消息依赖：Humble desktop 能构建通用 ROS 包，但不自动包含 AS2 消息；调用 AS2 协调器的 Adapter 需在匹配 AS2 overlay 中做真实导入与节点检查。

**完成门：** 从声明输入可重复构建；目标环境能调用真实核心；构建、安装和运行版本一致。

## 对照核心结果

使用与上游相同的固定输入，对比输出数量、顺序、时间、数值、约束和失败结果；容差在运行前声明。已有缺陷修复与版本迁移分别给出对照。性能测试使用单调时钟并记录预热、重复次数和统计方法。

**完成门：** 目标核心满足预声明对照，或准确保留失败项；核心实现具备可调用入口、构建入口、版本、依赖、补丁和证据索引。

## 建立 Component Harness

完成 [Component 契约](component-contract.md)，选择 Function、Behavior 或组合：

- **Function** 持续或请求式地产生状态、环境、计划或推理结果，对外使用 ROS 2，内部可有独立 worker。
- **Behavior** 接收任务 Goal，拥有反馈、结果、取消和执行资源生命周期。
- **组合** 由 Function 产生计划或参考，Behavior 负责执行；二者分别可观测，算法核心保持单一。

优先复用目标 AS2 的真实接口和扩展机制。Function 定义输入、输出、ready、health、reset、停止和失效；Behavior 定义 Goal、Feedback、Result、取消、暂停/更新、输出静默和失败交接。共同处理 namespace、QoS、frame、时钟、版本、任务世代、多机隔离及发布权。

平台输入转换在 Harness 边界显式完成。缺失的物理信息保持未知或拒绝，不以改 frame 名、改成 now、补零或真值替代。

**完成门：** 核心、Function、Behavior、平台和消费者的责任边界明确；真实 ROS 2 接口可调用；空间、时间、reset 和失败行为符合契约；没有竞争发布者。

## 对接消费者

证明目标消费者实际使用 Harness 输出。计划进入既定 Behavior；运动参考通过 AS2 控制/平台到 PX4。状态和环境能力进入声明的下游；Inference 的内部 HTTP 不替代外部 ROS 能力接口。

秒级推理提供可取消的 ROS 调用，但区分调用取消和实际执行终止。测试取消后仍占 busy、迟到结果不驱动旧任务、输入与结果保持原采集 header，并在推理前后验证年龄。私有凭证使用忽略且 owner-only 的文件或部署秘密注入；真实云端调用缺少授权凭证时，先完成本地协议/ROS 证据，明确保留真实服务与平台验证未完成，不用协议替身提升等级。

定义资源所有权、冲突、取消交接、任务完成、低速/停止和异常恢复。声明冲突只是契约，运行时互斥和恢复必须由实现与测试证明。

**完成门：** 连接证据能从真实输入追踪到核心输出和实际消费者；没有 mock、Marker、真值或旁路执行替代目标链路。

## 独立 Python 核心的原样安装

从已锁定子模块直接复用单文件算法时，优先在 Adapter 包安装原文件和许可证，不复制成第二份算法实现。`ament_python` 的 colcon `data_files` 要求源路径为相对路径；包外子模块可使用相对路径引用，运行时通过 ament share 查找安装件。实际构建后用 `cmp` 核对算法与许可证安装件，并在隔离源码挂载下实际导入核心。YOPO-Tracker 目标 EKF 已验证此方式；首次使用绝对 data_files 的构建失败，不能沿用该配置。
