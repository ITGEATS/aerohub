---
name: uav-build-pc-runtime
description: 为无人机算法及 AS2 扩展建立固定工具镜像内的可复现 PC 构建，锁定源码、依赖、工具链和自包含产物。
---

# 构建 PC 运行时

用于把已明确边界的算法和接入实现构建为可追溯、可重复、自包含的 PC 候选。构建成功只产生待验证产物，不等于仿真、飞行或复现成功。

## 输入

核实用户指定工作区、源码与子模块状态、基础镜像、目标架构、编译器、ROS 版本、依赖来源、构建入口和磁盘。跨工作区同步前核对差异；构建脚本名字不是从空机可复现的证据。

## 规则路由

首次建立 PC 构建时全部读取；后续按变更范围读取，但不得遗漏跨域约束。

1. [源码与依赖](references/source-and-dependencies.md)：上游、Fork、子模块、版本锁定、路径迁移和 ROS 包隔离。
2. [容器与产物](references/container-and-artifacts.md)：固定镜像、自包含交付、构建身份、GPU 环境和多角色镜像。
3. [构建操作](references/build-operations.md)：包发现、并发、缓存、远程构建和中断恢复。
4. [构建案例](references/build-cases.md)：已验证案例、失败边界和可复用偏离模式。

涉及 CUDA 模型、权重、独立 uv 环境或模型 README/setup 时，还必须执行 [`uav-prepare-cuda-inference`](../uav-prepare-cuda-inference/SKILL.md)。

## 工作流

1. 锁定源码、依赖、工具链、基础镜像和实际构建上下文，记录未提交差异与外部前置条件。
2. 按源码所有权决定 Adapter、项目自有实现或 Fork 子模块归属，不以临时补丁、ROS 1 runtime 或宿主残留依赖作为最终来源。
3. 在固定 `onboard_computer` 基线上构建依赖、算法和 Adapter；接口/ABI 变化时重编译受影响消费者。
4. 检查 install tree、包前缀、动态库、入口、overlay、挂载和输入摘要，证明实际运行项来自当前构建。
5. 执行相应安装/加载检查，记录缓存命中、复用与重编译范围；将构建身份交给 [`uav-package-pc-runtime`](../uav-package-pc-runtime/SKILL.md)。

## 完成门槛

交付可执行构建命令、Dockerfile/Compose、源码与依赖来源、基础和最终镜像 ID/digest、平台架构、补丁/Fork 归属、产物清单、构建测试日志索引和前置条件。运行容器不得依赖宿主仓库、源码 workspace、build/install overlay 或依赖目录。

只有来源可获取、依赖可解析、干净上下文可重建、产物与当前输入对应且安装/加载检查通过，才能称为可重复 PC 构建。产物相同、干净构建通过和任务验证通过是不同结论。
