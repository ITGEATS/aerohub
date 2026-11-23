# 从 PC 迁移到 RoboPi

`board-adapt` 只接收已经在 PC `onboard_computer` 基线上形成正式组件候选、接口契约和可重复证据的工作。PC 未完成时返回 `pc-adapt`；不在板端重新迁移算法语义、ROS/AS2 接口、传感器语义、组件契约或能力卡。

板端应保持 PC 已确定的外部契约。若目标平台限制迫使接口或算法语义改变，先停止板端迁移，返回 `pc-adapt` 修改契约、消费者与 PC 实现并重新验证；形成新 PC 基线后再继续。不能只在 RoboPi 上维护一套分叉契约。

## 1. 冻结 PC 参考

记录 PC 源码和子模块提交、镜像 digest、组件版本、启动配置、固定输入、接口、数值结果、频率、延迟、资源占用和故障行为。固定输入须能在 PC 与板端重复使用；PC 已通过不自动使板端通过。

## 2. 盘点板端与依赖差异

使用 [`uav-access-robopi-with-hdc`](../../uav-access-robopi-with-hdc/SKILL.md) 获取真实 OS、架构、ROS/TypeSupport、Python ABI、NPU runtime、资源、传感器、权限和安装位置，不根据产品名猜测。递归列出源码、ROS 包、动态库、Python 扩展、模型、配置、设备 runtime 和外部服务，分别标记为板端已有、可交叉构建、需替换后端、需修改上游或无法实现。

## 3. 实现板端候选

- ROS 及项目包使用 [`uav-cross-compile-ros-for-ohos`](../../uav-cross-compile-ros-for-ohos/SKILL.md) 的正式工具链和 sysroot 构建，不复制 x86_64 产物。
- CUDA/ONNX 模型只有在算子和数值边界适用时使用 [`uav-compile-model-for-houmo`](../../uav-compile-model-for-houmo/SKILL.md)；模型转换不代表完整组件迁移。
- 目标平台代码差异在本流程内通过条件实现、平台后端、配置或可追溯 Fork 处理，但必须保持 PC 外部契约；需要改变契约时按前述规则返回 `pc-adapt`。
- 多机任务使用 [`uav-integrate-swarm-infrastructure`](../../uav-integrate-swarm-infrastructure/SKILL.md) 的 `hybrid_board` 模式，并验证板端角色身份。

## 4. 正式化与部署候选

组件 ID、接口语义和能力卡继承 PC 候选。板端只记录目标平台产物、资源和运行限制；如需改变外部契约，返回 `pc-adapt` 建立新版本并同步消费者。使用 [`uav-install-robopi-runtime`](../../uav-install-robopi-runtime/SKILL.md) 生成安装、启动、停止、卸载和回滚入口。

交付目标架构产物、source manifest、依赖闭包、配置、模型校验值、资源与权限要求，以及与 PC 基线对应的板端测试输入。交叉编译成功、文件传输、包安装、进程存活或 topic 存在都不能写成板端复现成功；结论由 [`uav-repro-verify`](../../uav-repro-verify/SKILL.md) 给出。
