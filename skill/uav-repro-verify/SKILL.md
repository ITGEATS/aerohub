---
name: uav-repro-verify
description: 编排 PC 与 RoboPi 两类独立验证，路由到对应子 Skill，并维护共同成熟度、证据和结论规则。
---

# 复现验证

Verify 独立于实现阶段。适配完成、编译成功或组件可启动都不能自行提升成熟度。

## 验证路由

从原组件迁移来的完整矩阵与成熟度规则见[组件验证](references/component-verification.md)和[验证 Harness](references/validation-harness.md)。

| 子 Skill | 触发条件 | 必需输入 | 输出 | 失败返回 |
| --- | --- | --- | --- | --- |
| [`uav-validate-pc-runtime`](../uav-validate-pc-runtime/SKILL.md) | 所有 PC 复现候选 | 同一版本的 PC 源码、产物、配置、能力卡、预声明矩阵和运行入口 | 有屏 AirSim + PX4 SITL 证据、逐项状态和 PC 成熟度结论 | 实现缺陷返回 pc-adapt；范围/门槛错误返回 Preprocess |
| [`uav-validate-robopi-runtime`](../uav-validate-robopi-runtime/SKILL.md) | 用户目标包含 RoboPi，且对应 PC 候选已完成验证 | PC 基线证据、同源板端产物、固定输入、真实设备链路和板端矩阵 | 固定输入对照、真实链路/资源/恢复证据及独立板端结论 | 平台实现问题返回 board-adapt；契约/算法问题返回 pc-adapt；范围错误返回 Preprocess |

板端验证不重复与板端变化无关且证据充分的 PC 测试，但必须证明复用证据与当前源码、配置和产物版本对应。

## 结论

逐项报告通过、失败、跳过、未执行和证据缺失。只有全部适用项在同一实现、配置和产物版本上通过，才能声明对应平台复现成功。PC 与 RoboPi 分别判定；一个平台的 `reproduced` 不自动传递到另一个平台。

验证结果用于更新组件清单和能力卡的成熟度事实，但不得改写其接口契约来适配测试结果。
