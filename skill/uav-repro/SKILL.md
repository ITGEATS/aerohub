---
name: uav-repro
description: 编排无人机开源工作的完整复现流程，依次路由 Preprocess、Adapt、Verify，并在需要展示时进入可选 Demo；用于正式交给 Agent 的顶层复现入口。
---

# UAV 复现

这是正式部署时的顶层入口。它只维护阶段顺序、阶段契约和恢复规则；具体操作必须进入对应父 Skill 和其路由的子 Skill，不在这里复制规则。

## 输入

确认待复现工作、上游来源或候选来源、目标能力、目标平台（PC，或 PC 后继续 RoboPi）、期望交付物，以及是否需要 Demo/Gallery。未知项交给 Preprocess 查明，不用猜测补齐。

## 阶段编排

1. 使用 [`uav-repro-preprocess`](../uav-repro-preprocess/SKILL.md) 锁定来源、边界、原始基线和预声明验证范围。
2. 将其退出产物交给 [`uav-repro-adapt`](../uav-repro-adapt/SKILL.md)。必须先形成 PC 候选；只有用户目标包含 RoboPi 时才继续 board-adapt。
3. 将每个平台候选分别交给 [`uav-repro-verify`](../uav-repro-verify/SKILL.md)。PC 与 RoboPi 分别判定，不能相互继承成功结论。
4. 仅在需要演示或 Gallery 材料时，将已经验证的版本和证据交给 [`uav-repro-demo`](../uav-repro-demo/SKILL.md)。Demo 是可选阶段，不参与复现成功判定。

每次阶段交接都记录输入版本、输出、证据位置、未完成项和下一阶段可依赖的事实。跳过不适用阶段时说明理由；Preprocess、PC Adapt 和对应平台 Verify 不可跳过。

## 失败与恢复

- Preprocess 无法锁定来源、能力边界或完成标准：停留在 Preprocess，保留未知项及验证方法。
- Adapt 暴露上游或契约判断错误：返回 Preprocess；board-adapt 需要改变算法、ROS/AS2、传感器或组件契约时，返回 pc-adapt 建立并验证新 PC 基线。
- Verify 发现实现缺陷：返回对应 Adapt 类别；发现测试范围或能力声明错误：返回 Preprocess。修复后只复用仍与当前版本严格对应的证据。
- Demo 发现材料与实现、能力卡或证据不一致：返回 Adapt 或 Verify 修正，不能在展示层掩盖差异。

任何失败、跳过、未执行或证据缺失都保持未完成状态。只有对应平台预声明的适用验证全部通过，才能声明该平台复现成功。
