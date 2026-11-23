---
name: uav-repro-demo
description: 将已验证的无人机能力组织为可重复运行、可观察和可发布的 Demo，并生成与实际能力和证据一致的 Gallery 内容。
---

# 复现 Demo

本阶段只消费已经由 [`uav-repro-verify`](../uav-repro-verify/SKILL.md) 判定的组件版本、能力卡和证据，不承担组件适配、板端迁移、部署安装或成熟度提升。

## Demo 入口

为目标场景提供明确的启动、就绪、任务调用、观察、停止和清理入口。Demo 参数与历史默认配置隔离，固定组件版本和拓扑；展示层不得使用真值、mock、旁路控制或旧证据冒充正式能力。

展示需要的中间表示必须来自正式组件接口或能力卡中明确标记的 demo 输出。可视化成功不等于算法、下游消费或故障测试通过。

## 路由

| 子 Skill | 触发条件 | 必需输入 | 输出 | 失败返回 |
| --- | --- | --- | --- | --- |
| [`uav-create-gallery-entry`](../uav-create-gallery-entry/SKILL.md) | 需要为一个已验证项目生成 Gallery 内容和素材 | 已验证版本、源码/正式说明、能力卡、证据和展示资源 | 单项目 Markdown、配置和可追溯素材 | 能力或证据不一致返回 Adapt/Verify；不在展示层补写能力 |
| [`uav-build-gallery-site`](../uav-build-gallery-site/SKILL.md) | 单项目条目已完成，需要登记、同步并检查整站 | 已生成条目、Gallery 配置、现有站点结构 | 本地构建通过的 Gallery 站点及检查结果 | 内容问题返回条目 Skill；站点问题留在本 Skill |

当前没有生产环境 Gallery 发布 Skill，本阶段结束于本地整站构建和检查。Demo 引用的组件清单、能力卡、版本和证据必须一致；发现不一致时返回 Adapt 或 Verify 修正。

交付可重复入口、支持的场景与参数、展示内容、停止清理方式、对应证据和明确限制。
