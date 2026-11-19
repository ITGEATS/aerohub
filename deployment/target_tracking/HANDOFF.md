# 场景分支接入交接

## 已实现

- Inference Engine 注册 `grounding-dino-tiny`，独立模型环境、HTTP runtime 与 ROS 服务；默认端口 8003。
- Detect / Tracking 对外查询统一为 `query`，替代原 `category` 请求字段；输出 category 仍是模型实际标签。接口有破坏性变更，所有消费者须重新构建匹配 overlay。
- Tracking Demo 默认 Grounding DINO / `car`（模型 prompt `car.`）；共享 Detect Demo 仍默认 YOLO，无隐式后端回退。
- Tracking 不控制运动，Mission 编排 Tracking 与 Navi；本轮 Mountains 测试只有原生起降和观察，没有 Navi 追逐。

## 构建与启动

按 [Demo README](README.md#dino-default) 准备模型和统一 query overlay；不要混用历史 tracking overlay 或旧消息包。模型权重、`.venv`、构建工作区和运行日志不入 Git，换机器需重新准备。

## 最新真实验证与未完成项

见 [受控初态验证](airsim/mountains/VALIDATION-controlled-initial-20261116.md)。

- 原生起飞3m、稳定悬停、Land、landed/disarmed及相机/TF核对通过。
- 第二轮25次真实CUDA推理均有框，但所有三维投影被框深度策略拒绝；没有有效三维Tracking。
- 静态门未通过，车队未启动，12秒移动观察未执行；追踪任务尚未验收成功。
- 已停车并清理本轮资源，未改变既有容器。

场景分支应先用新场景验证有效静态三维观测，再移动目标，最后才接 Navi。保留深度、新鲜度、停止及降落门槛。

## Elastic-Tracker 讨论边界

仅作源码调研，未加入依赖、未迁移算法。其固定目标高度/检测框高度估距不读取深度图；尺寸先验测距尚未实现。不能将 `car` query 当作尺寸先验，也不能直接把其固定高度公式用于俯视车辆。是否采用及如何约束观察角度留待新场景明确后实施。

## 提交前复核

本轮重新执行 Mission 7项和 Demo 配置6项回归；另运行统一 query ROS 回归。此前真实 CUDA、回放与平台测试的范围和失败证据以各 VALIDATION 文档为准，不将接口通过等同于移动追踪通过。
