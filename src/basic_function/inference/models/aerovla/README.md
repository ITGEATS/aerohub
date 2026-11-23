# AeroVLA 推理模型（板端 Houmo xh2）

AeroVLA（ECCV 2026，Apache-2.0）的 openvla-7b+LoRA 策略网络在 RoboPi NPU 上的
Inference 适配。算法归属与迁移契约见
[planner README](../../../../tool/navigation/planner/aerovla/README.md)。

## 开始前准备什么

- 板端 Houmo 运行环境：`/data/local/houmo/python`（tcim_lite）与 `/data/local/houmo/lib`
  （HAL/runtime 库）——由板端系统镜像提供，属部署前提，不由本模块安装。
- 三件套 HMM 与 assets（token_embedding/id2token/assets.json）已在
  `/data/local/workspace/aerovla_infer/`（2026-10-18 量化轮产物，sha256 锁定于
  [houmo/artifacts.json](houmo/artifacts.json)）。重新量化按
  [model-migration-houmo](../../../../../../skill/model-migration-houmo/SKILL.md) 在
  x86 Linux + GPU 工具链容器执行，产物再上板。

## 服务契约（HTTP worker）

`POST /infer`：`{request_id, front_b64, down_b64, input_ids[1..512]}`
→ `{request_id, model_version, device, text, gen_ids, timings_ms}`。

- 输入两路为 PNG/JPEG base64 的 BGR 原始观察帧（前视/下视）；worker 内部完成
  224×224 squish mosaic 与双归一化（6 通道，模型预处理契约，见 `processing.py`）。
- **分词与 prompt 组装、动作反量化与落地判定不在本层**：`input_ids` 由 planner
  提供，`text` 由 planner 解析（2026-11-19 迁移裁决；与旧板端 service.py 的
  分层差异记录于 planner README 偏离清单）。
- `GET /health/live`、`GET /health/ready`（model_version 锁定三枚 HMM 哈希前缀）；
  单 worker 串行，忙时 429；非法输入 422。

## 启动与验证

打包/部署/启动步骤见 [houmo/README.md](houmo/README.md)。
数值门槛（运行前声明，不放宽）：

1. 模块级 golden 对照：`aerovla_infer/smoke.py`（vision/prefill cosine ≥ 0.98）；
2. worker 契约与贪心确定性：`houmo/tests/http_smoke.py`；
3. 行为层对照阶段 1 legacy 基线：在 planner/ROS 层验证（见迁移计划阶段 3/5）。

## 边界

- 与 bf16 上游 GPU 运行时的逐张量对照**未执行**（无可用 GPU 主机），保留为
  未完成缺口；golden 是导出期参照，不等同上游运行时对照。
- warmup 仅覆盖 vision 模块；prefill/decode 首次真实请求才过设备（记录，不宣称全链路预热）。
- 合成图冒烟不证明真实传感器行为；传感器闭环与飞行验收在 Demo 层。
