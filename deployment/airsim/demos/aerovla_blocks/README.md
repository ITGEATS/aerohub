# AeroVLA Blocks

当前入口由 AeroVLA Tool 顺序调用原生 `GoToBehavior`，检测到 LAND 后调用
`LandBehavior`；不创建 FollowTimedPath 会话。任务配置为 `task.json`。
模型动作、收敛及取消语义见 [执行适配核查](../../../../docs/AeroVLA-GoTo执行适配核查.md)。

`flight_chain_case.py` 已对齐 Tool backend、GoTo Action 状态和逐机停止确认。
formal/cancel 用例观察或取消 GoTo；fault 用例检查不可达推理端点的失败收口。
Tool 已成功降落时客户端跳过额外 Land，否则停止确认后由客户端调用 Land。
仍通过 PX4 armed 状态独立检查解除武装，不将 Land 返回值等同于该状态。
这些脚本调整仅完成静态检查，当前入口的飞行验收尚未执行。

`run_case.ps1` 依赖未随仓库交付的 Windows/WSL 本地 harness，路径通过
`AEROVLA_HARNESS_ROOT`、`AEROVLA_HARNESS_WSL_ROOT` 提供，板端地址通过
`ROBOPI_IP` 提供。复用前须确认 harness 启动 GoToBehavior、LandBehavior、
当前 Registry 及依赖，并保存它的版本与启动配置。此入口尚不构成独立可复现部署。
新证据保存到 `dev/aerovla-goto/evidence/`。

`validation.md`、`validation.json` 保留 2026-11-21 的 FollowTimedPath 历史结果，
不证明当前 GoTo/Land 入口通过验收，也不作为当前用例的验收门槛。
飞行验收后续单独开展，执行前须重新声明适用矩阵和指标。
