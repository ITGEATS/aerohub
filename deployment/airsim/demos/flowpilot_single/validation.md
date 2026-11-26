# FlowPilot 验证入口

当前新基线结果、历史边界和必须补跑矩阵统一见 [正式验证说明](../../../../docs/flowpilot-validation.md)。本目录已经改为公共NavigateToPose、组件镜像及Linux入口；旧服务器飞行结果不能直接覆盖此版本。当前保持partial。

2026-11-25本机实跑：新路径代码的真实CUDA完整输出对照、GPU HTTP功能、兼容v0.1.1诊断镜像的4包构建、20项Humble回归、DDS fixture及公共Action拒绝/未就绪分支通过。HTTP有效采样4/6超过100ms，WSL Vulkan仅枚举CPU llvmpipe；正式Linux GPU交付镜像、真实相机、FollowTimedPath物理执行及AirSim/PX4飞行未完成。本轮未起飞，未降低门限；证据保留于本机dev/flowpilot/local-run-20261125，详细边界及清理见正式说明。
