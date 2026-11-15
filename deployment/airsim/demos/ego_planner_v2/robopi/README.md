# RoboPi 单机入口

操作、构建、候选安装与回退见 [Demo说明](../README.md#robopi-单机)，历史验收见
[VALIDATION.md](../VALIDATION.md)。此目录最初由 STEP23 已验收快照整理，现合入混合阶段的每机监管、诊断与顺序停止增量。
单机 CLI 仍仅拥有 Drone0。默认 vehicle=drone0 使用每机接口；旧单机入口的 vehicle="" 全局接口兼容方式保留，只用于匹配的历史安装。候选目录与接口模式由本地配置明确指定，不根据目录名猜测；启动时保留原有部署清单、包集合和文件哈希检查。混合拓扑由 [hybrid 入口](../hybrid/README.md) 管理。

状态未知或停止未确认时保留仿真/时钟/控制服务，检查本轮 run ID、PID及启动时间，
通过 `run.py status/stop/collect --output <session>` 管理。回退须先安全停止当前会话，
选择保留候选的匹配配置，再启动新会话；不替换运行中库、不转移旧规划任务。
