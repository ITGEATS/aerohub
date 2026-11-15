# dev 场景管理集成约定

本分支维护统一的仿真生命周期入口、资源解析与 ProjectAirSim 扩展。
集成目标是保留旧场景和下游业务，通过明确的入口迁移继续使用；不要求所有内部路径保持不变。

## 本次集成

2026-11-15：接入 origin/dev `cfb685c`，保留其能力目录重组、推理/检测及其他功能变化。
dev 曾在 `b9dc353` 合入 scene-player，并在 `1b338a5` 回退；本次明确恢复场景 manager、
ProjectAirSim、场景配置、基础场景检查及 pairsim-build，不将回退视作仍应删除这些功能。
恢复的 ProjectAirSim Demo 引用随 dev 迁移到 `src/tool/navigation/planner/` 和
`src/basic_function/inference/`。这些 Demo 的历史飞行结果不自动代表新布局已完成端到端飞行复验。

## 用户已有场景如何继续使用

- 保留原生 AirSim JSON profile/settings 格式；不要求为统一 manager 改成 ProjectAirSim 或 YAML。
- `simulator/airsim/airsim.sh` 经过统一 manager，再委托原 AirSim 实现；
  直接调用 `simulator/airsim/manager/manager.py` 的现有 Demo 也继续有效。
- 新入口为 `simulator/simulator.sh`。它处理后端分发；原生仿真协议、飞控、传感器与
  业务算法的归属不变。ProjectAirSim HUD 与扩展 RPC 不接管原生 AirSim 协议。
- 本机资源优先级为：共享 resources → 用户级 resources → 旧 AirSim local → 新根目录 local。
  新旧本机配置共存时仅同名项按此顺序覆盖，具体路径见 [配置说明](../../config/README.md)。
  原 AirSim resources 模块导入转交统一解析器，避免两个入口选到不同路径。
- `status/stop` 从保存的运行状态判断后端及进程归属，不要求原 profile 仍然存在或可解析。
  项目不因此扩大进程终止范围；外部编辑器实例只停止本会话辅助进程。
- 新增 `runtime.json` 与 backend 标识属于附加运行信息；原 state/settings 及旧运行目录仍可读取。

## 分支维护与合入检查

1. 定期合并 dev，在本分支消化冲突；算法和能力布局以 dev 的最新归属为准。
2. 仅在必要接线处调整其他 Demo，不用 ours/theirs 整边覆盖，不把历史回退当作自动恢复。
3. 冲突处理后核对启动入口 → profile/settings → resource → manager → 下游连接。
   Git 无冲突不代表该链路已经接通。
4. 核对本机未跟踪的环境/权重和已有上游检出；迁移前验证内容，保留用户修改。
   不将它们纳入提交，不用清理整个旧目录解决路径问题。

## 回归入口和本轮结果

测试依赖单独放在 `simulator/manager/requirements-test.txt`，可安装到自选局部 venv：

```bash
python -m unittest discover -s simulator/manager/tests -q
python -m unittest discover -s deployment/swarm/tests -q
```

本轮局部环境使用 PyYAML 6.0.3、msgpack 1.2.4、Pillow 12.3.0；36 项 manager/client
测试与 33 项 swarm 测试通过。新增检查包括本机资源优先级、旧 profile 缺失后的
status/stop、外部 ProjectAirSim 会话停止，以及 NH、CAT-ORA、FAST-LIVO2、swarm
四个 AirSim profile 在统一入口/旧入口的解析和检查结果一致。

profile 检查使用 `/bin/true` 作为可执行文件占位，仅验证配置与分发契约；不会启动
仿真器，不能当作场景实际启动或飞行验收。原始日志在被忽略的
`log/projectairsim/dev-compatibility/`。本轮没有重启已有仿真、重建共享镜像，
也没有声称完成 dev 全部 Demo 的运行或飞行验收。
