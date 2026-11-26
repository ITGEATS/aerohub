# FlowPilot 自包含交付镜像

先读 [模型说明](../../../src/basic_function/inference/models/flowpilot/README.md) 和 [单机 Demo](../../airsim/demos/flowpilot_single/README.md)。本镜像包含 worker 的独立 Python 环境、ROS 接口、FlowPilot Planner、NavigationModule 和 FollowTimedPath，运行时不挂载主仓源码、宿主 venv 或 build/install overlay。

构建前初始化锁定子模块。确认实际可获取的 onboard v0.1.2 与其配套 flight_board v0.1.0；发布说明存在状态不同步，镜像可用性与 digest 尚需现场核实。另选一个可获取、固定版本或 digest、能够读取提交锁文件的 Linux uv 镜像，不使用漂移的 latest。

```bash
python3 deployment/inference/flowpilot/build.py --base-image VERIFIED_ONBOARD_REFERENCE --uv-image PINNED_UV_REFERENCE --tag flowpilot-component:dev --weights-dir PREPARED_WEIGHT_DIRECTORY --dry-run
python3 deployment/inference/flowpilot/build.py --base-image VERIFIED_ONBOARD_REFERENCE --uv-image PINNED_UV_REFERENCE --tag flowpilot-component:dev --weights-dir PREPARED_WEIGHT_DIRECTORY
```

三个大写占位值分别替换为已核实的基础镜像引用、uv 镜像引用和权重目录；组件标签不能替代共享镜像。模型环境在构建阶段按 lock 准备，不在运行时下载安装依赖。四份权重通过 BuildKit 命名上下文编入 `/models`，构建前、镜像内和启动时按同一 SHA256 清单校验；目录只含四份声明文件。权重不加入 Git，运行时不挂宿主权重。构建需要支持命名上下文的 Docker Buildx/BuildKit，正式构建会增加镜像体积。记录镜像 ID、来源 SHA、构建输入及权重摘要。

`--dry-run`只检查源码和生成构建计划，不读取大型权重或证明镜像可构建；实际构建才校验权重上下文。

已有受信任的 Linux 下载缓存时，所选 uv 工具镜像可额外提供完整的 `/python`（CPython3.11安装）和 `/uv-cache`。构建会将解释器复制到镜像内固定路径，再从锁文件离线新建模型 venv；不会复制旧 venv。缓存只在构建阶段使用，目标镜像不依赖宿主缓存。记录该工具镜像的来源、版本、内容校验和及 image ID；不修改共享基础镜像。普通仅含 `/uv` 的工具镜像继续使用联网准备路径。

同一组件镜像提供 compute、sensor、overhead、preflight、mission、check 和 parity 角色。工作站独立容器只启动任务客户端；高频执行仍在 compute。`entrypoint.sh` 只加载镜像内安装前缀，开发 overlay 构建脚本不作为交付运行入口。

状态：构建文件和启动入口已编写，本轮尚未实际 Docker build 或验证镜像，见 [验证记录](../../../docs/flowpilot-validation.md)。
