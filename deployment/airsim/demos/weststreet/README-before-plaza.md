# 柱区前编排副本与录屏启动入口

当前状态：**一次真实65秒追踪及离线阶段验收通过**。证据：
`log/weststreet-before-plaza/mission-recovery-clockorder-65s-01/`。
这不代表完整循环、全面碰撞或可靠性验收通过。

`demo_before_plaza.py` 以历史成功轮为基线，保留柱区前路线和编排，并新增
Tracking 失效后最多2秒的 Mission 等待。等待期间维持最后导航点和朝向，
不使用无效目标位置/速度；收到新的有效且新鲜观测才恢复，超时仍失败。
Tracking 原1秒预测期、YOLO阈值均未修改。原始副本保存在
`log/weststreet-before-plaza/restoration/demo_before_plaza.original.py`，
原始及当前哈希记录于 `before_plaza_checkpoint.json`。

Navigation Adapter 从历史成功轮恢复到专用 overlay（运行配置中的
`acceptance_workspace`）。执行器另有定位时钟排序修复：
超前时间戳先缓存，待ROS时钟追上再采用，保留原0.5秒过期门限。
启动脚本校验专用源码和二进制哈希。保留完整专用工作区，包括
`restored-source.json`、`src/` 和 `install/`。

历史证据：`log/weststreet-demo/live-tangent-pv-lead1s-recovery1s-65s-01/`。

## 启动

将 [runtime.example.json](runtime.example.json) 复制为同目录下的 `runtime.local.json`，
再填入本工作树使用的路径。启动脚本默认读取它，Git 忽略该本机文件。
`--runtime-config` 可选择同一 Demo 的其他部署配置。相对路径以配置文件为基准，支持 `~`。
配置引用长期工作区和已安装的 overlay。旧工作区曾放在 `log/` 时，先在固定位置重新构建
或验证迁移后的安装目录，保留原工作区直到新路径通过检查。基础 profile 及其引用的场景、
机器人配置、SDK、bridge overlay 也要指向可长期使用的位置。任务输出仍可写入 `log/`。

```bash
python3 deployment/airsim/demos/weststreet/launch_before_plaza.py --check-runtime
python3 deployment/airsim/demos/weststreet/launch_before_plaza.py
```

`--check-runtime` 核对部署输入、验收快照哈希和场景时钟，不启动模拟器或申请飞行。
直接使用 `run_tool_demo.py` 时接受同一配置；只在需要历史验收快照时显式传
`--component-overlay`。旧的 `run.py --perception-only` 使用 `--query-workspace`
指定检测 overlay。

启动入口默认加载 `WestStreetClosedLoop`。路线保留原起跑点及前段街区路线，末段沿错开的回程跑道转回起跑方向。地图和动画作为独立资产保存。用 `--legacy-route` 可加载原 65 秒飞行验收地图。下文的历史飞行结果均对应原路线。

UE 观察窗口默认限帧 30 fps。使用 `--fps 60` 切换到 60 fps。此参数控制 UE 渲染上限，GNOME 录屏帧率由录屏工具设置。

配置固定为原起点、约2米起飞、持续 p/v 跟随（不设录屏时长，Ctrl+C安全停止）、前看1秒、Tracking预测窗口1秒、Mission失效等待最多2秒、4米转弯半径、弯道2.75m/s、直线5m/s、YOLO阈值0.35。仿真时钟保持3ms/3ms。录像由使用者自行启动；脚本不录屏。

脚本检查源码哈希、端口和现有 WestStreet 容器占用，拒绝替换其他运行实例。它加载已有部署，打开 UE 与官方 YOLO 小窗；UE视图就绪后默认预留15秒，再继续启动任务。需要更长准备时间时：

```bash
python3 deployment/airsim/demos/weststreet/launch_before_plaza.py --viewer-wait 30
```

点击 UE 窗口后，`Tab` 切无人机视角，`G` 切 Global。录制起飞、两次转弯、继续跟随和最终降落；未跑满、异常退出或缺少物理解除武装证据均保留失败结果。

## 停止

原终端 `Ctrl+C` 请求任务取消、降落和清理，等待终端结束。也可从另一终端运行：

```bash
python3 deployment/airsim/demos/weststreet/launch_before_plaza.py --stop
```

停止命令只匹配本入口记录的进程及证据目录。不要直接杀 UE 或 PX4。

脚本使用现有 Docker 镜像临时提高图像 DDS 所需的宿主接收缓冲上限，并在退出时恢复原值；因此运行用户需要已有 Docker 权限。无需重建镜像或重新烘焙场景。每轮生成新证据目录，最终判据查看 `mission/result.json` 与 `cleanup.json`。

## 诊断与本轮结果

历史源码对照发现 Navigation 的停止期间更新逻辑有变化，但执行器原定位
新鲜度检查没有变化，不能将原来的 `STALE_LOCALIZATION` 直接归因于
Navigation 版本。恢复历史 Adapter 后，也出现过观测失效和定位时序误报。

实际诊断捕获执行检查时钟与定位时间戳相差3ms的竞态。因此只在专用执行器
加入有界的因果定位缓存，避免新定位暂时超前时覆盖仍有效的旧定位；真正过期、
时钟回退及非法定位仍会失败。历史Adapter 10项测试、执行器3项C++测试、
Mission恢复6项测试通过。

本轮 `mission-recovery-clockorder-65s-01`：

- 跑满65秒；一次Tracking失效后等待0.292秒，新观测到达并接续。
- 因果定位缓存处理3次超前3ms的定位；未记录定位过期中止。
- 追踪阶段3242条实际控制参考，最大水平位移161.48米。
- 正常降落、物理解除武装、服务清理，宿主DDS缓冲上限恢复原值。
- `evaluation.json` 的6项阶段门均通过；不覆盖完整循环、全面碰撞和可靠性。

此前失败轮仍保留在同级目录；源对照、构建日志在
`log/weststreet-before-plaza/restoration/`。本入口依赖当前机器已构建的专用
runtime，不是可脱离部署环境分发的独立包。

录屏入口现默认 `--duration 0`，Mission及宿主不按时长退出。现有Tool接口只接受有限正超时，因此专用副本传入1e12秒预算，避免默认300秒退出。观测丢失、控制异常、磁盘不足等退出条件仍有效。该持续运行配置已完成静态检查，以上65秒实飞是取消时长限制前的证据。

## 场景时钟与红字修复（2026-11-18）

人物改为跟随 ProjectAirSim 仿真时钟；原3ms/3ms和PX4 lockstep配置保留。
GNOME录屏下旧版在固定3秒仿真窗口后，人物序列已走到3.774秒（含原起步渐变），
新版在相同窗口正确走到2.5秒；不同仿真速率下的实际人物位置一致。
新版录屏场景回归15项通过，包含暂停、恢复、停止、复位和重复启动。

当前工程纹理池为4096MiB（24GiB GPU配置），保持streaming开启；
Landscape物理材质已构建并显式保存。UE5.2 `-game`漏初始化临时保存哈希导致的
误报由场景插件按当前材质哈希校验后补齐，不隐藏原生过期告警。
工程插件已增量编译安装，仍用上述入口启动。构建与回归步骤见
[场景运行维护](../../../../simulator/projectairsim/weststreet/RUNTIME.md)。
GNOME持续录屏下，另执行一次65秒墙钟追踪：Mission成功，实际追踪水平位移
151.10m，录包包含3239条执行参考、220个不同采集目标更新；正常降落、物理解除
武装及服务清理均确认，既有离线6项阶段门全部通过。录屏成片132.19秒，覆盖本轮
启动、任务和收尾；宿主DDS缓冲上限恢复到原值。
证据位于`log/weststreet-clock-fix/`，包括`flight-check.json`和
`flight-gnome-65s/evaluation.json`。仅为本次有界阶段通过，完整环线、全面碰撞、
可靠性及完整故障矩阵仍未验收。
