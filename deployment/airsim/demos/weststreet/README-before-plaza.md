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

Navigation Adapter 从历史成功轮恢复到
`log/weststreet-before-plaza/runtime/` 专用 overlay；执行器另有定位时钟排序修复：
超前时间戳先缓存，待ROS时钟追上再采用，保留原0.5秒过期门限。
启动脚本校验专用源码和二进制哈希。主线 Navigation、Behavior 源码及共享
install 未回退；请勿清理这个专用 runtime 目录。

历史证据：`log/weststreet-demo/live-tangent-pv-lead1s-recovery1s-65s-01/`。

## 启动

在仓库根目录运行：

```bash
python3 deployment/airsim/demos/weststreet/launch_before_plaza.py
```

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
