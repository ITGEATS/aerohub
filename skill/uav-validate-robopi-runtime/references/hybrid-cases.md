# 板端与混合拓扑案例

以下内容是已验证案例和诊断边界，不是所有 RoboPi 任务的默认拓扑或通过证据。

## AeroVLA onboard 降级跨容器链

2026-11-21 的范围降低拓扑不等于原跨机 DDS 目标达成：真实 NPU 推理仍留 RoboPi（HTTP 8090），AS2/framework/planner/inference-adapter 放到 PC 的 `onboard_computer` 容器，flight_board 供 PX4/clock，DDS 只走 PC 侧本机，跨机只剩 HTTP。该矩阵连续两次取得 `FLIGHT_CHAIN_PASS`：板端 NPU 真实前向、跨机 HTTP、真实时钟与传感器输入、滚动执行、取消、停止、降落和解除武装均有对应证据。

PX4 为 SITL、DDS 仅在 PC 本机，因此该结果不代表原跨机 DDS 目标或真机通过，仅限当轮版本。相关 Planner 修复归 `uav-adapt-planner`，证据位于 `log/aerovla/stage4/onboard-pass-20261121/`。

## PC 编排与板端执行

分别记录算法镜像、目标安装 manifest、DDS 网卡与 peer、domain、PX4 system ID 及进程身份。任务退出前以停止反馈、降落和解除解锁作为清理门槛；结果缺失或恢复未确认时保留执行端、时钟和仿真，交付其状态及恢复方法。

Fast DDS 显式 `initialPeersList` 用于混合部署时，应同时包含跨机 peer 和本机可达地址。仅恢复 `/clock` 参与者不能证明导航和规划节点也互相发现；地面验收要求真实算法输出而非进程存活。

混合部署共用 `/clock` 不代表 PX4 控制消息的系统时钟已同步。运行前沿用项目已声明的墙钟偏差门槛；偏差超限时保留测量并拒绝飞行。系统对时影响设备上的其他服务，必须在授权后执行并重新验证。时钟异常区分初始偏差、持续频率漂移与系统服务调时造成的跳变；短观测未见跳变不等于排除间歇故障。

## 历史全板端 SUPER 案例

历史 `robopi` 是 PC 规划、板端 AS2 执行；`robopi-all` 把 SUPER、导航、Adapter 和 Mission 也放到板端，PC 只运行 AirSim/PX4 与数据桥。不能根据参数名称判断算法位置，须以板端安装包、进程树和日志为证。该历史拓扑偏离当前工作站任务边界，仅在复现旧记录时使用。

板端 Mission 访问 AirSim RPC 时必须核对服务监听地址；编排进程退出会销毁线程内 TCP relay 时，应使用有独立进程身份的中继并在确认任务恢复后再清理。原生 AS2 起飞取消可能进入 EMERGENCY，任何应急兜底须单独记录，不能算原生 LandBehavior 通过。

SUPER 滚动轨迹首次切换控制模式时，须以当前 frame、有效 setpoint 数和首计划开始时间匹配实际参考；历史参考不能解除等待。`verify_board_handoff.py` 的通过不替代真实动力学闭环。

## SUPER 四方实例

`four-part` 将 Mission 放在工作站共享 ROS 镜像，导航/SUPER/Adapter/AS2 放在 RoboPi，PC flight_board 提供 PX4、传感器和时钟，AirSim 为宿主进程。部署启动与任务调用分开：先等待板端能力的真实地图和 ROS 就绪，工作站再经 ROS 调用。记录工作站无算法子进程、任务后板端能力仍存活以及显式回收身份。

2026-11-09 该拓扑的地面和 4 m 短程通过，但原 forest8 在滚动更新时被拒，因此不能称完整原任务通过。错误信息合并多个谓词时，须补齐提交/接收时间、generated/start、失败条件和转换耗时，不能只根据错误文字认定时钟漂移或放宽门槛。失败轮、停止确认、降落、最终 disarmed 和资源抽样均须保留。
