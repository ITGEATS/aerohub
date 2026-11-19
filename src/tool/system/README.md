# System Tool

任务侧通过 System Tool 调用系统能力。当前提供起飞、普通降落和只读状态查询；
后续按能力扩展，不让任务脚本承担 Behavior、飞控服务或运动参考的调用。

```text
工作站 demo.py → SystemClient / drone.system → ROS System Tool（算力侧）
  → TakeoffBehavior / LandBehavior、平台准备服务 → AS2 → PX4
```

起降使用同一个有限时长 Action，状态通过独立服务读取。查询不会占用运动资源。
解锁和 Offboard 是起飞内部准备步骤，当前没有任务层独立解锁、停桨或应急接口。
部署和验证状态见 [部署说明](../../../deployment/system/README.md)。

| 操作 | 调用 | 语义 |
| --- | --- | --- |
| 起飞 | `takeoff(height=3., speed=.5, timeout=60.)` | 相对当前高度，内部解锁/Offboard，调用原生起飞，确认高度、FLYING和低速稳定 |
| 降落 | `land(speed=.5, timeout=90.)` | 原生降落；确认LANDED/DISARMED，再确认未解锁和低速稳定 |
| 查询 | `get_state()` | 返回状态时间、fresh、平台状态、全局 pose/twist、本地 navigation_pose、运动占用者与当前阶段；保留原始 frame/stamp，全局 fresh 与 navigation_pose_fresh 分别判定 |

返回值是异步 ROS Future，须有独立执行器处理回调。起降 Future 返回 Tool Action
handle，必须检查 accepted，再等待结果；发送请求不等于完成。结果包括 success、
code、reason、stop_confirmed、landed、disarmed。拒绝请求没有 Action Result，可查询
状态诊断占用。Action 反馈提供阶段、相对高度和解锁状态。

支持 `drone.load_module('system')` 后调用 `drone.system.takeoff/land/get_state`，
也支持普通 ROS Node 构建 `SystemClient(node)`。任务 Demo 使用后一种形式。

超时使用单调墙钟，包括状态和依赖就绪等待；清理另有默认10秒预算。
正常起飞要求连接正常、未解锁且在地面，不接管已解锁平台。降落不抢占导航；
调用方须先取消 Navigation 并确认其 stop_confirmed，再调用 land。
参数门：起飞0.5–20m、速度0.1–2m/s、任务预算5–300s，均须有限值。
默认状态最大年龄0.75秒，低速门0.25m/s，稳定1秒；这些是本接口检查门，
不是所有场景下的飞行动力学保证。

取消/失败时停止本次原生 Behavior 并等待终态、确认低速稳定；若仍在地面且已解锁，
回滚解锁。不会在空中直接停桨。无法确认原生提交结果、停止、状态或时钟回退时，
返回失败并保留运动占用，禁止后续任务接管。取消起飞不代表落地，调用方必须查看
landed/disarmed并决定后续普通降落。应急保护、人工接管与权限隔离不由本版实现。

## 运动互斥边界

公共纯Python包 `uav_motion_resource` 提供每车辆命名空间的 flock 和占用记录。
System和SUPER的 `NavigateToPose` 在接收任务时原子获取，确认停止后释放。
进程退出或停止未确认时记录保留，不按TTL清除。部署必须让两者使用相同
`AEROHUB_MOTION_LOCK_DIR`，跨算力容器须挂载同一宿主目录；记录还应跨容器重建保留。
默认目录为 `/tmp/aerohub-motion`，仅适合同一容器进程互斥，不保证容器重建后的恢复。

当前互斥仅覆盖这两个已接入的托管入口；YOPO、旧enable入口、原生Behavior及平台
直连客户端不在覆盖范围。该部署不得同时启动这些旁路作为运动控制方。
本版不宣称实现仓库全部Planner的全局资源仲裁或应急优先级。

残留记录不能通过“进程不在了”判定可删。运维恢复须独立确认旧执行已终止、平台
已安全停止，再清除对应车辆的记录；当前没有自动恢复/强制抢锁API。
