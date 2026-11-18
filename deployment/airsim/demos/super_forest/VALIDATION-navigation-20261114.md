# 公共导航调用层验证（2026-11-14）

对象是 PC Docker 中新增的 `NavigationModule → NavigateToPose → SUPER Adapter` 调用层。
SUPER 原生源码、轨迹执行客户端、FollowTimedPath 和控制链未修改；本次不是重新迁移
算法，也不提升板端或其他 Planner 的验收状态。

现有镜像 `uav/apps-super-forest:migration-main-behavior` 保持不变，外部 colcon overlay
补编译公共接口、锁定的 AS2 Python API、公共 Module 和相应消费者。应用镜像 ID 为
`sha256:da9dc3e62ed355d030e277fc40c5152092827e8f9c86d75fc5969adf30068a78`；
飞控镜像 ID 为 `sha256:8c33881fde6985fe5a43a894bde075471df85064e45ac8e5de28efebcc737cd2`。
每轮 source-manifest.json、images.json、planner.yaml 和 result.json 保留实际运行输入。

## 接口与回归

- 真实 ROS Action 与 AS2 动态加载：7 项通过。执行端使用受控替身，覆盖无效输入、
  不支持朝向/速度、忙、连续任务、执行失败、取消、超时和停止确认；不代表算法测试。
- 真实 ROS 类型下 SUPER 数学/转换：8 项通过，无跳过。
- 公共规划框架回归：29 项通过。
- Demo 配置检查：4 项通过。

构建及前三组日志：`log/super-navigation/build-and-interface-tests.log`。

## 有屏 AirSim + PX4 SITL

全部通过现有 PC Demo 编排执行，正常导航经公共 Module，飞行指令经 AS2/PX4。
起降由 Demo 负责；碰撞、状态新鲜度和实际位置继续独立观测。

| 场景 | 证据目录（位于 log/super-navigation/） | 结果 |
| --- | --- | --- |
| 地面检查 | ground-final/ | 通过；未请求运动 |
| 4m 短航程 | short-public-only/ | 通过；到点误差 0.367m，目标仅经公共 Action 传入，停止、降落解除武装确认 |
| 主动取消 | short-cancel-01/ | 通过；CANCELED，停止后未恢复执行，降落解除武装确认 |
| SUPER 断流 | short-input-loss-02/ | 通过；保留 command stream expired 原因，停止、降落解除武装确认 |
| 外部接管 | short-takeover-01/ | 通过；保留 CONTROL_MODE_LOST 原因，未夺回控制，降落解除武装确认 |
| forest8 | forest8-01/ | 通过；到点误差 0.336m，航段 38.932 仿真秒，停止、降落解除武装确认 |

forest8 实测峰速 8.371m/s，沿用原任务 observe 策略，不表示严格 8m/s 限速通过。
观测到主轨迹 flag1 5021 次、备用 flag2 4 次；不能据此推广为备用切换成功率。
动态俯视相机退出时记录 545 次更新；一次 RPC 超时后恢复，不以进程存活代替更新证据。

## 保留的失败尝试

- ground-01：宿主缺少观察相机所需 msgpack；复用现有镜像依赖到本轮目录后重试。
- ground-02：源码目录挂载遮蔽镜像中的 airsim_rpc；改为执行只读仓库脚本，保留运行文件。
- ground-03：AS2 Python 接口初始化后 Demo 未及时刷新自身观测；增加一轮新观测等待，
  原新鲜度阈值保持不变。
- short-01：公共导航成功并已停止，旧 Demo 控制模式检查将主动收口误判为丢失控制；
  PC 路径改为消费公共 Action 生命周期，底层执行保护及独立实际观测保留。
- short-input-loss-01：在故障注入前触发现有 FLEET_ACK_TIMEOUT，已停止并降落解除武装；
  保留失败证据，重试未修改时间门槛，不宣称该时序波动已修复。

原始失败日志和结果均保留在对应目录。汇总为 `dev/super-navigation/evidence/summary/validation-summary.json`。
本轮提供接口封装及上述用例的验证结果，不构成长期可靠性、跨 Planner 或板端验收。
