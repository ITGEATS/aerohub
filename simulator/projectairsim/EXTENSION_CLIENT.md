# ProjectAirSim 扩展 RPC Client

`extension_client.py` 是本项目对 ProjectAirSim 新增 RPC 的统一客户端对象，Python 3 标准库即可运行，
不依赖 ProjectAirSim SDK、ROS、图像库或 UE 控制台。算力板业务代码通过它调用
仿真器上的自建 HTTP JSON-RPC。当前已实现显示功能及场景行人控制；
后续本项目新增 RPC 也统一在此对象封装，不再按显示功能另建传输客户端。
原生仿真 RPC 与传感器 topic 仍由原有链路负责。

```python
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient, ExtensionError

# host 填算力板能访问的仿真器地址；端口使用部署实际值。
display = ProjectAirSimExtensionClient(host="127.0.0.1", port=9091, timeout=2.0)
try:
    display.set_title("城市低空巡检", side="left")
    display.set_title("双机协同", side="right")
    display.start_run("Drone1")
    display.show_message("任务开始", duration_seconds=3)
except ExtensionError as error:
    # 交由业务决定记录或降级；显示失败不等于飞行指令失败。
    print(error)
```

构造对象不连接网络；每次方法调用同步发一个请求。不要放进飞控高频回调。
不自动启动场景、加载机器人或注册相机；调用方发出生命周期事件，
服务端统一更新显示状态和计时。多机状态和计时必须明确传入
vehicle_id；标题和中央提示属于整个观察窗口。多方同时调用时以服务端最后处理的
写入为准，部署侧应指定标题/提示的发布方；本 client 不提供跨板仲裁。

| 方法 | 语义 |
| --- | --- |
| `set_title(text, side="left")` | 左暖金、右亮青；空字符串独立清空 |
| `show_message(text, duration_seconds=3)` | 替换中央提示；0 持续显示 |
| `clear_message()` | 清空中央提示 |
| `start_run(vehicle_id)` | 进入 RUNNING，新一轮计时和峰值清零；重复 start 保持原计时 |
| `finish_run(vehicle_id)` | 进入 FINISHED，冻结计时；重复 finish 保持结果；IDLE 下拒绝 |
| `reset_run(vehicle_id)` | 进入 IDLE，计时和峰值清零，支持重复调用 |
| `set_run_state(vehicle_id, state)` | 兼容底层接口：idle/running/finished，与计时独立 |
| `control_timer(vehicle_id, event)` | start/finish/reset；reset 同时清该机速度峰值 |
| `set_view(vehicle_id, camera_id, image_type="RGB")` | 选择观察适配层已登记的原生相机 |
| `set_global_view()` | 选择保留身份 __global__ 的质心俯视 |
| `get_state()` | 返回服务端原始状态字典，包含 title_left/title_right 等 |
| `call(method, **params)` | 底层接口，供适配层 RegisterViews 或后续扩展使用 |

以上 side、duration_seconds 均为仅限关键字的 Python 参数。通常成功写入返回服务端
确认值，get_state 返回字典；确认表示显示模型已更新，不保证某一帧已呈现。
文本长度、车辆身份、相机是否存在及计时状态转换以服务端验证为准。

## 扩展职责

新 RPC 在该对象上增加具有业务语义的方法，复用 call 的传输、请求 ID、
超时及错误处理。当前 `/aerohub/rpc` 地址保持兼容；当前服务端插件仍名为
AerohubDisplay，不因客户端职责扩展而修改已运行的协议或插件。

## 错误与重试

- `ValueError`：本地连接参数或枚举/时长错误，未发送请求。
- `ExtensionTransportError`：连接、超时、HTTP 错误；请求可能已执行。
- `ExtensionProtocolError`：JSON、响应 ID 或 JSON-RPC 结构无效。
- `ExtensionRPCError`：服务端拒绝；保留 code、message、data。
- 后三者均继承 `ExtensionError`，可统一捕获。

每个请求使用递增 ID，并检查响应匹配。不自动重试，尤其是 ShowMessage 重试会重新
计算过期时间。超时后由调用方通过 get_state 核对，再决定是否重发。
不依赖环境 HTTP 代理。对象无持久连接，无需 close，不提供隐式后台线程。

## 算力板到仿真器的网络入口

host 必须是从算力板/容器可达的仿真器地址；板端 localhost 通常并非工作站。
仅改 client host 不会改变 UE 服务端监听。当前 UE HTTPServer 的默认监听为回环，
跨机器部署应在仿真器工程/运行包的 Engine 配置中显式配置实际可达接口，例如：

```ini
[HTTPServer.Listeners]
DefaultBindAddress=192.0.2.10
```

示例地址须替换为仿真器实际网卡地址，重启运行包后生效。也可复用部署已有端口转发，
使 client 连接本地转发端口；不默认修改共享镜像、系统网络或对所有接口开放。
本任务仅验证宿主回环实际场景请求，真实 RoboPi 到工作站网络尚未验收。
配置依据：UE 5.2 `HttpServerConfig.cpp` 的 HTTPServer.Listeners/DefaultBindAddress
及 `HttpListener.cpp` 的接口地址绑定逻辑。

## 安装和现有入口

在已有仓库挂载/检出上，将仓库根目录加入 Python 模块搜索路径即可 import；不需
运行 setup_client.py 安装原生 SDK。本模块自身也可作为单独 Python 文件随组件分发。
`display_session.py` 与诊断入口 `hud_rpc.py` 共用此 client；最初的 DisplayRPC(port)
保留兼容，推荐新业务使用 ProjectAirSimExtensionClient。小窗 ImageWindows 属于图像消费进程，
不属于这个 HUD client 的远程控制范围。

协议细节见 [RPC.md](unreal/RPC.md)，固定视觉规格见
[TITLE_STYLE.md](unreal/TITLE_STYLE.md)。2026-11-15 双标题视觉已获用户认可；
client 新增 HTTP 契约、错误路径和实际宿主场景验证单独记录。

验证记录：31 项 manager/client 测试通过（含临时真实 HTTP 服务的请求契约、
响应 ID、服务端拒绝和超时路径）。新 client 已调用当前场景双标题更新、
状态查询及非法参数服务端错误，并恢复原文；证据为
`log/projectairsim/hud-style-review/client-check.json`。未声明板端网络或飞行验收。

## 生命周期主入口

新业务使用 start_run / finish_run / reset_run，分别只发一次 ControlRun RPC，
服务端在游戏线程同一回调中完成指定车辆的状态及计时变更。
采用服务端单调墙钟；开始新一轮清零峰值，完成保留本轮峰值，重置清零。
FINISHED 后 start 开始新一轮；RUNNING 时重复 start 不重置起点或峰值。
IDLE/尚无记录时 finish 拒绝且不改状态。切换观察相机不影响任何车辆的计时。
旧 set_run_state/control_timer 保留兼容，不与新生命周期入口混用；底层独立写入
可能使状态和计时不一致，ControlRun 以服务端 timer 为转换依据并重新对齐状态。
这些操作只管理显示生命周期，不会启动/取消算法或控制飞行。

生命周期实现验证：32 项测试通过，实际运行包完成 start/finish/reset、
重复调用、IDLE 下 finish 拒绝、重新开始、Global 切换及双机隔离检查；
证据为 `log/projectairsim/hud-style-review/lifecycle-check.json`。
当前静止无人机的峰值为零，未执行非零飞行速度峰值验收。

## 场景行人控制

同一个客户端显式传入 `scene_port`，HUD 保持原端口；场景请求复用请求 ID、超时和错误处理。

```python
client = ProjectAirSimExtensionClient(host="127.0.0.1", port=19891, scene_port=19892)
client.reset_actor()                 # runner 回到路线起点并暂停
state = client.get_actor_state()     # 查询位置、时间和运行状态
client.start_actor()                 # 从暂停位置继续
client.stop_actor()                  # 冻结路线时间、位置和动画
```

默认 actor_id 为 `runner`。也可调用 `control_actor("runner", "start")`。
需要 UE 加载 AerohubSceneControl 插件；端点为 `/aerohub/scene/rpc`。
未传 scene_port 时场景方法在本地拒绝，不把场景请求发给 HUD。
人物控制独立于无人机 HUD 与 AS2 飞行控制；起飞后启动人物的编排示例见
`deployment/airsim/demos/weststreet/mission.py`。
