# ProjectAirSim 仿真集成

本目录与 `simulator/airsim/` 同属主仓库的仿真接入层：外部场景、源码和工具链由资源配置定位，仓库维护管理、适配、配置和自建插件，运行产物进入 `log/`。不复制上游源码，不默认替换共享 Docker 镜像。

## 环境与依赖

AirSim 管理器和传感器适配使用宿主 Python 的 PyYAML/msgpack 与项目自建 RPC；没有要求全仓库统一安装完整 AirSim SDK。ProjectAirSim 原生协议、场景加载及图像工具通过上游 SDK 接入，使用独立 Python 环境，不将其全部依赖加入算法环境或宿主系统 Python。

| 入口 | 用途 |
| --- | --- |
| `config/resources.yaml` 与本机覆盖 | 统一定位外部安装资源 |
| `projectairsim_source` | 上游源码，包含原生 Python SDK |
| `projectairsim_python` | 项目客户端使用的专用解释器 |
| `setup_client.py` | 将配置中的上游 SDK 可编辑安装到配置中的环境 |
| `requirements-client.txt` | 仅提供注册表版本回退，不能替代锁定源码的开发安装 |

安装与资源查询见 [配置说明](../../config/README.md#python-客户端环境)。`setup_client.py --check` 只检查路径；不创建环境、不启动仿真。正式安装使用 uv，依据上游 pyproject 声明依赖。可编辑安装随上游文件变化，更新源码后须重新核对版本、依赖和实际服务版本。

2026-11-14 已在配置中的环境完成 `projectairsim 1.1.0` 可编辑安装；移除 PYTHONPATH 后验证 SDK、World、Drone 和 ImageDisplay 导入指向资源源码，uv pip check 的 43 个包依赖兼容。该结果不代表飞行或完整图像接口验收。

## 开发与运行导航

- [原生能力与接口来源](../../docs/ProjectAirSim原生显示接口核查.md)：开发前核对已有能力、来源版本、连接归属及使用限制。
- [统一 manager](../manager/README.md)：`simulator/simulator.sh` 的检查、启动、attach、状态和停止；不自动启动 PX4、AS2 或 planner。
- [传感器与 ROS bridge](SENSORS.md)：原生 bridge 及接口边界。
- [自建显示插件](unreal/README.md)：HUD、显示 HTTP、遥测及构建验证状态。
- [客户端开发清单](../../docs/场景客户端开发清单.md)：插件、UE 场景与 manager 的开发事项。

本次接入修改在当前工作树维护，未同步覆盖其他工作树或修改其 AirSim 实现。

## 算力板端扩展 RPC 控制

本项目对 ProjectAirSim 新增 RPC 统一由无 SDK 依赖的 `ProjectAirSimExtensionClient`
封装；当前方法覆盖双标题、中央提示、
按车辆生命周期与视角选择。业务通过 `start_run/finish_run/reset_run`
一次 RPC 更新状态和计时，服务端维护转换；底层独立写入接口保留兼容。方法、异常、部署地址与示例见
[EXTENSION_CLIENT.md](EXTENSION_CLIENT.md)。

## 小窗口的代码接口

小窗口使用代码中的运行时列表，不增加 config 文件。`ImageWindows` 是本项目对
原生 `ImageDisplay` 的整组生命周期封装；`ImageWindow.source` 是调用方提供的
实际数据源标识，建议使用原生图像 topic 完整路径。

```python
from simulator.projectairsim.image_windows import ImageWindow, ImageWindows

rgb = "/Sim/ScenePlannerCourse/robots/Drone1/sensors/FrontCamera/scene_camera"
depth = "/Sim/ScenePlannerCourse/robots/Drone1/sensors/FrontCamera/depth_camera"
windows = ImageWindows(window_size=(480, 270))
windows.show([ImageWindow("Front RGB", rgb), ImageWindow("Front Depth", depth)])

# 已有订阅回调：不在显示层新建连接。
# client.subscribe(rgb, lambda _, image: windows.receive(rgb, image))
# client.subscribe(depth, lambda _, image: windows.receive(depth, image))

# 运行时替换：只保留深度，按列表顺序重建整组小窗。
windows.show([ImageWindow("Depth", depth)])
windows.close()
# 可以再次 windows.show([...])；最后用 close() 或 with 语句释放显示。
```

以上数据源名称是示例，以实际已加载的 scene/vehicle/sensor 为准。show 登记后等待
有效首帧才打开窗口。它不保证所选流存在，不启用相机采集，不订阅或取消 topic，
不执行飞行或世界操作。Pair0 与 bridge 的连接归属继续由现有采集层处理。
控制线程调用 show/close；图像回调调用 receive。替换时会丢弃旧显示队列；
以实际源标识路由，避免旧源帧进入已切换内容的窗口。

自动排列使用显式屏幕大小（默认 1920×1080），不自动探测桌面；超出单排容量会报错。
同一进程使用一个管理对象负责本组 ImageDisplay，避免多个 OpenCV 显示循环争用 GUI。
这套管理层尚未接入通用 manager 或新增 HTTP 操作入口；单元验证不替代实际 GUI 验收。


## HUD v1 操作入口（2026-11-15）

当前接口与调用示例以 [HUD runtime RPC](unreal/RPC.md) 为准。
display_session.py 使用原生 SDK 展开场景、登记主视角、读取每架车辆真实遥测，
并接入 ImageWindows。主窗口 Tab 轮换实际登记视角，G 切换 Global。
hud_rpc.py 使用标准库调用独立接口，不导入 ProjectAirSim SDK。

小窗实际重复关闭测试发现原生 Qt 跨显示线程重建会卡住，因此 ImageWindows
目前为每组启动独立 spawn 显示进程：父进程只负责有界图像转发，关闭时等待并在必要时
回收本组子进程；客户端和仿真始终保留。show/receive/close 的调用方式不变。
使用该接口的 Python 程序须有 if __name__ == '__main__' 入口保护。
三轮真实 RGB/深度开关检查已验证实际窗口出现及消失，证据见开发清单。

前文“管理层尚未接入、实际 GUI 待验证”是此前阶段记录，已由本节更新；
新布局的最终视觉验收仍交由用户操作。
