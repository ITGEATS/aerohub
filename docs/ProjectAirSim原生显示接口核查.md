# ProjectAirSim 原生能力与显示接口开发参考

## 核查范围

- 上游锁定 commit：`92032a6c4d98724d6f23b7346b8c01be955910c3`
- 核查日期：2026-11-14；范围为场景客户端相关的观察、成像、标记、状态、环境及通信能力，不是全部飞行控制 API 清单。
- 来源仓库：`https://github.com/iamaisim/ProjectAirSim.git`，本机资源名为 `projectairsim_source`，由 `config/resources.yaml` 登记，本机差异使用 `config/resources.local.yaml`。本次实际读取目录为 `~/ProjectAirSim`，Linux 大小写敏感，不是 `~/ProjectAirsim`。
- 下表上游路径均相对 ProjectAirSim 仓库根目录，行号对应上述 commit。不是相对 Aerohub 仓库；不要把外部源码复制进本仓库。
- 核心能力表按锁定源码核查；外部工作树存在地磁和 UE 工程修改，不能将其归为纯净上游能力。本项目显示插件另列，不能视为上游自带。
- 本文的“支持”表示源码提供接口；未在本次核查中执行新一轮 UE、打包、多机或飞行验收。现有运行证据及其范围见 `simulator/projectairsim/unreal/README.md` 和 `simulator/manager/README.md`。
- 这是后续插件开发的入口参考；版本变化后应重新核查接口与语义，不依据方法名称推断兼容。

## 能力结论

| 能力 | 确切结论 | 上游源码依据 |
| --- | --- | --- |
| Viewport camera topic | 原生 viewport camera 提供 3 个订阅 topic：`desired_pose`、`aspect_ratio`、`zoom`。Scene 以 ID `viewport_camera` 创建组件，因此完整路径模式为 `/Sim/<scene>/interactive/sensors/viewport_camera/desired_pose`、`/Sim/<scene>/interactive/sensors/viewport_camera/aspect_ratio`、`/Sim/<scene>/interactive/sensors/viewport_camera/zoom`。 | `core_sim/src/scene.cpp:535-540`；`core_sim/src/viewport_camera_impl.hpp:25-36,59-84` |
| Viewport 切换示例 | Python 原生示例通过 `world.switch_streaming_view()` 将模拟器 viewport 切换到车辆相机；客户端请求方法路径为 `<scene parent topic>/SwitchStreamingView`。 | `client/python/example_user_scripts/hello_unreal_vehicle.py:144-161`；`client/python/projectairsim/src/projectairsim/world.py:1272-1283` |
| `camera_look_at_object` | 是持续跟踪，不是一次性设置。相机组件在每次 `TickComponent` 中读取目标名、查找 Actor，并按目标当前位置重新计算朝向。原生 Python 示例调用该 API 让 `DownCamera` 指向 `OrangeBall`。 | `unreal/Blocks/Plugins/ProjectAirSim/Source/ProjectAirSim/Private/Sensors/UnrealCamera.cpp:695-705`；`client/python/example_user_scripts/camera_pose.py:111-120` |
| Trace | 原生 trace 是当前被观察 Actor 的单目标轨迹：每帧只读取 `unreal_actors[idx_actor_to_view]` 的位置并追加持久线段。关闭 trace 调用 `debugFlushPersistentMarkers()`，会清除全部持久调试标记，而非只清当前目标轨迹。 | `unreal/Blocks/Plugins/ProjectAirSim/Source/ProjectAirSim/Private/UnrealScene.cpp:375-380,803-810` |
| 现成 HUD | 锁定源码没有可直接承载任务状态、规划轨迹或多机信息的通用运行时 HUD。原生可见的 UI 资产仅包括传感器调试 Widget 和天气参数 Widget，不构成上述 HUD；因此此类显示需要新增 Unreal UI/HUD 实现。 | `unreal/Blocks/Plugins/ProjectAirSim/Content/Sensors/DebugWidgetMonospace.uasset`；`unreal/Blocks/Plugins/ProjectAirSim/Content/Weather/UI/WeatherBoolWidget.uasset`；`unreal/Blocks/Plugins/ProjectAirSim/Content/Weather/UI/WeatherParamScalarWidget.uasset`；`unreal/Blocks/Plugins/ProjectAirSim/Content/Weather/UI/WeatherParamWindWidget.uasset`（二进制资产，无源码行号） |

## 限定结论

1. 原生 viewport camera topic 可直接控制观察相机位姿、宽高比和缩放，但没有“持续注视某个 Actor”的 viewport 专用 topic。
2. 传感器相机的 `camera_look_at_object` 已具备逐帧持续跟踪语义。
3. 原生 trace 的状态绑定当前 `idx_actor_to_view`，不是多目标独立轨迹集合；切换观察目标也不会形成可分别管理的多目标 trace。
4. 锁定源码没有现成通用 HUD，可复用项仅是局部调试/天气 Widget 资产。

## 其余可复用接口

以下同样按本机锁定版本源码核查，未执行新的运行验收。

| 领域 | 原生支持与边界 | 上游依据 |
| --- | --- | --- |
| 相机切换 | Tab / `World.switch_streaming_view()` 轮换 streaming-enabled 相机及车辆；viewport camera 启用时切换函数直接返回。不是按车辆和相机 ID 精确选择的接口。 | `unreal/Blocks/Plugins/ProjectAirSim/Source/ProjectAirSim/Private/UnrealScene.cpp:344-369,545-553` |
| 自由观察相机 | `client.set_interactive_feature("viewport_camera", True)` 启用；发布上述三个 topic 控制。持续俯视质心仍需调用侧计算与更新。位置在 UE 侧由 NED 米转换到 NEU 厘米。 | `client/python/projectairsim/src/projectairsim/client.py:178-191`；`core_sim/src/simulator.cpp:230`；`core_sim/src/component.hpp:98`；`unreal/Blocks/Plugins/ProjectAirSim/Source/ProjectAirSim/Private/Sensors/UnrealViewportCamera.cpp:64-118` |
| 传感器相机 | GetImages、图像 topic、相机信息 topic；RGB、深度、分割等；相对位姿、复位、FOV、焦距和视锥显示。可复用成像设施，小窗布局与交互仍由插件实现。 | `client/python/projectairsim/src/projectairsim/robot.py:51-75,461-672,710` |
| 世界标记 | 点、实线、虚线、箭头、文字、坐标轴及带名字坐标轴，支持颜色和持续时间等；持久标记通过统一 flush 清除。多机规划轨迹可复用绘制接口，独立图层和更新策略需补充。 | `client/python/projectairsim/src/projectairsim/world.py:1450-1711` |
| 状态数据 | ground-truth 运动学、位姿，actual_pose/collision_info topic，GetReadyState；就绪状态来自车辆 API，不能等同于 AS2 和任务就绪。 | `client/python/projectairsim/src/projectairsim/robot.py:104-149,218-245`；`vehicle_apis/multirotor_api/src/multirotor_api_base.cpp:1131` |
| 仿真时间 | GetSimTime（纳秒）、GetSimClockType、pause/resume/is_paused、按时间或步数推进；须按时钟模式实际验证，不代表 PX4/ROS/任务全链自动同步暂停。 | `client/python/projectairsim/src/projectairsim/world.py:86-362` |
| 场景对象 | list_actors/list_objects、读取或修改 pose/scale、spawn/destroy、材质与纹理、分割 ID 与包围盒。调用需区分观察操作与世界修改。 | `client/python/projectairsim/src/projectairsim/world.py:364-708,792-958,1286-1358,1748` |
| 环境显示 | 天气、风、光照、时间及太阳位置；F10 天气 GUI。视觉天气与物理传感器影响需分别核对。 | `docs/scene/weather_visual_effects.md:1-5`；`client/python/projectairsim/src/projectairsim/world.py:128-222,710-790,960-1032` |
| 远程画面 | 原生集成 UE Pixel Streaming，通过信令服务器和浏览器传输画面及键鼠；streaming-enabled 与原始图像 capture-enabled 独立。该版本文档注明不支持在编辑器中运行 Pixel Streaming，本项目远程部署未验收。 | `docs/sensors/camera_streaming.md` |
| 传输与发现 | 原生 RPC、topic 发布订阅、topic 信息及订阅摘要、build commit 查询，可用作插件接入基础；ROS/AS2 任务状态仍需外部提供。 | `client/python/projectairsim/src/projectairsim/client.py:106-200,255` |

本项目建议复用原生相机、真值读取和标记绘制；自建插件承担 HUD、交互、状态聚合、数据新鲜度、多机质心观察及轨迹图层管理。插件新增接口应补足上游缺失的语义，避免复制已有 RPC。

## 开发接入约束与常见误用

### 原生通信与本项目 HTTP 分开

原生 `ProjectAirSimClient` 默认 topics 端口为 8989、services 端口为 8990，可在配置中覆盖。它使用 pynng/MessagePack 通信，不是 HTTP，也不是 ROS topic。`publish()` 的 topic 消息需按原生序列化格式构造，不能将 ROS 消息或随意 JSON 直接当作兼容输入。

来源：上游 `client/python/projectairsim/src/projectairsim/client.py:32-104,142-176,255-309,352-417,496-524`；`core_sim/src/message/pose_message.cpp`、`core_sim/src/message/float_message.cpp`。

原生 topic socket 是 `pynng.Pair0`，不能当作支持任意多个独立订阅客户端的广播总线。ROS bridge 已占用该连接时，新增显示进程不能直接照搬 `client.connect()`。现有速度转发使用 `connect_services()`，仅建立 Req0 服务连接。未来 viewport camera topic 发布也必须先明确通道归属、复用或转发方式，再验证与 bridge 共存；有原生 topic 不等于当前拓扑可以直接新增客户端。

来源：上游 `client/python/projectairsim/src/projectairsim/client.py:47-104`；本项目 `simulator/projectairsim/display_telemetry.py`、`simulator/manager/README.md`。

### 观察连接不能意外改变世界

`World(client, scene_config_name=...)` 会加载指定场景；即使未给场景名，构造函数也会导入一个 `null_trajectory`。其默认 scene parent topic 为 `/Sim/SceneBasicDrone`，不能用它推断已运行场景 ID。观察已有会话时应读取 manager 输出的 resolved scene/runtime，明确实际 scene ID 和调用副作用，不能为读取速度随意新建并加载 World。

来源：上游 `client/python/projectairsim/src/projectairsim/world.py:38-76`；本项目 `simulator/projectairsim/display_telemetry.py`、`simulator/manager/projectairsim_scene.py`。

主视口 topic 的 `<scene>` 是原生场景 ID，不是地图文件名或车辆 namespace。启用接口只创建控制入口，不自动产生鼠标漫游、平滑跟随或机群覆盖策略。topic wire payload、旋转约定和启用后的发现时序需要实现时按源码验证。

### 就绪、时间与轨迹边界

- `Robot.get_ready_state()` 最终由车辆 API 提供；例如 MAVLink 实现返回内部 `is_ready_` 及消息，而 `CanArm()` 还检查 GPS lock。不能将其直接标为“可以解锁”或“AS2/任务就绪”。来源：上游 `vehicle_apis/multirotor_api/src/mavlink_api.cpp:2469-2478`。
- 仿真时间、墙钟时间、任务执行时间需要分别命名；原生 pause/step 不证明外部 PX4、ROS、AS2 和任务共同暂停。来源：上游 `client/python/projectairsim/src/projectairsim/world.py:86-362`。
- 调试绘制不是计划执行接口。绘制规划轨迹不表示执行器已接收或开始执行，原生 `import_*_trajectory` 也不能未经语义审查就当作本项目 FollowTimedPath 执行链。
- `flush_persistent_markers()` 是全局清理，原生 trace 关闭也调用它。需要按车辆/任务隔离的图层不能靠全局 flush 实现。来源：上游 `client/python/projectairsim/src/projectairsim/world.py:1450-1461`、`UnrealScene.cpp:375-380`（完整 UE 路径见前表）。

## 本项目现有扩展及职责

| 能力 | 当前实现及限制 | 本项目来源 |
| --- | --- | --- |
| 独立 UE 显示插件 | `AerohubDisplay` 提供顶部计时条和速度仪表；当前按单个 vehicle 配置，非通用多机状态界面 | `simulator/projectairsim/unreal/AerohubDisplay/Source/AerohubDisplay/Private/AerohubDisplay.cpp`、对应 Public 头文件 |
| 显示 HTTP | 默认 8991：GET `/aerohub/status`，POST `/aerohub/telemetry` 和 `/aerohub/mission/{start,finish,reset}`；端口可配置；mission 路由实际只修改显示计时，不调用任务或飞控 | 同上；`simulator/manager/display.py` |
| 速度数据 | 通过原生 GetGroundTruthKinematics 读取 `twist.linear` 模长，转发到 HUD；属于仿真真值，不是 PX4/AS2 状态估计 | `simulator/projectairsim/display_telemetry.py` |
| 配置与生命周期 | 场景 YAML `display` 字段，manager 写入显示配置并管理遥测辅助进程；attach 与独立启动的配置路径有区别 | `simulator/manager/display.py`、`simulator/manager/manager.py`；`config/README.md` |
| 离线视觉预览 | Pillow 生成审查图片；可检查布局和异常状态，但不验证 UE 字体、运行时更新、原生连接或飞行 | `simulator/projectairsim/display_preview/render.py` 及 README |
| v6 源码迭代 | TIMER/WALL、SIM GT、遥测状态及失效指针处理已改源码和预览；尚未编译或运行验收 | `simulator/projectairsim/unreal/README.md` 的 2026-11-14 迭代说明 |

接口兼容注意：目前 HTTP status 的 `mission` 字段仍是显示计时状态；已有 start 请求在 running 状态返回 HTTP 400，不能依赖重复 start 为成功的幂等请求。后续扩展应明确字段和版本兼容，并同步维护 README 与实际实现。

## 后续开发核对顺序

1. 从 `config/resources.yaml` 与本机覆盖解析上游位置，核对 Git HEAD、工作树差异及实际运行服务的 build commit（`client.get_build_commit_hash()`）。Python client、原生库、UE 包和 bridge 版本分别核对。
2. 在本文能力表找到原生接口，阅读锁定版本 Python 包装、core 注册和 UE 实现；区分原生方法、示例、局部 UI 与本项目扩展。
3. 明确新接口的数据源、车辆/场景身份、时间与坐标、更新/过期语义及副作用；确认连接归属和 bridge 共存方案。
4. 用原生能力补齐插件交互，避免重复维护成像、场景 RPC 或第二套世界模型。需要额外状态时由外部适配层提供，遵守既定四方职责。
5. 分别验证接口、真实 UE 显示和目标打包程序。多机、断连、重载及数据过期单列证据，更新本文和正式插件 README；未验证项保持待验证。

## 来源复查方法

下面的命令在所选 ProjectAirSim 源码目录执行，仅作只读核查；路径通过资源配置解析，不要求其他开发者采用本机目录。

```bash
git remote get-url origin
git rev-parse HEAD
git status --short
git show 92032a6c4d98724d6f23b7346b8c01be955910c3:core_sim/src/viewport_camera_impl.hpp
git show 92032a6c4d98724d6f23b7346b8c01be955910c3:client/python/projectairsim/src/projectairsim/world.py
```

公开来源为上述 origin 仓库的锁定 commit；网页默认分支可能已更新。源码行号用于定位，发生漂移时按方法名检索并核对实现。正式设计与开发进度见 [场景客户端开发清单](场景客户端开发清单.md)。

## 当前键盘绑定（2026-11-14 源码与配置核查）

| 按键 | 行为 | 条件与来源 |
| --- | --- | --- |
| Tab | 轮换 streaming-enabled 相机和车辆 | 非 VR 时注册；viewport camera 启用时函数直接返回。上游 `unreal/Blocks/Plugins/ProjectAirSim/Source/ProjectAirSim/Private/UnrealScene.cpp:344-369,545-553` |
| T | 开关当前观察对象的运动轨迹 | 关闭会清除全部持久调试标记。上游同文件 `375-380,555-560,803-810` |
| F10 | 打开天气设置 GUI | 上游 `docs/scene/weather_visual_effects.md:5` 的文档声明；本次未解析 Blueprint 键事件或实际按键验收 |
| F11 | 全屏切换 | UE 工程 `unreal/Blocks/Config/DefaultInput.ini` 的 `bF11TogglesFullscreen=True`；实际表现取决于窗口与运行模式 |
| ~ | 控制台入口配置 | 同文件 `ConsoleKeys=Tilde`；不保证 Shipping 包开放控制台 |
| Esc | 浏览器串流释放键鼠捕获 | Pixel Streaming 网页行为，上游 `docs/sensors/camera_streaming.md:84`；不能当作本地场景退出快捷键 |

本项目 `AerohubDisplay` 当前源码没有额外按键绑定；计时 start/finish/reset 通过 HTTP/CLI 操作。此次未确认通用 WASD/鼠标自由漫游或键盘飞控操作，不能将 UE 编辑器操作习惯视为打包客户端已支持的功能。使用按键前需让对应游戏窗口或串流画面获得输入焦点。

## Tab 候选与小画面（补充核查）

Tab 更换的是整个游戏主视口，不是 HUD 小画面。候选为“车辆 → 相机 → 图像 capture 类型”三级集合：相机须被创建，且 capture-settings 中至少一项 `streaming-enabled: true`；同一相机可有多个 streaming capture。内部先轮换当前相机的 streaming capture，回绕后切到下一个相机，车辆全部相机回绕后切下一车辆。图像类型按 ImageType 索引收集；相机/车辆顺序应以实际创建数组为准，不把摄像头名称的字母顺序当成契约。没有 streaming 相机的车辆仍有 actor 视点回退，不会自动获得可用传感器画面。

来源：上游 `unreal/Blocks/Plugins/ProjectAirSim/Source/ProjectAirSim/Private/Robot/UnrealRobot.cpp:558-598,1612-1620`；`Private/Sensors/UnrealCamera.cpp:310-321,617-626`；`Private/UnrealScene.cpp:344-369`（后两路径相对同一 ProjectAirSim Source 目录）。切换还会调用 SetViewportResolution 匹配当前 render target，见 `UnrealRobot.cpp:629`。

`capture-enabled` 与 `streaming-enabled` 独立：前者用于获取/发布原始图像，后者决定主视口切换候选。启用 RGB/深度采集并不自动加入 Tab。HUD `display.camera` 也不配置 Tab 候选。

上游明确移除了 AirSim 配置中的 `SubWindows`，改为客户端 `ImageDisplay` API。该 API 用 OpenCV `namedWindow`/`imshow` 创建桌面弹窗，可同时显示多路图像，默认提供三个子窗口位置；不等于嵌入 UE 主画面的画中画。原生 `hello_drone.py` 示例订阅 Chase、DownCamera RGB 和 Depth 后交给 ImageDisplay。复用时须同时考虑图像 capture 开关及前述 Pair0 与 bridge 的连接归属。

来源：上游 `docs/transition_from_airsim.md:80-82`；`client/python/projectairsim/src/projectairsim/image_utils.py:21-59,158,223`；`client/python/example_user_scripts/hello_drone.py:20-59`。

本项目当前 v5/v6 HUD 不绘制内嵌相机小画面。若需要 UE 内画中画，可复用相机 render target，在自建插件里实现显示布局及交互；不应把 OpenCV 弹窗称为已经具备的 UE 内嵌小画面。

2026-11-14 本机 `log/simulator/scene/resolved_scene.json` 的配置快照为 Drone1：Chase 的 RGB streaming=true、capture=false；DownCamera 的 RGB 与 Depth capture=true、streaming=false。因此该快照只有 Chase RGB 一个 Tab 候选，按 Tab 可能看不到变化。该文件为被忽略的运行产物，只说明本次读取的配置，不证明当前服务已加载它；换场景须重新枚举。

### ImageDisplay.add_image 的运行时语义

`add_image(image_name, subwin_idx, resize_x, resize_y)` 登记窗口元数据和队列，不立即创建 OS 窗口。`start()` 启动显示循环，`receive()` 将有效图像入队，循环在读到首帧后调用 `cv2.namedWindow()` / `moveWindow()`。因此可以在显示运行期间增加窗口登记，但原生实现没有完整的动态窗口生命周期管理：重复同名 add_image 会覆盖队列，跨线程注册没有明确同步，不能直接作为可靠的运行时开关接口。

`add_image` 不建立图像订阅，不修改相机 capture 配置；调用侧仍需选择数据源并将图像传给 receive。原生没有对称的单窗口 remove/enable/disable 公共方法；stop 停止整个显示循环并销毁窗口。Esc 同样停止 ImageDisplay 全部窗口，不是逐窗口开关。

来源：上游 `client/python/projectairsim/src/projectairsim/image_utils.py:70-102,120-175`。本项目小窗口首版配置采用自动排列、按列表顺序，不提供 position 字段；运行时开关尚待实现。

### ImageDisplay 再核对结论（2026-11-14）

- 自动排列仅指初始化时预计算一排等间距槽位：`gap=(screen_res_x-num_subwin*subwin_width)/(num_subwin+1)`。不会自动探测桌面尺寸、换行、缩小窗口或因窗口增删重新排列；窗口过多会出现负间距。来源 `image_utils.py:24-56`。
- `subwin_idx=None` 时不调用 moveWindow，位置由窗口系统决定。按 config 顺序排列需要本项目按启用项顺序分配 `subwin_idx=0,1,...`，不是原生读取 YAML。来源 `image_utils.py:87-97,158-164`。
- 构造器的 subwin_width/subwin_height 用于槽位计算，不自动设置每幅图的显示大小。需要对 add_image 同时传 resize_x/resize_y 才会显式缩放；两者必须同时非空。来源 `image_utils.py:50-56,87-97,216-219`。
- 运行中 add_image 从实现上可被显示循环在后续轮次发现，但官方示例只展示先登记再 start，没有可靠动态注册的同步保障；不能将其宣称为原生正式支持的运行时窗口管理。来源 `image_utils.py:87-97,149-171`；`example_user_scripts/hello_drone.py:35-58`。
- destroy_windows 是整组销毁且保留登记；若循环继续收帧，会重新创建窗口，不能用作持久关闭。窗口关闭按钮没有被代码显式检测为禁用。来源 `image_utils.py:139-144,149-177`。
- Esc 把 running 置 false 并关闭整组窗口，但显示线程 runner 引用仍保留；start 仅在 runner 为假时创建线程，因此不能保证 Esc 后直接 start 能恢复。stop 在 runner 已退出时也不清引用。实现运行时启停应封装和验证这一边界。来源 `image_utils.py:72-85,173-177`。

因此此前“运行中 add_image 即可新增窗口”的表述应限定为实现层面可以登记、没有完整生命周期保障；首版采用启动前按配置统一登记，动态开关另行实现。

### 运行后新增小窗口实测（2026-11-14）

已使用资源环境可编辑安装的 SDK 和运行中走廊实例的真实 FrontCamera RGB（1280×720 BGR）执行两组实验：先登记、start 后 receive；以及先 start、随后 add_image 和 receive。两组均在首帧前无窗口，首帧后 created=true 且显示线程存活，无需重启客户端或仿真。临时测试窗口完成后关闭，原演示保持运行。

证据：`log/projectairsim/imagedisplay-start-check/result.json`、`runtime-add-result.json` 及对应脚本、日志。结论限定为当前环境实际新增成功，不等于任意并发注册安全。切换来源可以沿用窗口名送入另一数据源，但仍需处理订阅切换、积压帧和新鲜度；单窗口关闭/恢复仍需封装，不能用 add_image 代替。

### 早期小窗配置重载建议（已被代码运行时方案替代）

配置采用列表和 enabled、vehicle、camera、image 等字段；自动排列，按启用项的列表顺序编号，不提供逐窗口 position。修改后整组关闭并重建：停止旧显示、解除本组显示订阅、释放旧对象，读取新配置并新建 ImageDisplay，再登记、接入图像和启动。客户端与仿真器保持运行，不加载新世界或重启车辆，不影响共享数据消费者。

显示重载只选择已经可采集的数据源，不自动启用场景中关闭的相机采集。首版无需逐窗口热切换；重载 API 和正式配置入口尚待实现，不能将方案确认等同于已完成。重复重载、全禁用、首帧缺失及退出恢复仍需验证。决策主记录见 [开发清单](场景客户端开发清单.md)。

### 当前小窗管理方案：代码运行时列表

用户最终选择不为小窗新增 config。本项目 `simulator/projectairsim/image_windows.py`
提供 ImageWindow(name, source) 和 ImageWindows.show/receive/close。show 按代码传入列表
整组替换并自动排列，close 后可重新 show；每轮新建原生 ImageDisplay，不重启客户端
或仿真，不管理连接和订阅。此前有关小窗 YAML、enabled 字段、文件重载的建议不再实施。
原有仿真与 HUD 配置保留各自职责。接口例子和限制见
[接入说明](../simulator/projectairsim/README.md#小窗口的代码接口)，最终决策见开发清单。

当前管理层仅完成生命周期和路由的单元验证，完整真实 GUI 反复启停仍待验收；此前
原生 start/add_image/receive 的真实图像测试不能替代这层管理逻辑的验收。


## HUD v1 实施后边界（2026-11-15）

当前项目观察层通过原生场景 SDK 展开得到 streaming capture 列表，
映射锁定版本 UE 公开场景组件的名称、位姿和后处理；插件自建 CameraActor
作为实际主视口，并接管 Tab/G。因此本项目 HUD 的身份不推测原生私有 streaming index，
也未修改上游源码。原生 T 仍绑定上游 index，不保证跟随这套观察层。
独立 HTTP JSON-RPC 接口与用户直接调用方法见
[RPC.md](../simulator/projectairsim/unreal/RPC.md)。

Global 使用 __global__ 保留观察身份、界面显示 Global；实际相机在车辆实时质心
上方垂直俯视。地面受控瞬移后的独立 GetObjectPose 检查验证相机随质心变化。
小窗 runtime 管理现采用每组独立显示子进程；这是实际重复启停测试后为隔离 Qt
线程亲和问题所作的本项目适配，原生 ImageDisplay 的线程逻辑没有修改。
