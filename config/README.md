# 用户配置入口

共享仿真配置从 `config/simulator/` 选择，统一启动入口默认读取 `default.yaml`。
默认配置和普通场景 YAML 可以纳入 Git；只有 `*.local.yaml` 本机配置被忽略。
其他场景配置需显式选择，不自动覆盖默认项。

```text
config/simulator/
├── default.yaml         默认配置，纳入 Git
├── blocks.local.yaml    本机场景配置，不入 Git
└── forest.yaml          共享场景配置，可纳入 Git
```

安装资源统一登记在 `config/resources.yaml`，包括 UE4、UE5、AirSim 和 ProjectAirSim。
本机路径差异放在已忽略的 `config/resources.local.yaml`，同名资源覆盖共享值。
资源按以下顺序加载，后者覆盖同名项：共享 `config/resources.yaml` →
用户级 `~/.config/aerohub/resources.yaml` → 旧 AirSim 本机
`simulator/airsim/config/resources.local.yaml` → 新 `config/resources.local.yaml`。
共享默认值不会覆盖已有本机配置。
场景 YAML 通过 `source_resource`、`binary_resource`、`python_resource` 和
`ros_setup_resource` 引用资源名；原先直接填写路径的方式仍可使用。
UE 资源目前用于登记和查询，尚未提供自动启动编辑器的命令。

## 配置归属

配置跟随负责读取和维护它的模块。跨模块使用同一个值时，由其归属模块声明一次，其他入口引用或接收解析结果。

| 放哪里 | 放什么 | 现有例子 |
| --- | --- | --- |
| 根目录 `config/` | 仓库共享的资源表、可选的仿真运行组合、推理服务绑定 | [共享资源表](resources.yaml)、[默认仿真组合](simulator/default.yaml)、[Cloud 推理样例](inference.cloud.example.yaml) |
| `deployment/` | Deploy 怎么把进程跑起来，包括拓扑、镜像选择、部署端口、任务参数。专用于某个 Demo 的车辆、传感器和起点也随 Demo。 | [Swarm 板端验证拓扑](../deployment/swarm/configs/swarm_open_field.yaml)、[Swarm Demo 编组](../deployment/airsim/demos/swarm_open_field/fleet.yaml)、[YOPO Demo settings](../deployment/airsim/demos/yopo_single/settings.json) |
| `simulator/` | 仿真器和场景自身维护的配置，包括地图生成参数、可复用的场景 profile 和基础 settings | [CAT-ORA Open Field 基础 profile](../simulator/airsim/scenarios/catora_open_field/profile.json)、[森林基础 settings](../simulator/airsim/scenarios/super_forest/settings.json)。[Course 地图生成参数](simulator/blocks_course.yaml) 目前仍由 `config/simulator/` 提供，供 [Course 构建入口](../simulator/projectairsim/course/README.md) 使用。 |
| 组件目录 | 组件自己解析的算法参数和依赖配置。Deploy 按需选择该组件提供的配置。 | [SPF Demo 的 Planner 参数覆盖](../deployment/airsim/demos/spf_language_nav/planner.yaml) |

例如，同一个 AirSim 场景包可以在根资源表登记一次，基础 settings 放在场景目录。各 Demo 的 profile 选择该包，指定自己的 settings 或引用基础 settings。启动器根据本轮部署生成最终配置，并把相同的端口和车辆名传给相关进程。

机器差异通过本机覆盖提供。RoboPi 工具当前读取 [`.env` 样例](../.env.example) 所示的变量，WestStreet 工作区路径有 [Demo 本机样例](../deployment/airsim/demos/weststreet/runtime.example.json)。密钥可随本机配置保存，或通过环境变量提供。实现内部固定的值留在代码；自动选出的 ROS domain、镜像 ID 和最终 settings 留在本次运行目录。

### 本机覆盖与密钥

- 安装路径用 `config/resources.local.yaml` 覆盖，可复制 [本机资源样例](resources.local.example.yaml) 后按实际安装修改。资源加载顺序见本文开头。
- 组件专用本机差异随组件保存，使用入口显式支持的覆盖路径。`*.local.yaml`、`*.local.yml`、`*.local.json` 和 `*.local.toml` 已统一忽略；共享样例使用 `*.example.*`。
- 密钥可直接填写在已忽略的 local 配置中，也可使用组件支持的环境变量或 `.env` 入口。Cloud 配置支持 `api_key` 和 `api_key_env`，样例见 [Cloud 配置](inference.cloud.example.yaml)。共享 example 使用占位值。ModelScope 使用 CLI 本机登录凭据也可。

`.env` 是本机 Shell 赋值文件。RoboPi Shell 入口通过 `tools/robopi/load_env.sh` 自动加载，其他 Python、ModelScope 及直接启动的命令只继承进程环境。需要这些入口使用 `.env` 时，在启动它们的 Shell 中加载。

```bash
set -a
source .env
set +a
```

配置覆盖是否自动发现由对应入口决定，文件命名只约定归属与 Git 忽略。更新共享配置或准备脚本时保留已有本机文件。

### 启动配置实例

| 入口 | 默认值与运行时来源 |
| --- | --- |
| WestStreet | Demo 的 [端口配置](../deployment/airsim/demos/weststreet/deployment.json) 提供默认值。复用会话时从原会话的 profile 与 scene-control 配置读取仿真端口。 |
| Swarm Open Field PC | 本轮 AirSim settings 提供仿真 RPC 端口，启动器传给时钟、传感器和工作站。 |
| Swarm Open Field RoboPi | [混合部署配置](../deployment/swarm/configs/swarm_open_field.yaml) 声明 AirSim RPC 与板端 relay 端口。运行目录记录实际端口。 |

## 开发资源导航

`resources.yaml` 同时登记运行资源和开发资源。ProjectAirSim 开发从
`projectairsim_source` 进入，读取其原生文档、Python API、core 和 UE 实现，
再选择复用或扩展。本机实际位置由资源解析器确定，不以文档中的示例路径为准。

| 资源名 | 用途 | 开发时查阅 |
| --- | --- | --- |
| `projectairsim_source` | 上游源码根目录 | `docs/`、`client/python/projectairsim/src/projectairsim/`、`core_sim/`、`unreal/Blocks/Plugins/ProjectAirSim/Source/` |
| `projectairsim_project` | UE 工程文件 | 场景、项目插件、编译和编辑器入口 |
| `projectairsim_blocks` | 已打包可执行文件 | 独立启动与目标包验收；源码改动需重新构建才会进入包 |
| `projectairsim_python` | Python client 环境解释器 | 原生客户端依赖和调用验证 |
| `projectairsim_ros2_setup` | ROS bridge overlay | 已构建桥接器的运行环境；不代表其源码位置 |
| `ue5_root` / `ue5_editor` | UE 工具链与编辑器 | 插件编译、场景编辑和打包 |

从仓库根目录查询本机实际源码路径，自动应用 `resources.local.yaml` 覆盖：

```bash
python3 -c 'from simulator.manager.resources import resolve_resource; print(resolve_resource("projectairsim_source"))'
```

打开该目录前核对其存在性；该查询只解析配置，不下载、创建或启动资源。
`config/simulator/*.yaml` 的 `projectairsim.source_resource` 可以选择另一资源名，
应以所选场景配置为准；`projectairsim_source` 是默认入口。

原生能力、锁定版本、源码位置、接口限制与本项目扩展见
[ProjectAirSim 开发参考](../docs/ProjectAirSim原生显示接口核查.md)。
自建显示插件位于 `simulator/projectairsim/unreal/AerohubDisplay/`，
管理与适配位于 `simulator/manager/`、`simulator/projectairsim/`。
上游阅读不授权修改其源码；扩展方式遵守[开发者贡献指导](../docs/开发者贡献指导.md)。

## Python 客户端环境

AirSim、ProjectAirSim SDK 和 Simulator Manager 共用仓库根目录的 `.venv`。从仓库根目录执行：

```bash
bash simulator/setup_python.sh
.venv/bin/python simulator/projectairsim/setup_client.py --check
```

首次准备会按根 `pyproject.toml` 和 `uv.lock` 创建或同步 Python 3.12 环境。
完整准备另外同步 SDK 依赖，并从 `projectairsim_source` 指定的源码可编辑安装 SDK。只用 AirSim 时可以执行
`bash simulator/setup_python.sh --airsim-only`，稍后运行完整命令补装 SDK。

原生客户端通过 `projectairsim_python` 定位，默认使用根目录 `.venv/bin/python`。
显式指定的 profile 解释器或本机资源覆盖继续生效。仿真入口会优先使用仓库根环境，
Manager 单元测试也可用该解释器执行。详见 [ProjectAirSim 接入说明](../simulator/projectairsim/README.md)。

```bash
# 使用默认配置（先设置其中的安装路径）
bash simulator/simulator.sh start

# 或复制为独立场景，修改后显式选择
cp config/simulator/default.yaml config/simulator/forest.yaml
bash simulator/simulator.sh check --config config/simulator/forest.yaml
bash simulator/simulator.sh start --config config/simulator/forest.yaml
bash simulator/simulator.sh status
bash simulator/simulator.sh stop
```

也可以将 `SIMULATOR_PROFILE` 设置为所选 YAML 路径。指定 `--config` 优先于该环境
变量。无配置参数的 status/stop 从运行目录读取已有状态，不需要重复指定配置。
启动前需加载当前桌面会话的
DISPLAY/XAUTHORITY；`render: offscreen` 可选择离屏运行。停止前确认 PX4 已降落并
disarm。此入口管理仿真和传感器 adapter，PX4/AS2 部署仍使用已有部署入口。

```text
config/simulator/default.yaml（或 --config 指定的 YAML）
├── backend              仿真后端
├── render               windowed / offscreen
├── ros_domain_id        仿真传感器 ROS 通信域，需与部署侧一致
├── projectairsim        安装资源引用，或直接填写 source / binary / python / ros_setup
├── scene                原生场景名，或自定义场景绝对路径
└── vehicles
    └── name / namespace
        ├── px4          可选：覆盖默认端口及 lockstep
        └── sensors
            └── 传感器名称
                ├── enabled
                ├── position / rotation
                ├── width / height / fov / captures（相机）
                └── channels / range / points_per_second / rotation_hz（LiDAR）
```

`position` 为原生机体 FRD 的米，`rotation` 为 roll/pitch/yaw 角度。
`captures` 支持 rgb、depth_planar、depth_perspective、segmentation。
未指定的物理、噪声、扫描参数保留上游模型。已有传感器按名称修改；新增 LiDAR
须填写 `type: lidar`，系统复用上游标准 LiDAR 模型。其他新增传感器在自定义原生
场景中定义，避免维护第二套机体/物理模型。

默认场景使用上游 `scene_px4_sitl.jsonc`，YAML 中的 DownCamera、IMU1 对应该场景
的传感器名称。多车辆使用含有对应 actor 的原生场景，并在 vehicles 中逐一配置。
车辆初始位置、物理模型和环境对象继续由原生 scene/robot 描述。

原生场景加载、默认 bridge 参数和 ROS 接口转换在实现侧完成，运行产物写入
`log/simulator/scene/`，包括 profile.json、runtime.json、resolved_scene.json 和
sensor-contract.json。无需在 log 里维护输入文件，也无需配置旧的 resources.yaml。
桥接器外部 overlay 的首次构建方法见 `simulator/projectairsim/SENSORS.md`。

旧 JSON profile 和 `--profile` 仍可使用。AirSim YAML 可使用同一原生字段：

```yaml
backend: airsim
binary: ~/AirSim/scenes/nh/AirSimNH/Binaries/Linux/AirSimNH
project: AirSimNH
settings: ../../simulator/airsim/config/settings/px4-sensors.json
args: [-RenderOffscreen, -vulkan, -nosound]
```

AirSim 的传感器仍在其原生 settings 中维护；本次简单传感器 YAML 选项已实现于
ProjectAirSim，尚未将其自动翻译到 AirSim settings。两种后端均从 config/ 选择入口。

## UE 编辑器开发

打开 Blocks 工程，点击 Play 后，从仓库根目录连接当前实例：

```bash
bash simulator/simulator.sh attach --config config/simulator/blocks.local.yaml
bash simulator/simulator.sh status
bash simulator/simulator.sh stop
```

`attach` 会按同一份 YAML 重新加载无人机和传感器，并启动桥接；只在没有活动
飞行任务时使用。它不会启动第二个 Blocks，也不会管理编辑器进程。
`stop` 仅停止该会话启动的桥接，不停止 UE Play、不卸载无人机、不停止 PX4。
因此飞行结束并降落、disarm 后再 stop。停止 Play 后端口失活时 status 返回失败。
重新 Play 前先 stop 旧桥接，再 attach。当前 status 检查服务端口及桥接进程存活，
不代表传感器持续出数或 PX4 已连接；仍须检查下游数据新鲜度。

## 观察显示

安装独立的 `AerohubDisplay` UE 插件后，同一份场景 YAML 的 `display` 字段控制
顶部状态条与速度仪表盘。启用 display 时 manager 自动管理真实运动学速度转发进程；
当前相机小画面不绘制。初始化安装、数据语义和接口见
[UE 显示说明](../simulator/projectairsim/unreal/README.md)。

```bash
bash simulator/simulator.sh display --event status
bash simulator/simulator.sh display --event start
bash simulator/simulator.sh display --event finish
bash simulator/simulator.sh display --event reset
```

这些事件只操作显示计时器，不代替实际任务或飞控指令。
