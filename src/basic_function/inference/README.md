# Inference Robotic Function

## TODO

- 支持独立 Demo 跨 DDS domain 共享一个 VLM worker。当前 SPF 和 CityNav 共用 `vlm` 绑定及启动入口，每次部署可同时提供单图和多图 Action；两个 Demo 分别启动时仍各有一个实例。后续需明确共享实例的连接、健康检查、启停归属以及并发请求的处理方式。

YOLO26n-seg is available as an isolated segmentation worker and typed ROS service:
see [model preparation and validation](models/yolo26n_seg/README.md). Its separate
[Object Detection Tool](../../tool/object_detection/README.md) adds registered
ROS depth/CameraInfo and historical TF for measured 3D positions; target KF and
motion execution remain outside Inference. Select only `yolo26n-seg` with
`config/inference.yolo26n-seg.example.yaml`; existing model selections are retained.

[Grounding DINO tiny](models/grounding_dino/README.md) 提供文本条件二维框推理及
`InferGroundingDino` typed ROS 服务，模型 ID 为 `grounding-dino-tiny`。
使用[专用配置](../../../config/inference.grounding-dino.example.yaml)选择该模型；
[组件清单](../../../deployment/inference/grounding_dino.component.yaml)与
[能力卡片](../../../cards/inference/grounding-dino.yaml)关联既有 Engine 验证。
当前为 `partial`，已存真实图像验证不代表实时传感器、Tracking 或飞行验收。

Inference 是与 Perception、State Estimator 并列的基础能力：对外使用 ROS 2，内部通过 FastAPI 隔离模型服务。CUDA、Houmo、Cloud 是可选执行插件；模型适配内部可使用张量或供应商协议，调用者不选择设备/协议。CUDA/Cloud 使用独立 uv 环境，RoboPi HMM 使用板端 Python 与应用私有运行包。

当前提供公共 FastAPI runtime、ROS HTTP 客户端、模型 manifest、用户 YAML 准备入口、原生 AS2 双容器通信基准，以及 [YOPO](models/yopo/README.md)、[DiffPhys](models/diffphys/README.md)、[YOLO26n-seg](models/yolo26n_seg/README.md) 和 [Grounding DINO](models/grounding_dino/README.md) typed ROS 推理服务。YOPO Planner 的轨迹重建和执行在 `src/tool/navigation/planner/yopo/`，模型服务只返回候选 PVA/cost；DiffPhys返回原始策略输出和显式GRU隐状态，尚无运动执行接入。

## 开始前准备什么

先确定是调用已部署服务、部署已有模型，还是开发/迁移模型。准备条件按实际任务选择：

| 任务 | 必须准备 | 入口 |
| --- | --- | --- |
| 调用已就绪的 ROS 服务 | ROS 2 Humble、匹配的 `uav_inference_interfaces`、可发现服务的 ROS 网络/domain | 本文“YOPO ROS 服务” |
| 在 NVIDIA 上运行 YOPO | Linux x86_64、Git、uv、可用 NVIDIA GPU/驱动、锁定 Python/wheel 的网络或缓存 | [YOPO 准备与启动](models/yopo/README.md#准备) |
| 在 RoboPi 上运行已有 HMM 包 | 匹配的板端 Python/ROS、TCIM/HAL/驱动、完整模型运行包、Inference ROS overlay | 下方“RoboPi 使用者” |
| 开发 CUDA 模型适配 | 对应模型的独立环境、源码与权重；构建 ROS adapter 时使用 ROS Humble/colcon | [CUDA Skill](../../../skill/uav-prepare-cuda-inference/SKILL.md) |
| 将模型迁移为 HMM | Docker、固定 Houmo 工具链镜像、OHOS Native SDK；板端验证所用匹配运行环境 | [Houmo Skill 工具准备](../../../skill/uav-compile-model-for-houmo/SKILL.md#工具准备) |

调用已部署服务的客户端只需 ROS 接口，无需准备模型权重、CUDA 或 Houmo 工具链。
NVIDIA worker 的 Torch/CUDA 运行库由模型锁定依赖准备；当前 YOPO 无需额外安装
宿主 CUDA Toolkit。ROS 留在单独环境，模型 venv 不安装 ROS。
`setup_models.py` 的模型准备入口目前调用 CUDA `setup.sh`，不会自动转换 HMM 或部署 RoboPi。

### RoboPi 使用者

运行已有 HMM 服务需拿到以下内容：

1. 板端基础环境：匹配的 Python 与 ROS 2，以及可工作的 Houmo TCIM/HAL/驱动。
2. 完整模型运行包：HMM、CPU 处理参数、对应 ABI 的 `tcim_lite` 绑定、目标动态库、
   服务代码及 Python 依赖。YOPO 的打包输出为 `board-runtime/`。
3. ROS overlay：匹配板端 ABI 的 `uav_inference_interfaces` 和 `uav_inference`。
   仅启动内部 HTTP worker 可不加载此 overlay；对外 ROS 服务必须加载。

完整运行包准备好后，板上无需 Docker、OHOS SDK、uv、Torch 或 postmo 源码。
仓库不提交 HMM、目标库和运行包；新用户需由迁移开发者提供匹配的运行包，或按
[YOPO Houmo 说明](models/yopo/houmo/README.md)自行构建。不能只拉取仓库就假定模型产物已存在。
该说明记录了已查询的板端依赖、打包、启动和 ROS 检查命令。

### Houmo 开发者按阶段复用的依赖

基础开发工具是固定工具链镜像和 OHOS Native SDK，完整版本与获取说明以
[Houmo Skill](../../../skill/uav-compile-model-for-houmo/SKILL.md#工具准备)为准。
在此基础上，只有执行对应步骤时才需要以下现有目标资源：

| 执行步骤 | 需要复用的资源 | 配置方式 |
| --- | --- | --- |
| 编译板端 runner | AArch64 TCIM 开发包的头文件和目标库 | `TCIM_RUNTIME_PATH` |
| 编译 TCIM Python 绑定 | 同版本 TCIM 的 `python/` 源码、目标 Python 头文件和 `libpython` | `TCIM_RUNTIME_PATH`、`OHOS_SYSDEPS` |
| 打包板端服务 | 已编译的绑定、目标 TCIM/HAL 库、模型文件和运行参数 | 打包脚本 `--tcim-lib`、`--hal-lib` 等参数 |
| 编译 ROS overlay | 现有 OHOS ROS builder、已准备的隔离 SDK/ROS 工作区及目标依赖 | `OHOS_WORK_ROOT`、`OHOS_SYSDEPS`、`OHOS_INSTALL_PREFIX` |
| 上传与板端验证 | OHOS SDK 的 HDC、可连接的 RoboPi | `HDC_BIN`、`HDC_TARGET` |

这些资源优先复用已有 OHOS 开发环境；新开发者缺少目标包时需取得与板端版本匹配的
TCIM/HAL 和 Python/ROS 开发资源，工具链镜像的 x86 库不能替代它们。
ROS builder 的准备入口见 [OHOS 构建说明](../../../deployment/oh-ros-build/README.md)，
本地路径填入忽略的 `.env`，字段示例见 [`.env.example`](../../../.env.example)。
模型专有导出脚本、校准数据和图修改属于适配工作，不列为通用安装工具。

## 文件职责

- `runtime/`：公共 FastAPI 应用工厂，每进程加载并预热一个模型，提供 `/health/live`、`/health/ready`、`/infer`。模型提供 factory、请求与响应 Pydantic 类型。
- `ros2_adapter/`：`uav_inference` ROS Python 包，提供内部 HTTP 客户端及 YOPO、DiffPhys、YOLO26n-seg、Grounding DINO ROS Service Adapter。
- `manifest.json`：已支持模型 ID、仓库相对目录和描述；以实际 manifest 为准。
- `models/`：各模型各自提供 README/setup、依赖声明和锁文件、schema 与推理 adapter。算法源码按功能归属保留原目录。

## 模型选择与准备

用户配置统一使用 YAML。将下面片段加入已有用户配置，通过 `--config` 指定路径。独立配置默认路径为 `config/inference.local.yaml`，该本地文件被忽略，示例见 [inference.example.yaml](../../../config/inference.example.yaml)。读取脚本使用部署环境已有的 PyYAML（`import yaml`）。

```yaml
inference:
  models: []
```

```bash
python3 deployment/inference/setup_models.py --list
python3 deployment/inference/setup_models.py --config config/inference.example.yaml --dry-run
python3 deployment/inference/setup_models.py --config config/inference.example.yaml
```

Grounding DINO 的选择检查：

```bash
python3 deployment/inference/setup_models.py --config config/inference.grounding-dino.example.yaml --dry-run
```

在 `inference.models` 中填写其 ID，脚本先验证全部选择，拒绝未知或重复 ID，再按顺序调用各模型 `setup.sh`；失败停止。`--check-cuda` 将同名标志转交模型准备脚本。空列表不操作，不卸载未选模型、不启动服务。现有本地配置如选择尚未登记的模型，会明确报错，不自动改写用户配置。

manifest 不重复保存模型版本、权重或依赖。新增模型必须有 README、setup.sh、pyproject.toml 和 uv.lock，CUDA 过程使用 [uav-prepare-cuda-inference](../../../skill/uav-prepare-cuda-inference/SKILL.md)，云端模型不要求本地权重或 CUDA 检查。每模型环境与 ROS overlay 分离，不重建稳定 AS2 镜像。

## 公共运行约定

模型 worker 和 ROS Adapter 属于算力角色，工作站通过 ROS 调用。内部 HTTP 不取代对外 ROS 契约；不同容器需显式配置可达 endpoint。具体请求类型、时间/坐标、ROS Service/Action/Topic 在模型接入时定义。

runtime 忙时返回429，不无限排队；调用方断开或取消后，底层推理完成前保持 busy。HTTP 超时不代表 GPU 计算已停止，客户端不自动重试旧观测。当前不提供跨模型 GPU 仲裁或自动启停管理。

## 验证

### 地面站到 onboard 的原生 AS2 通信基准

Inference 的部署验收以“地面站 ROS 客户端 → onboard ROS 服务端”为基准，客户端和服务端必须在不同容器/进程；同进程的本地 smoke 不替代这一层。先运行原生 AS2 只读查询基准：

```bash
python3 deployment/inference/tests/cross_container_baseline.py
```

脚本复用固定 `osrf/ros:humble-desktop-full-jammy` 与 `onboard_computer:v0.1.0`，在独立 Docker bridge/domain 中运行原生 Pixhawk Platform。地面站仅从 onboard 镜像的规范 `.srv` 定义局部生成 `ListControlModes` 查询类型，不复制 AS2 二进制，不运行替代服务。调用30次 `platform/list_control_modes`，要求每次返回 `source=Platform` 和非空模式列表；只测量时延，不设置性能达标门限。无 PX4、无运动调用，不修改现有运行实例。测试后清理本次临时容器和网络，镜像不重建。

2026-11-10 实测30/30成功，p50 3.32 ms、p95 3.88 ms；证据 `log/inference/as2-baseline/1391046/`，包含镜像 ID、规范服务定义哈希、逐次时延、构建及原生节点日志。此为单宿主双容器通信基准，不代表跨物理设备、完整 AS2 飞行链路或 Inference 已在该部署拓扑通过。

YOPO 另以 `deployment/inference/tests/yopo_cross_container.py` 验证工作站容器客户端 → onboard Adapter → GPU worker。2026-11-10 的320×185合成输入10/10成功，往返中位17.76 ms、最大50.95 ms；证据 `log/yopo-single/model-chain.json` 与 `model-chain-manifest.json`。它仍不证明真实相机或飞行通过；Planner/仿真结果见 [YOPO Demo](../../../deployment/airsim/demos/yopo_single/validation.md)。

### 公共逻辑测试

配置选择测试仅需系统 Python 与 PyYAML；runtime 测试使用独立临时环境，不依赖任何模型：

```bash
python3 -m unittest discover -s deployment/inference/tests -p test_selection.py -v
PYTHONPATH=src/basic_function/inference/runtime uv run --no-project --python 3.8 --with-editable ./src/basic_function/inference/runtime python -m unittest discover -s deployment/inference/tests -p test_runtime.py -v
```

公共逻辑、AS2 只读通信与 YOPO 模型链分别报告；跨物理设备和真机未验证。实际 AirSim/Planner 证据以各 Demo 验收记录为准。

## YOPO ROS 服务

在共享 ROS 环境补编译 `interfaces/` 与 `ros2_adapter/`，显式 source overlay，再执行：

```bash
python3 -m uav_inference.yopo_node --ros-args -r __ns:=/drone0 -p endpoint:=http://127.0.0.1:8000
```

服务 `/drone0/inference/yopo/infer` 使用 `uav_inference_interfaces/srv/InferYopo`。实际测量 `observation_header` 与图像采集时刻分别检查，`state_header` 是规划参考时刻/对齐坐标，可有界未来（默认300ms）。响应保留两者、请求ID、候选/评分、时长、模型配置与设备。完整契约见 [Planner 接口](../../tool/navigation/planner/yopo/README.md)。

YOPO Adapter另发布`<namespace>/inference/yopo/timing`（std_msgs/String JSON，以request_id关联）。包含ROS图像转换、内部HTTP、Adapter耗时及worker预处理/前向传输/解码分项；worker耗时属于HTTP子项。最新2 m/s双容器GPU结果及动力学见[Demo验收](../../../deployment/airsim/demos/yopo_single/validation.md)。


## 模型逻辑与执行引擎

模型前后处理与具体设备执行分开维护。YOPO 的 CUDA/HMM 共用
`models/yopo/processing.py` 和 `predictor.py`；各 adapter 只准备模型并选择后端。
Torch 与 HMM 后端均使用命名 NumPy 输入输出、固定张量契约及一致的生命周期和
分项计时，详见[后端契约](runtime/aerohub_inference/backends/README.md)。
Torch 后端是模型环境内的可选导入，公共 HTTP runtime 和 RoboPi worker 不依赖 Torch。
ROS service、HTTP schema、busy/取消语义继续由现有公共设施提供。


## Cloud alternative 与 model_name

新增 [Vision-language Function](models/vision_language/README.md) 使用
`InferVisionLanguage` Action，对外选择 `model_name`，输入通用 prompt，返回文本与实际
RGB 图像采集 header；不解释导航点、语义深度或运动。SPF 是其一个消费者。
既有 YOPO、DiffPhys、YOLO 的 typed 服务保持兼容，不伪装为已经统一成一种万能消息。

```bash
cp config/inference.cloud.example.yaml config/inference.cloud.local.yaml
```

配置使用 `models.<model_name>.<engine>`，例如 `models.vlm.cloud` 中填写
`base_url/model/api_key`。对外调用 `model_name="vlm"`，内部 `model` 可为 GLM 等供应商
模型；更换供应商不改变逻辑身份。缺省的引擎不支持，环境变量也不会自动创建声明。
一个引擎直接选用；多个引擎必须配置 `active_engines.vlm: cloud`，选择失败不回退。
当前 SPF 只有 Cloud 实现，不能只加 CUDA/Houmo 配置就声称本地已支持。

每个条目可用 `api_key_env`、`base_url_env`、`model_env` 引用环境变量，被引用的值
覆盖该条目的文件值，不使用全局 OPENAI_* 覆盖所有模型。保留单个被忽略的配置文件，
无需向导或 XDG 密钥目录；请勿提交/打印密钥，修改后重启 worker。旧配置不自动改写。

`runtime/aerohub_inference/engines.py` 惰性注册 `cuda`（既有 TorchBackend）、
`houmo`（既有 HmmBackend）、`cloud`（CloudBackend）。三者不是一种张量 ABI；模型
适配决定内部输入输出。现有模型无需为新增 Cloud 改动，VLM 目前只提供 Cloud 实现，
无相应本地产物时明确拒绝 CUDA/Houmo，不隐式换模型或回退。

Cloud worker 复用公共 runtime 的 busy/health/关闭规则。取消 ROS 请求不代表远端计算
停止；执行槽保留到实际 HTTP 调用结束，HTTP runtime 也独立防止计算未结束时重入。
不自动重试旧观测，真实图像年龄在推理前后检查。云端配置不必有本地权重，但仍需
锁定客户端依赖、供应商模型标识与验证证据；可变模型别名不等于可复现的权重版本。

本地协议、ROS 与真实云端/平台/飞行验证分开报告：见
[Cloud 验证](../../../deployment/airsim/demos/spf_language_nav/cloud_inference_validation.md)。
