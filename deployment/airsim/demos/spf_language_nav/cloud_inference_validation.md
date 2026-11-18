# SPF 下层迁移：Cloud Inference Function

最新真实云端与 AirSim 回归见 [live_cloud_validation.md](live_cloud_validation.md)。
下方无凭证/未飞行描述保留为各阶段的历史状态。

## 当前交付与调用链

SPF 私有 `uav_spf_vlm_proxy` 包及其 `spf/vlm/query|result` 接口已移除。
导航 prompt 和 point/depth 解析原样移入 SPF Tool；认证、供应商协议、模型路由与
图像编码进入 Inference Function。新链路：

```text
SPF Tool(model_name + prompt)
  → InferVisionLanguage Action（vehicle namespace）
  → Inference ROS Adapter（该 vehicle RGB，原采集 header，JPEG）
  → 公共 FastAPI runtime
  → Cloud execution plugin
  → 配置的供应商
  → 通用 text + 图像 header
  → SPF Tool 解析、反投影与轨迹 → 既有 FollowTimedPath
```

CUDA、Houmo、Cloud 使用惰性插件入口；内部接口可不同，不强行统一成张量 ABI。
此 VLM adapter 目前只支持 Cloud；其他 CUDA/Houmo 模型的输入输出与服务保持兼容。
未知模型或不支持的引擎明确失败，不静默替换。

## 用户配置（当前：逻辑模型 → 引擎）

复制 `config/inference.cloud.example.yaml` 为被忽略的 `config/inference.cloud.local.yaml`：

```yaml
models:
  spf:
    cloud:
      base_url: https://open.bigmodel.cn/api/paas/v4
      model: glm-4v-flash
      api_key: "你的密钥"
```

对外模型名为逻辑 `spf`，内部 model 是供应商模型。未声明的模型/引擎不支持；
唯一引擎自动选用，多引擎必须设置 `active_engines.spf`，不存在静默回退。
当前 SPF 只实现 Cloud；选择尚未实现的本地引擎时拒绝加载，不改用 Cloud。
每条目可通过 api_key_env/base_url_env/model_env 显式引用环境变量；未引用的全局
OPENAI_* 不覆盖其他模型，也不会隐式创建 Cloud 支持。
不使用向导或 XDG 目录，旧文件不改写。与较早的平铺配置、环境变量独立模式不兼容，
请按示例手动调整本地结构（系统不会迁移或打印密钥）。

## 已执行的本地验证

使用固定 `onboard_computer:v0.1.1`；不改镜像。源码只读，ROS 产物在
`log/spf/inference-cloud/ros_ws`。正式 worker 准备入口已实际执行：在镜像的
Python 3.10 下，向 `log/spf/pc-e2e/cloud-env`（容器内 `/cloud-env`）安装锁定依赖。
初次直接从只读源构建 runtime 因 setuptools 写 egg-info 失败；修正为复制本项目
runtime 构建输入到容器临时目录后通过。测试随后加载这个正式环境，而非只测宿主临时 venv。

构建包：uav_inference_interfaces、uav_inference、uav_planner_interfaces、
uav_spf_planner、uav_planner_python_api、uav_follow_path_execution。

测试入口与覆盖：

| 入口 | 内容 | 证据文件（上述日志目录） |
| --- | --- | --- |
| `deployment/inference/tests/test_cloud_function_ros.py` | 合成 padded RGB → 实际 ROS Function → 公共 HTTP runtime → 实际 Cloud 插件 → 本地协议替身 → SPF 实际文本消费；原 prompt/header、模型路由、取消保留 busy、陈旧输入/结果、编码错误、未知模型、timeout | cloud-function-ros.txt |
| `deployment/inference/tests/test_cloud_unit.py` | 文件/环境变量/默认值及优先级、无效配置错误脱敏、惰性插件、OpenAI-compatible/Gemini 本地协议、错误脱敏 | cloud-unit.txt |
| SPF `test_semantic_navigation_ros.py` | 接入新的 typed Inference Action 后复跑 12 项 timeout/取消/停止门控 | navigation-ros.txt |
| `test_runtime.py` / `test_selection.py` | 既有 runtime busy/取消及选择器行为 | runtime-unit.txt / selection-unit.txt |
| SPF projector/trajectory/prompt/parity 与 `test_cloud_protocol.py` | 原有 23 项测试迁移后的等价回归 | spf-existing-tests.txt |
| FollowPath C++ timeline | 执行消费者回归 | follow-path-tests.txt |

原上游子模块已按父仓库 gitlink 初始化到
`5621bcf43e9826d60df014541dd0498e743a92bd`，未修改 gitlink 或上游源码；
prompt 保真测试用真实锁定源码比较，早期因未初始化导致的两项失败不作为最终通过证据。

复跑时加载 ROS、AS2 和本轮 overlay，选择未占用 domain；给容器挂载只读仓库 `/repo`、
日志 `/evidence`、正式 worker 环境 `/cloud-env`：

```bash
source /opt/ros/humble/setup.bash
source /root/aerostack2_ws/install/setup.bash
source /evidence/ros_ws/install/setup.bash
export ROS_DOMAIN_ID=186 ROS_LOCALHOST_ONLY=1 PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=/cloud-env/lib/python3.10/site-packages:/usr/lib/python3/dist-packages:$PYTHONPATH
/cloud-env/bin/python /repo/deployment/inference/tests/test_cloud_function_ros.py
python3 /repo/src/tool/navigation/planner/spf/ros2_adapter/tests/test_semantic_navigation_ros.py
```

测试中 Key 和模型为明确标注的虚构值；网络关闭，只允许容器 loopback 协议服务。
本地 HTTP 仅在显式测试配置允许，实际配置命令只接受 HTTPS。

## 不能据此声明完成的部分

- **真实云端供应商尚未验证**：没有用户填入的授权凭证，不伪造在线成功；配置命令准备好后由用户填入，再实际测试。
- **真实平台 RGB 消费、新部署拓扑和有屏 AirSim + PX4 SITL 尚未验收**。合成 RGB 和本地协议返回不代替它们。
- 原有 SPF 转向符号缺陷仍待独立修复，不能据本轮 Function 测试宣布飞行成功。
- `to_semantic_goal` 仍无到达判定：timeout 停止返回 TIMEOUT，不报告目标成功。
- 本轮不涉及本地 CUDA 模型执行；没有以本地 GPU 测试替代远端真实模型验证。
- 实际供应商可能使用可变模型别名；依赖锁/模型名不等于权重版本可复现。

声明等级为“下层归属实现与本地接口验证”，**不是完整 SPF 迁移/复现成功**。
历史 `validation.json` 与前一轮 `semantic_contract_validation.md` 保留各自原边界，
新 Cloud 链以本文件及本轮日志为准。


## 最终本地结果

Function ROS 7/7、Navigation ROS 12/12、Cloud 配置/协议 8/8、公共 runtime 2/2、选择器 3/3、迁移后原有测试 23/23、C++ timeline 1/1 通过。
独立 Tool/Function 两容器链通过；证据在 `log/spf/inference-cloud/cross-container/`，使用 internal Docker 网络和本地协议替身，无真实 API 或飞行。
最终同步 pytest 显式禁用无关插件自动发现：镜像旧 pytest 与 worker anyio 插件不兼容；没有跳过测试项。
主机磁盘已满，未清理用户已有数据、镜像或项目产物。只移除了本次临时环境/缓存及本次自动下载且未使用的远端镜像标签；原有固定镜像保留。配置真实密钥前需先恢复主机可写空间。
当前仍缺真实凭证、真实平台 RGB 与新链飞行验收，不提升为完整迁移或复现成功。


## 配置简化后的回归

用户要求改为常规 ignore YAML / 环境变量，已移除交互向导及 XDG 依赖。
先前“最终本地结果”对应简化前版本；本次配置回归日志保存于
`log/spf/inference-cloud/config-simplification/`，不是新增真实 API/飞行证据。

简化版实际回归：配置/协议 **12/12**、Function ROS **7/7** 通过。
覆盖仅文件、仅环境变量、环境覆盖文件、默认 URL/model、无环境密钥的文件凭证调用；
本次仍仅本地协议服务，未新增真实供应商或飞行验收。


## 模型/引擎目录回归（当前版本）

当前结构为 `models.<logical_model_name>.<engine>`，多引擎选择放在顶层
`active_engines`。SPF 准备清单、Tool 默认参数和 worker 的默认服务名统一为 `spf`；
当前 worker 只消费自己负责的逻辑条目，不因同文件中有 YOPO/CUDA 就尝试加载它。
本轮证据：`log/spf/inference-cloud/model-engine-config/`，不将旧格式测试结果当作新格式证明。
真实供应商、真实平台输入、飞行与本地 SPF VLM 仍未验收。

模型/引擎目录实际结果：配置/协议 **20/20**、Function ROS **7/7**、SPF 生命周期
**12/12**、准备选择器 **3/3**、原有 SPF 回归 **23/23** 通过；两容器链亦通过。
Function 测试退出日志出现一次 rclpy 异步销毁警告（断言仍通过），原样保留，
不据此宣称退出路径、真实供应商或飞行全矩阵通过。
