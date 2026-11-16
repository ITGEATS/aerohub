# 目标追踪任务样例与独立 Tracking Tool

## 当前 Demo 默认：Grounding DINO / car（平台验收未完成）

Mission → Tracking → DetectSnapshot → InferGroundingDino；8003 worker 为
`grounding-dino-tiny`。使用统一 query overlay 和 Demo 专属 params，保留深度、新鲜度门。
[当前部署命令与只观测/显式悬停步骤](../../../target_tracking/README.md#dino-default)。
以下原 YOLO 结果、截图和类别诊断属于历史阶段，不代表 DINO 已验收。
`diagnose_classes.py` 仅供显式 YOLO 历史诊断，DINO 不自动运行。



已新增工作站编排样例 `mission.py`，尚未完成飞行验收。已实现独立、只观测的 Target Tracking Tool：
串行调用已有 `DetectSnapshot(estimate_position=true)`，维护位置、速度、协方差、
确认/预测/失跟状态与会话生命周期；不调用导航、不生成运动指令。

参考子模块已经登记并锁定：
`reference/yopo_tracker/`，官方
[TJU-Aerial-Robotics/YOPO-Tracker](https://github.com/TJU-Aerial-Robotics/YOPO-Tracker)，
提交 `d313f8610195dc986caceff7a3616f63d7e6f31f`。其他工作区使用：

```bash
git submodule update --init deployment/airsim/demos/target_tracking/reference/yopo_tracker
```

本次运行依赖该锁定源码的 `YOPO/policy/target_ekf.py`，构建时原样安装原文件与
MIT LICENSE；不使用其联合网络/控制器，不将整个 YOPO-Tracker 接成新 Planner。

| 职责 | 实现与边界 |
| --- | --- |
| 单次类别观测 | Object Detection Tool：真实 RGB-D、采样时刻 TF、按对象三维有效性 |
| 状态维护 | [Target Tracking Tool](../../../../src/tool/target_tracking/README.md)：最近门内关联、M/N 确认、速度估计、短时预测及失跟；无外观身份保证 |
| 跟随策略与编排 | `mission.py`：水平间距、当前自身高度、不约束 yaw；失跟停止退出 |
| 连续导航 | 现有 `NavigateToPose` 已有同会话 `UpdateNavigationTarget`、HOLD 与目标超时接口；见 [Navigation Module](../../../../src/tool/navigation/python_api/README.md) 的实际验证范围 |
| 运动执行 | 现有基础 Behavior/AS2/PX4；本 Tool 不取得运动资源 |

导航接口已接入不能推导移动目标跟随已验收；不再沿用“公共导航不能更新目标”的旧描述。
样例复用单个 HOLD 导航会话，不以取消重建任务替代连续跟随。

部署入口与测试见 [Tracking 部署](../../../target_tracking/README.md)，
结果与未完成矩阵见 [VALIDATION](../../../target_tracking/VALIDATION.md)。
样例 Mission 在工作站调用已部署的算力侧检测、跟踪、导航能力，
高频控制仍在算力侧；须独立验证移动目标、遮挡、多目标关联、输入中断、取消及运动资源交接。

## 工作站编排样例（未完成飞行验收）

先部署检测、Tracking 与导航服务，并显式加载 ROS/AS2 和 Python API install overlay。
默认只观测，创建跟踪会话、计算并记录跟随点，不调用导航：

```bash
python3 deployment/airsim/demos/target_tracking/mission.py --query car --duration 60
```

显式执行会产生运动请求。操作者必须先确保平台已经安全飞行，connected、armed、
offboard 且 PlatformStatus 为 FLYING。每次导航请求前检查平台时间戳及接收新鲜度。
脚本不解锁、不起飞、不切换 offboard、不降落、不启动容器、不调用 AirSim RPC。
停止后的平台安全处置由操作者负责。

```bash
python3 deployment/airsim/demos/target_tracking/mission.py --query car --duration 60 --execute-navigation
```

`--rate 2` 为工作站更新频率，`--distance 5` 为水平间距。沿自身到目标的水平连线
停在目标前；间距内保持自身 xy，每轮使用自身当前高度，不采用地面目标 z。
`to_point` 使用 UNCONSTRAINED orientation，不假定 yaw 或自动面向目标。
默认固定坐标系 `earth`（`--frame` 可改），Pose 和 Tracking 必须同 frame，不隐式 TF。
`--max-age 1` 限制 ROS 时间年龄和单调墙钟接收年龄，检查 valid、valid_until、未来
时间戳和 clock regression。默认仿真时间；墙钟 ROS 环境可选 `--use-wall-time`。
`--initial-wait 15` 限制初始目标等待；服务就绪也有单独相同上限。

导航只有一个 `arrival_behavior=hold`、`wait=False` 会话，重复更新目标，任务 timeout
等于 `--duration`，`--target-timeout` 默认 3 秒。目标失效/过期、输入陈旧、更新拒绝、
导航提前结束或时钟回退均退出，不自动重启。Tracking 仅预期在请求时长结束时返回
TIMEOUT，其他提前结束视为错误。Ctrl+C/异常先取消导航并等待最终 stop_confirmed，
随后取消 Tracking，限时排空结果（`--cleanup-timeout 10`）。取消 ACK 不代表停止；
提交响应不确定或停止无法确认以非零状态退出并记录缺口，仍尝试收束 Tracking。

导航目标更新需要重编译当前 `uav_planner_interfaces`（含更新的 Action 和
`UpdateNavigationTarget.srv`）、对应导航 adapter、`uav_planner_python_api`，并加载
一致的 install overlay。Tracking 接口、adapter 和 Python API 也须构建就绪。
复用稳定镜像，在外部工作区补编译；接口/ABI 改动连同受影响消费者重编译，不混用
镜像旧插件。详见 Navigation Module 文档中的目标更新要求。

默认日志位于仓库 `log/target_tracking/<UTC时间>/mission.jsonl` 与 `summary.json`，
可用 `--output` 指定。退出码 0 只表示预期编排时长与清理完成，属于观测/编排结果，
不是动态跟随、避障或飞行验收成功。

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover -s deployment/airsim/demos/target_tracking -p test_mission.py -v
```

纯逻辑测试覆盖跟随几何、新鲜度安全门、时钟回退和参数默认值；不验证 ROS Action、
实际平台停止确认、移动目标/遮挡或 AirSim + PX4 SITL 动力学闭环。
