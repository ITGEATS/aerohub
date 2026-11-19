# SPF（See, Point, Fly）Planner

上游：https://github.com/Hu-chih-yao/see-point-fly （CoRL 2025，training-free VLM 无人机语言导航）
子模块锁定提交：`5621bcf43e9826d60df014541dd0498e743a92bd`（2026-01-29 tip；四项关键事实抽查
与已删除 zip 快照全部一致，FOV 行 L30-31、prompt L111、duration L122、requires-python>=3.13）

> **⚠️ 许可证合规声明**：上游 LICENSE 为 **Proprietary Software License**（非开源协议），
> 明文禁止未经书面许可的复制、修改、分发、衍生作品与托管，仅允许非商业使用，副本须保留
> 本声明。本仓库以 **git 子模块引用**（仅存 gitlink，不复制源码入库）方式使用上游；
> 适配层中的算法逻辑迁移与 prompt 复制的合规边界见 §8，许可咨询邮箱：
> tommy60718.en10@nycu.edu.tw / jj2095813@gmail.com。

## Navigation Tool：`to_semantic_goal`

SPF 是 Navigation Tool 的 alternative，目前只支持 `to_semantic_goal`，不支持
`to_pose`。新增公共 Action `uav_planner_interfaces/action/NavigateToSemanticGoal`，
地址 `/<vehicle>/navigate_to_semantic_goal`；输入为 `description` 与必填的有限正数
`timeout`（单调墙钟秒数）。AS2 Python 调用：

```python
drone.navigation.to_semantic_goal("靠近橙球", timeout=60.0, wait=False)
# 使用已有 done / feedback / result / cancel()；接受不代表完成。
```

**当前不实现语义到达判定，也不报告 SUCCEEDED。** 超时覆盖推理等待、重试退避与
运动执行；到期停止后返回 `TIMEOUT`、`success=false`。取消返回 `CANCELED`；算法或
执行错误仍可提前失败。停止需服务确认、待提交/执行 Action 已收口，以及新鲜三维
速度 <0.2 m/s 持续 1 秒；停止有额外 10 秒宽限，失败返回 `STOP_UNCONFIRMED` 并保持
FAULT，不允许新任务交接。不会新增 `guide_by_description`。

旧 `spf/goal` 仅保留为调试入口，`max_cycles` 耗尽返回 `EXECUTION_FAILED/step_limit`，
不代表目标达成；正式 Action 不带轮数上限。云端调用现已下沉到 Inference Function，
Tool 只选择 model_name；Function 返回实际图像 header，并拒绝陈旧输入/结果。
转向符号已按上游 NED 顺时针到 ENU yaw 的约定修正，右/左及非零航向回归通过；飞行结果仍须单独验收。当前真实云端回归见 [记录](../../../../../deployment/airsim/demos/spf_language_nav/live_cloud_validation.md)。

当前变更的验证边界见 [契约验证](../../../../../deployment/airsim/demos/spf_language_nav/semantic_contract_validation.md)。
历史飞行证据不验证新增 Action 或 timeout 语义。

本文档是 SPF 接入 aerohub 的契约主文档。迁移方案与验收矩阵见
`.trae/documents/spf-seepointfly-migration-plan.md`；过程记录在 `skill/temp/spf-migration/`。

## 1. 上游真实管线（源码追踪，非论文宣称）

启动入口 `spf airsim`（`src/spf/main.py`）→ `airsim/main.py` 主循环。每周期：

| 步骤 | 能力 | 输入 | 输出 | 源码位置 |
| --- | --- | --- | --- | --- |
| 取图 | FPV 相机帧 | AirSim RPC `simGetImages` | 1920x1080 ndarray | `upstream/src/spf/airsim/main.py` |
| 语义标注 | 云端 VLM（Gemini 2.5 Flash / OpenAI 兼容） | 图像 + 语言指令 prompt | JSON：`[{point:[y,x] 0-1000 归一化, depth 1-10, label}]` | `airsim/action_projector.py` `_get_single_action` |
| 反投影 | 2D 点 + 深度档 → 机体系位移 (右,前,上) 米 | 归一化像素、`depth/10*2` 米 | `ActionPoint(dx,dy,dz)` | `base/action_projector.py` `reverse_project_point` |
| 动作分解 | 位移 → "先偏航对准（\|θ\|>10°），再恒速前飞 + 按比例爬升" | ActionPoint + 配置速度 | 命令序列 rotate_yaw / move_velocity_body | `airsim/drone_space.py` `action_to_commands` |
| 执行 | AirSim SimpleFlight 直控 | 命令 | `moveByVelocityBodyFrameAsync` 等 RPC | `airsim/controller.py` |

关键既定行为（复现时勿误判为 bug）：

- **FOV=108° 硬编码**（`base/action_projector.py`，Tello 假设）且 x/z 共用单因子；
  AirSim 相机默认 90°，上游自身存在几何失配。Legacy 段保留原值复现；AS2 段以
  CameraInfo 真针孔覆写（偏离 D3）。
- **min_command_duration=2.0s**：`duration=max(d_xy/v, 2.0)`，名义步长 0.2~2 m
  实际前飞 ≥4 m（2 s x 2 m/s）。保留（偏离 D5）。
- 深度是 VLM 语义估计档，非传感器测距；不改为 LiDAR（偏离 D9）。
- **高度下限 D12**：上游无高度保护（论文场景目标在视线高度）；Blocks 场景橙球
  贴地，VLM 语义深度点低于地平线导致无界下降蹭地（run z5 与 Ground 碰撞），
  AS2 段执行层对 t>0 轨迹点钳制 `min_altitude_m`（Demo 1.2m；Legacy/默认参数
  保真分支不含）。完整偏离清单 D1-D13 见迁移方案文档（本地工作文档，以本
  README、Demo README 与 validation.json 为准）。
- 上游无完成定义：语言目标由外部循环持续驱动，无到达判定。
- **共享导航接口对齐 D13**：轨迹执行走共享 `uav_planner_interfaces`
  （`src/tool/navigation/interfaces`，与 YOPO/SUPER 同一 FollowTimedPath 契约）；
  收口状态携带共享 NavigateToPose Result 码表（SUCCEEDED/CANCELED/TIMEOUT/
  NOT_READY/EXECUTION_FAILED/STOP_UNCONFIRMED）+ `detail` 内部子原因 +
  `stop_confirmed`。语言目标无法表达为位姿，故不提供 NavigateToPose 位姿入口，
  历史调用面为 `spf/goal`(String JSON)+`spf/cancel`；当前正式调用面见上方 `to_semantic_goal`。

## 2. 职责映射与目录归属

| SPF 部分 | aerohub 层归属 | 目录 |
| --- | --- | --- |
| 语言指令 → 每步位移决策 | Planner（Navigation Tool 实现基础） | `src/tool/navigation/planner/spf/` |
| 反投影 + 动作分解（算法本体） | Planner 适配层内实现 | `ros2_adapter/uav_spf_planner/` |
| VLM 模型执行与云端协议 | Inference Function / Cloud alternative | `src/basic_function/inference/`（算力角色） |
| SPF prompt 与点/深度解析 | Navigation Tool 算法适配 | `ros2_adapter/uav_spf_planner/prompt.py` |
| 轨迹执行 | 原生扩展 Behavior（复用，不新建） | `src/behavior/follow_path/` FollowTimedPath |
| 起飞/降落 | 原生 AS2 Behavior | TakeoffBehavior / LandBehavior |
| 图像/状态/时钟供给 | 平台与飞控板角色 | flight_board 传感器桥 / 时钟桥 / AS2 self_localization |

不设 `ros1_proxy/`：上游无任何 ROS 进程。
旧 `vlm_proxy` ROS 包已移除。云端也是 Inference 的执行 alternative；是否有本地
权重不决定归属。Tool 不管理 API key、URL 或供应商协议，也不把导航 prompt/点解析
下沉为通用模型引擎的规则。

## 3. 最小数据/调用链（AS2 段目标形态）

```
mission.py(工作站) --NavigateToSemanticGoal{description,timeout}--> uav_spf_planner(板端)
uav_spf_planner --InferVisionLanguage{model_name,prompt}--> Inference Function(算力角色)
Inference Function --公共 HTTP runtime / Cloud plugin--> 云端 VLM API
Inference Function --text + actual image header--> SPF Tool 解析 point/depth
uav_spf_planner --TimedPath(2~3点修订)--> FollowTimedPath(板端 AS2)
FollowTimedPath --Motion Reference--> AS2 控制器 --> PX4(flight_board) --> AirSim
```

图像由同命名空间的 Inference Function 消费，不经过 SPF Tool 转发；Function 可部署在
PC 算力角色，负责向已授权云端发送图像。工作站只调用任务与做仿真评分。

## 4. 复用决策

| 部分 | 决策 | 理由 |
| --- | --- | --- |
| FollowTimedPath | 每轮独立 is_final goal | 单步前飞+时长语义可完整表达为 2~3 点时间化轨迹，不丢中间语义 |
| yopo adapter 生命周期模式 | 模式参考（单任务占用/revision/停止门控） | 同为本仓库 TimedPath 消费者；但不引入其 0.1 s 推理预算（SPF 为秒级云推理） |
| 上游 controller/projector/drone_space 算法逻辑 | 语义逐字迁移到 adapter，接口重写 | 输入从 AirSim RPC 换 ROS 话题、输出从速度指令换 TimedPath |
| 上游 VLM prompt 与响应解析 | 保留于 SPF Tool | 复现保真；复制原因在文件头声明 |
| airsim/djitellopy/mss/pynput 依赖 | 不迁移 | 属上游平台直控与屏幕截图路径，AS2 段由平台能力替代 |
| SDK（google-genai/openai） | 不依赖，Cloud plugin 使用 stdlib HTTP | 云端调用与认证集中在 Inference，ROS 与 worker 环境隔离 |

## 5. 接口契约（阶段 2 实测后锁定，当前为设计值）

- Tool 输入：`<vehicle_ns>/self_localization/pose|twist`、CameraInfo、语义目标；
  RGB 图像由 Inference Function 直接消费，使用实际图像 header。
- 调用：`inference/vision_language/infer`（共享 InferVisionLanguage Action）；
  `follow_timed_path`（每轮独立 send_goal、stop）。状态：`spf/status`。
- 时间：VLM 结果绑定实际消费图像的采集 stamp；SPF 常量 `RESULT_MAX_AGE_S=30.0`；`use_sim_time` 经 `/clock`。
- 坐标系：机体系位移经当前 yaw 旋转到 `<vehicle>/odom` ENU；TF 仅取旋转（D6）。

## 6. 多机外溢声明

Function 的相机缓存与 Action 按 namespace 隔离；多机部署及资源调度仍未验收。**本期仅单机
验收，不声称多机能力**；机群级语言任务编排属 Mission 层，不在本 Planner 范围。

## 7. 待核查条件（进入阶段 1/2 前必须闭环）

0. **许可证处置决定**：[已裁决 2026-11-12，用户选择] 按 ③ 执行——保持逐字保真方案，
   仅限组内非商业研究使用，不对外分发、不公开演示上游源码副本；本 README 顶部的
   Proprietary 声明为永久保留项。若将来需对外发布或商业相关用途，必须回到 ① 书面
   许可或 ② 净室重实现。风险承担方：aerohub 团队（用户决定）。
1. ~~子模块锁定提交与四项关键事实一致性抽查~~ [通过，2026-11-12]
2. ~~flight_board CameraInfo 实测~~ [完成 2026-11-13]：256×256、fx=fy=128、
   hfov=vfov=**90.0°**、无畸变、frame `drone0/FrontCamera_optical`；
   tf_static 为纯旋转+杆臂 X=1.0。D3 失配实锤与量化见
   `log/spf/contract/result-20261113.md`。Demo 需 1280×720 变体（D4）不变。
3. 云端 API 出口：OpenAI 兼容端点选定与 key 供给。[已定智谱 GLM-4V-Flash（免费、国内直连，
   偏离 D11）；OpenRouter 论文模型路线保留为需充值时的对标选项]
4. RoboPi 板端 rclpy 环境下纯 stdlib 实现的 import 闭包审计（阶段 6）。


## Cloud Function 配置与验证

复制 `config/inference.cloud.example.yaml` 为被忽略的 `config/inference.cloud.local.yaml`，
声明 `models.spf.cloud` 并填写 base_url/model/api_key，可用 api_key_env 引用环境变量；
SPF 始终调用逻辑 model_name=`spf`。多引擎通过 active_engines.spf 显式选择，缺省不支持。
环境与部署细节见 [Vision-language Function](../../../../basic_function/inference/models/vision_language/README.md)，
验证边界见 [Cloud 迁移验证](../../../../../deployment/airsim/demos/spf_language_nav/cloud_inference_validation.md)。
