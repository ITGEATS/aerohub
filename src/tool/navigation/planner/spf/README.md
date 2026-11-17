# SPF（See, Point, Fly）Planner

上游：https://github.com/Hu-chih-yao/see-point-fly （CoRL 2025，training-free VLM 无人机语言导航）
子模块锁定提交：`5621bcf43e9826d60df014541dd0498e743a92bd`（2026-01-29 tip；四项关键事实抽查
与已删除 zip 快照全部一致，FOV 行 L30-31、prompt L111、duration L122、requires-python>=3.13）

> **⚠️ 许可证合规声明**：上游 LICENSE 为 **Proprietary Software License**（非开源协议），
> 明文禁止未经书面许可的复制、修改、分发、衍生作品与托管，仅允许非商业使用，副本须保留
> 本声明。本仓库以 **git 子模块引用**（仅存 gitlink，不复制源码入库）方式使用上游；
> 适配层中的算法逻辑迁移与 prompt 复制的合规边界见 §8，许可咨询邮箱：
> tommy60718.en10@nycu.edu.tw / jj2095813@gmail.com。

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
  调用面保持 `spf/goal`(String JSON)+`spf/cancel`。

## 2. 职责映射与目录归属

| SPF 部分 | aerohub 层归属 | 目录 |
| --- | --- | --- |
| 语言指令 → 每步位移决策 | Planner（Navigation Tool 实现基础） | `src/tool/navigation/planner/spf/` |
| 反投影 + 动作分解（算法本体） | Planner 适配层内实现 | `ros2_adapter/uav_spf_planner/` |
| 云端 VLM 调用 | 外部推理依赖，代理节点承载 | `vlm_proxy/uav_spf_vlm_proxy/`（PC 侧） |
| 轨迹执行 | 原生扩展 Behavior（复用，不新建） | `src/behavior/follow_path/` FollowTimedPath |
| 起飞/降落 | 原生 AS2 Behavior | TakeoffBehavior / LandBehavior |
| 图像/状态/时钟供给 | 平台与飞控板角色 | flight_board 传感器桥 / 时钟桥 / AS2 self_localization |

不设 `ros1_proxy/`：上游无任何 ROS 进程。
VLM 代理不放 `src/inference/`：该模块面向本地模型权重与独立 uv 环境；SPF 无权重，
是纯外部 API 调用，归属 Planner 自身目录（依据开发者贡献指导的推理依赖不改变源码
归属原则，与 YOPO 的上游留在 `src/planner/yopo/` 同理）。

## 3. 最小数据/调用链（AS2 段目标形态）

```
mission.py(工作站) --spf/goal{instruction}--> uav_spf_planner(板端)
uav_spf_planner --query{request_id,instruction,image_stamp}--> uav_spf_vlm_proxy(PC)
uav_spf_vlm_proxy <--result{point,depth,label}-- 云端 VLM API
uav_spf_planner --TimedPath(2~3点修订)--> FollowTimedPath(板端 AS2)
FollowTimedPath --Motion Reference--> AS2 控制器 --> PX4(flight_board) --> AirSim
```

图像数据流止步于 PC（传感器桥→代理），板端仅收发 <2 KB/s JSON；公网访问仅代理出板。

## 4. 复用决策

| 部分 | 决策 | 理由 |
| --- | --- | --- |
| FollowTimedPath + 修订/确认时序 | 直接复用 | 单步前飞+时长语义可完整表达为 2~3 点时间化轨迹，不丢中间语义 |
| yopo adapter 生命周期模式 | 模式复用（generation/revision/停止门控） | 同为本仓库 TimedPath 消费者；但不引入其 0.1 s 推理预算（SPF 为秒级云推理） |
| 上游 controller/projector/drone_space 算法逻辑 | 语义逐字迁移到 adapter，接口重写 | 输入从 AirSim RPC 换 ROS 话题、输出从速度指令换 TimedPath |
| 上游 VLM prompt 与响应解析 | 逐字复制进代理，不改一字 | 复现保真；复制原因在文件头声明 |
| airsim/djitellopy/mss/pynput 依赖 | 不迁移 | 属上游平台直控与屏幕截图路径，AS2 段由平台能力替代 |
| SDK（google-genai/openai） | 不迁移，纯 stdlib HTTP 等价实现 | OpenHarmony 无 pydantic-core Rust 轮子（偏离 D2） |

## 5. 接口契约（阶段 2 实测后锁定，当前为设计值）

- 输入：`<vehicle_ns>/self_localization/pose|twist`；`sensor_measurements/FrontCamera/rgb/image`
  + `camera_info`（几何校验与 stamp 对齐）；`spf/goal`（String JSON）；`spf/cancel`（Trigger）。
- 输出：`follow_timed_path` Action（首段 send_goal、滚动 modify、stop）；`spf/vlm/query|result`；
  `spf/status|events`。
- 时间：VLM 结果绑定请求时图像 stamp；`max_result_age_s=6.0`；`use_sim_time` 经 `/clock`。
- 坐标系：机体系位移经当前 yaw 旋转到 `<vehicle>/odom` ENU；TF 仅取旋转（D6）。

## 6. 多机外溢声明

`spf/goal` 与 VLM 代理接口按 `<vehicle_namespace>` 复制即可逐机独立运行；但代理的
图像缓存与 request_id 命名空间须按 vehicle 分帧隔离（本期不实现）。**本期仅单机
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
