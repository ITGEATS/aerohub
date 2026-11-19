# SPF 语言导航 Demo（spf_language_nav）

算法组件：`src/tool/navigation/planner/spf/`（契约与状态见其 README）。
当前接口和使用见本 README 与算法 README；最新实测见
[真实链路回归](live_cloud_validation.md)，[validation.json](validation.json) 保留旧契约历史证据。

## 拓扑（WSL2 适配四方）

| 角色 | 实际运行 |
| --- | --- |
| 仿真器 | Windows 宿主 Blocks.exe（本 Demo settings 变体写入 Documents\AirSim，原文件自动备份） |
| 飞控板 | WSL2 dockerd：flight-board v0.1.1 `vehicle` 容器（PX4 SITL + Agent，传感器关闭）+ `clock` 容器 |
| 传感器桥 | WSL2：flight-board 镜像独立 sensor 容器（FrontCamera 1280×720@10Hz，rpc 指向 Windows 宿主 IP） |
| 算力板 | WSL2：onboard-computer v0.1.1 `compute` 容器（AS2 + FollowTimedPath + `uav_spf_planner`） |
| 算力 Function | WSL2：独立 `inference` 容器（固定 onboard 镜像 + 外部 worker 环境，通用视觉语言 ROS Adapter / Cloud plugin） |
| 工作站 | WSL2：`mission.py` 一次性 ROS 任务容器；不运行 SPF 私有推理代理 |

与 Linux 原生入口的偏离（记录于 skill/temp/spf-migration）：
Blocks 经 Windows interop 启停而非 manager.py；**每次实验 Blocks 必须先重启，PX4
是第一个 TCP 客户端**（单客户端槽位）；`AIRSIM_HOST` 用 WSL 默认网关 IP。

## 运行（全部在 WSL Ubuntu-22.04 内）

```bash
cd <仓库目录>
cp config/inference.cloud.example.yaml config/inference.cloud.local.yaml
# 编辑 models.vlm.cloud 的 base_url/model/api_key；或配置 api_key_env
python3 deployment/airsim/demos/spf_language_nav/run.py prepare --output log/spf/pc-e2e/sitl-01
python3 deployment/airsim/demos/spf_language_nav/run.py start   --output log/spf/pc-e2e/sitl-01
python3 deployment/airsim/demos/spf_language_nav/run.py mission --output log/spf/pc-e2e/sitl-01 \
    --case ground --instruction "fly toward the orange ball" --navigation-timeout 180
python3 deployment/airsim/demos/spf_language_nav/run.py stop    --output log/spf/pc-e2e/sitl-01
```

故障用例：`--fault cancel|stale|vlm-down`（mission 运行中由 run.py 注入并保留恢复证据）。
评分目标 `--target` 默认 OrangeBall 真值 (34.25, 33.10, -3.68)，仅用于打分。
`start --observe` 启用质心随动俯视观察视图（复用 `deployment/swarm/` 基础设施；
mission 期间 observer 容器记录 `overhead/track.jsonl` 与俯视帧作为随动证据；
健康检查仅证明相机连通摆放，随动成功只以任务期质心序列移动为准）。

## Linux 原生入口

`run.py` 自动识别 Linux/WSL，亦可设置 `SPF_SIMULATOR_PLATFORM=linux|wsl`。
Linux 复用共享 AirSim manager 与启动 relay；通过资源配置或 `SPF_AIRSIM_BINARY`
选择场景，使用 `DISPLAY` 和 `AIRSIM_RENDER_MODE=windowed` 保留有屏渲染。
复用现有 overlay 可设置 `SPF_ROS_OVERLAY`（容器内路径）；所有角色仍使用固定镜像。
宿主需能导入 RPC 的 msgpack，仓库环境可使用
`PYTHONPATH=$PWD/deployment/robopi/vendor`，无需向系统 Python 安装依赖。
换场景时须显式指定 `--instruction/--spawn/--target`，不沿用 Blocks 的 OrangeBall 评分目标。
每个独立用例采用新的 output，先确认降落/disarmed 再 stop；不清理其他会话资源。

## Inference 下层

SPF 只配置 `model_name`，prompt 与点/深度解析留在 Tool；图像编码、模型路由、云端
认证和协议进入 Inference Function。直接在被忽略的
`config/inference.cloud.local.yaml` 声明 `models.vlm.cloud`，填写 `base_url/model/api_key`，
或通过该条目的 `api_key_env` 等字段引用环境变量。对外模型名固定为逻辑 `spf`。
只声明一个引擎时直接使用；多个时填写 `active_engines.spf`。缺少声明或选择不明确会失败，
不会自动启用 Cloud，也不会将供应商模型名当作逻辑模型名。不强制文件权限、
交互向导或额外密钥目录；请勿提交或打印密钥，建议限制本地文件权限。

`prepare` 在固定镜像中为 worker 创建外部 `cloud-env`；源码只读，构建输入先复制到
容器临时目录，避免 setuptools 向仓库写入元数据。不重建共享镜像。可用
`SPF_COMPUTE_IMAGE` / `SPF_FLIGHT_IMAGE` 显式复用本机已有固定版本镜像。
配置或凭证缺失时 `start` 在启动仿真/飞控前失败。准备本身不调用 API。
真实云端和真实 AirSim RGB 链已通过；新链飞行暂未通过轨迹执行门，见 [真实链路回归](live_cloud_validation.md)。

## 当前 Navigation 接口

任务调用 `/<vehicle>/navigate_to_semantic_goal`（`NavigateToSemanticGoal`），输入
`description` 和 `timeout`。不再通过轮数宣告成功：SPF 暂无到达判定，正常预算耗尽
返回 `TIMEOUT/success=false`；取消与真实故障仍可提前结束。`--navigation-timeout`
设置该预算，`--timeout` 只限制工作站等待时间，应为停止留出至少 15 秒余量。
Demo 的 `navigation_contract` 门控验证预期 TIMEOUT/CANCELED 等返回和停止确认，
**测试通过不代表导航目标达成**。下方旧轮数门控与飞行记录保留为历史，不能验证当前接口。
本次验证见 [semantic_contract_validation.md](semantic_contract_validation.md)。

`run.py prepare` 同时重新生成工作站的共享导航接口，不能复用旧 Action overlay。

## 历史验收状态（旧 topic/轮数契约）

**部分验证通过（最小端到端验收，2026-11-16）**：M5=1 指令 ×1 完整实例
3 循环连续、无碰撞、task_complete 收口（`run-20261116z6/formal1`）；
M6=cancel 注入正确收口（`run-20261116z7/cancel1`；stale/vlm-down 未执行）；
M7=质心随动证据（track.jsonl 质心移动 30.5m + 俯视帧目视居中确认）；
M8=算力角色 onboard-computer 容器完成 + import 闭包审计
（`log/spf/pc-e2e/container-acceptance-20261116.json`）。
详见 [validation.json](validation.json)。**不声明"复现成功"**（全矩阵
3 指令×3 实例、逐步误差统计、stale/vlm-down 注入、板端部署未执行）。

**合并 dev 后复验（2026-11-17，基线 8d2226e+183ff7a，共享接口/D13/上游
follow_path）**：链路级复验全部通过——5 个时间化段执行无连续性拒绝、
D13 共享码表生效（cancel 路径 CANCELED、失败路径 EXECUTION_FAILED+detail+
stop_confirmed）、碰撞门控与观察视图正常、cancel1 完整通过
（`run-20261117e/cancel1`）；**M5 三循环完整实例在新基线未通过**
（3 次尝试分别止于循环 2/3，均为替代模型连续失败，聚合 parse 5/26≈0.19
与 D11 已知水平一致；缺口如实记录于 validation.json，重跑留作后续）。

已知运行约束（证据与修复链见 validation.json attempt_history）：
同一 Blocks 会话仅第一个 mission 可靠 arm（teleport 后 PX4 GPS drift
preflight 拒锁数分钟）——每个用例独立 start→mission→stop；
该 Blocks 构建无 `simGetCameraPose`/`resetLastCollision` RPC，碰撞检测用
`time_stamp` 比对；触地/粘滞标志不计入导航期碰撞门控。

## 历史 Blocks 实验限制

- 智谱免费档延迟与 JSON 稳定性（D11）：闭环指标解释力受限，论文数值不可对标；
  重试退避 5s/连续失败上限 6 为容错配置。
- Blocks 场景与论文 TravelUAV 差异：目标语义（orange ball）替代 crane 类指令；
  橙球贴地导致 VLM 语义深度下降指令蹭地，执行层加高度下限 D12（min_altitude_m 1.2）。
- 单机；多机外溢声明见 `src/tool/navigation/planner/spf/README.md` §6。
- 当前生命周期测试入口为 `ros2_adapter/tests/test_semantic_navigation_ros.py`；旧实验当时没有独立测试。

## SPF 结果有效期

SPF 客户端在 `vlm_client.py` 使用常量 `RESULT_MAX_AGE_S = 30.0`（秒）。
同一值传给 Inference Function 并用于客户端复核，不再暴露配置项或 ROS 参数。
`vlm_timeout_s` 控制单次请求等待，`--navigation-timeout` 控制整个任务预算，两者独立。

## 当前待办与验证缺口

- 模型空响应：30 秒有效期实验中 5 次请求仅 1 次形成执行，4 次 empty_response。
  需记录脱敏的供应商 finish_reason/用量和分阶段时延，诊断原因；不能直接认定为网络或 token 上限。
- 验收：真实平台 stale/Inference 掉线恢复、运动中取消、左右转向与多指令重复可靠性
  尚未完成。完整 AS2 DroneInterface 的 NavigationModule 加载调用也未实测。
- 当前常量版本通过 ROS 回归；已有单周期飞行使用相同 30 秒值的前一配置版本。
  无语义到达检测是已确认的 timeout-only 契约；本地 SPF CUDA/Houmo 尚未实现，均不伪装为已支持。

任务指标 `cycle_completion_ratio` 是完成执行周期数/推理尝试数，不等于模型解析成功率；
旧证据中的 `parse_ratio` 保持原样，新入口已更正名称。

## 健康检查与退出修复（3c3cde9 后续）

sensor 入口通过轻量 wrapper 复用固定镜像的原 SensorNode；RGB 和 CameraInfo 完成
发布并出现新的采样 stamp 后才更新健康状态，5 秒没有新采样则失败，正常退出清理状态。
Docker 使用专属检查，原镜像不重建。它验证本节点发布进展，跨容器实际消费仍须另测。
实际 AirSim 地面验证 healthy → 暂停采集 unhealthy → 恢复 healthy 已通过，未解锁。

固定 Humble 的 MultiThreadedExecutor.shutdown 未 join Python 回调线程池，排队订阅
可能在 destroy_node 后执行。Function 的退出桥接先停止 spin、等待线程池，再销毁实体；
版本相关的内部字段访问集中在 executor_shutdown.py。测试捕获异步 Future 异常并断言
为零，不靠忽略警告。原复现 21 次产生 1 个 InvalidHandle，修复后同一复现 21 次为零。
证据：`log/spf/inference-cloud/exit-warning-red.txt`、`exit-warning-final-02.txt`，
以及 `log/spf/live-20261117T085008Z/sitl-health-fix/sensor-health-regression.json`。
