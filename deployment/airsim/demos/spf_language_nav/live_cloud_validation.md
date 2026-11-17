# SPF 真实 Cloud 链路回归（2026-11-17）

最新：用户授权将测试结果门限配置为 30 秒后，一个真实导航执行用例通过：
60 秒任务中 5 次推理尝试、1 个完成周期，水平位移约 1.9 米，timeout/停止/原生
Land/disarmed/无新增碰撞均通过。仍有 4 次 empty_response，不能据此声明稳定导航
或完整复现成功。配置及实际参数读回见 `sitl-result-age-30/spf-test.yaml` 与
`effective-result-timeout.txt`，实际结果见 `formal-timeout/result.json`。

以下保留此前 6 秒配置的结论：真实供应商、跨容器 Function 与真实 AirSim 相机消费通过；正常导航运动未通过。
不声明完整迁移验收或复现成功。证据根目录 `log/spf/live-20261117T085008Z/`，
预声明矩阵与过程见 `skill/temp/spf-live-regression/README.md`。

## 配置与环境

用户实际配置已原样保存到 ignored `config/inference.cloud.local.yaml`，示例密钥清空。
逻辑 `model_name=spf`，引擎 cloud，供应商模型 deepseek-flash；没有换模型或放宽
6 秒结果年龄门。无本地 SPF VLM 实现；本轮不涉及 CUDA/Houmo 算法验收。
私有配置不进入证据哈希或日志，扫描证据未发现实际密钥。

固定镜像 onboard_computer:v0.1.1 / flight_board:v0.1.1 / ROS Humble workstation，
外部 ROS overlay 与锁定 cloud-env；不重建镜像。Linux 有屏 AirSimNH + PX4 SITL，
算法与 AS2 在 compute，Inference 独立容器，任务仅在 workstation。镜像、来源及
启动配置见各 run 的 topology/source-manifest 与根 source-hashes。

## 实际结果

| 项 | 结果 | 证据 |
| --- | --- | --- |
| M1 小图真实 API | 通过，1.98s，识别红方块 | m1-real-api.json |
| M2 SPF 原 prompt 与解析 | 通过，4.23s，合法 point/depth | m2-real-spf-prompt.json |
| M3 两容器生产 Function → 实际 SPF 消费者 | 通过，2.94s，合成图像；仅接口/数学轨迹证据 | m3/result.json |
| M4 真实 AirSim RGB → Function → Cloud → SPF | 通过，3.35s；1280×720、真实内参与 header，地面未解锁 | m4/m4-ground.json、ground-fpv.jpg |
| M5 正常 timeout 飞行 1 | 运动未通过：15s、2 次尝试、零完成周期，STALE_RESULT；timeout/停止/降落/disarmed/无新增碰撞通过 | sitl-ground-02/formal-timeout/result.json、compute.txt、assessment-timeout-01.json |
| M5 正常 timeout 飞行 2 | 未通过：30s、3 次尝试、零完成周期；两条 STALE_RESULT；收口与无新增碰撞通过 | sitl-timeout-02/formal-timeout/result.json、compute.txt |
| M5 取消 | 通过：推理等待中 CANCELED、stop_confirmed、Land/disarmed、无新增碰撞；不是运动中取消 | sitl-cancel/formal-cancel/result.json |
| M6 stale / Function 掉线注入 | 本轮真实平台未执行；已有本地协议与生命周期测试不替代它们 | 旧本地证据见 cloud_inference_validation.md |

正常用例指令固定为沿道路中央前飞避障；spawn NED(0,0,0)，评分目标 ENU(0,10,1.5)
仅用于记录，不是语义到达判据。两轮均保留原模型；新场景重启后仍发生结果过期，
说明地面单次接口通过不足以证明飞行可用性。未以延长结果有效期、改 stamp 或换模型
消除失败。没有证据证明转向修复后的轨迹已在本轮飞行中执行。

首次 mission 的原始 success=true 只覆盖生命周期门，不能代表正常运动通过；已保留
原文件并附独立失败判定，后续入口新增 trajectory_execution 门要求正常用例至少完成
一次实际执行。第一轮 observer 日志写入遇到 root 目录权限问题，修正为宿主先建目录，
第二轮日志成功保存。

## 修复与限制

- SPF 上游右转角是 NED 顺时针为正，ENU 适配应减去该角。独立右/左及非零航向测试
  先红后绿，连同既有轨迹共 8 项通过；上游 gitlink 与源码不改。
- Linux 入口复用共享 manager、启动 relay、固定镜像与 per-run workstation overlay；
  不再调用 Windows 启停命令。清理核验资源归属及解除武装，不触碰其他会话。
- sensor 容器继承的 supervisor healthcheck 显示 unhealthy；实际启动独立 ROS 节点，
  不经过该 supervisor。实际输入由下游验证，但部署健康检查仍有缺口。
- 有屏场景及俯视 track/帧已保存；零导航运动不能作为移动质心跟踪能力通过的证据。
- 真实飞行结果缺少逐请求供应商耗时拆分；现有证据能确认结果超过年龄门，尚不能把
  全部耗时归因于供应商。后续需量化图像初始年龄、编码、HTTP、ROS 往返和时钟进度。
- 完整指令/重复矩阵、实际运动中的取消与转向、stale/掉线恢复及板端验收仍未完成。

本次回顾更新平台传感器适配 Skill 的 NED/ENU 转向核查，以及端到端验证 Skill 的
云端分层、freshness 与“收口通过但零执行”的判定；未创建新 Skill。

## 30 秒门限配置

该次实验使用 `config/spf_language_nav.yaml` 的 `max_result_age_s`（历史配置）。同一参数传到
Function Action 与 Tool 结果复核；请求 timeout 与任务 timeout 独立。启动时快照配置，
不改旧图像 stamp。新实验保持相同模型/指令/执行保护，任务预算预先设为 60 秒。
先前三次已返回结果变为可见的 empty_response，第四次形成实际执行；最后一次也为空。
目前仅确认执行链可用，模型返回内容不稳定的根因未诊断。旧的过期日志不能追溯
推断为有效模型输出，也不能被新用例覆盖。

后续按用户要求收回 SPF 控制：`vlm_client.py` 的 `RESULT_MAX_AGE_S=30.0` 是当前唯一来源，
已移除该配置文件、ROS 参数及启动快照逻辑。上面的飞行证据对应修改前的同值配置版本，
不能称为常量版本重新执行了飞行。
