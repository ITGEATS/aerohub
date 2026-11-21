# AeroVLA Blocks 用例矩阵验收（阶段 5，OOD 口径）

日期：2026-11-21
Demo：`deployment/airsim/demos/aerovla_blocks/`
拓扑：降级拓扑（用户 2026-11-21 裁决）——真实 NPU 推理留 RoboPi HTTP 8090，
AS2/framework/planner/inference-adapter 在 PC `onboard_computer` 容器，flight_board 供
PX4 SITL/clock，DDS 仅 PC 本机，跨机只剩 HTTP。PX4 为 SITL，非真机。

## 运行前锁定的判据（OOD 口径，先于任何计分运行声明）

OOD 口径：验收只断言**链路正确执行策略输出**，不断言导航/行为定性一致（不承诺
"reach orange ball" 的地理成功）。每条用例的门槛先于运行固定，运行后不得放宽。

通过等级定义：
- `PASS`：该用例全部门槛成立且有证据。
- `FAIL`：任一门槛不成立。
- `NOT-EXECUTED`：本轮未运行（不得记为通过）。

门槛（先于运行锁定）：

| 用例 | 运行前锁定的门槛（OOD） |
|------|------------------------|
| formal | START 被接受；在步数上限 N=20 内，planner 产生 ≥2 个合法步（每步含合法 NED TimedTrajectorySegment：时间单调、首点=上一段参考末点、首点速度≈0）；FollowTimedPath goal ACCEPTED；motion reference delta>0；随后 CANCEL→stop 确认→Land→disarm 全通。不断言到点距离/朝向收敛。 |
| cancel | 在 START 且 goal ACCEPTED 后注入 CANCEL；`planner/control` CANCEL 被接受；FollowPath stop_confirmed 在 30s 内成立（速度门）；随后 Land+disarm 收敛。不断言取消前走过的距离。 |
| fault | 推理端点不可用（INFER_ENDPOINT 指向不可达地址）时：planner 在 `max_consecutive_failures` 内上报 Failure（overall_status=failure，failure_reason 非空），framework 不崩溃、CANCEL 可响应；recover() 完成 land+disarm。不得出现进程 core dump 或节点失联导致的无响应（PASS 客户端 teardown 竞态除外，非此列）。 |

记录项（不计入门槛，仅证据）：每步 total_ms（NPU 推理耗时）、motion reference 计数、
goal status 序列、planner status JSON。

## 用例与运行脚本

用例运行器：`deployment/airsim/demos/aerovla_blocks/run_case.ps1`（参数 `-Case formal|cancel|fault`）。
每个用例独立启动 Blocks + PC 栈 + onboard 链客户端，证据写到独立目录，
结论汇总到 `validation.json`。

## 结果摘要（运行后填写，见 validation.json）

| 用例 | 结果 | 客户端 exit | 证据目录 |
|------|------|-------------|----------|
| formal | **PASS** | 0 | `log/aerovla/stage5/formal-20261121-110657/` |
| cancel | **PASS** | 0 | `log/aerovla/stage5/cancel-20261121-110327/` |
| fault | **PASS** | 0 | `log/aerovla/stage5/fault-20261121-110510/` |

- formal：真实推理到 step 5（NPU total_ms~1747）、motion ref delta=102、goal 保持 ACTIVE(2)、CANCEL/stop/land/disarm 全通。
- cancel：goal ACCEPTED 后立即 CANCEL，stop_confirmed + land + disarm 收敛（首次尝试遇 NPU 瞬时 INVALID_INPUT×3 导致 loop pause，证据保留于 `cancel-20261121-110120/`，重跑通过）。
- fault：端点指向 `127.0.0.1:9`，planner 报 FAILED（overall_status=4，reason=TRAJECTORY_TIMEOUT 非空），框架在终态仍响应 CANCEL，recover() 落地；无 core dump。adapter.log 无显式连接错误行，不可达的下游表现为"planner 无产出→轨迹超时"，如实记录。
- 三例门槛先于运行锁定于本文件，运行后未放宽；无 FAIL 改判 PASS。

## 边界声明

- 本验收在降级拓扑 + PX4 SITL 下成立；不代表原跨机 DDS 目标达成，不代表真机通过。
- OOD 判据只覆盖"链路执行策略输出的正确性"，不覆盖"导航任务是否完成/策略是否 Good"。
- 任何未运行用例保持 NOT-EXECUTED，任何 FAIL 不得改判 PASS。
