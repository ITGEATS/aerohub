# uav_citynav_planner（CityNavAgent 算力板侧 ROS 2 planner，迁移方案阶段 5）

`src/tool/navigation/planner/citynav/ros2_adapter/`，ament_python 包，运行于
onboard-computer 容器。依赖闭包 = rclpy + numpy/scipy + stdlib（容器 v0.1.1
py3.10 实测无 networkx，底层图用包内最小实现，见 D24）；
**禁 torch/cv2/sklearn**：grounding 与 CLIP 推理走 Inference HTTP 服务（阶段 4 /
并行任务），PNG 编解码用纯 Python zlib writer，统计滤波用 scipy cKDTree 重写
（与 sklearn `NearestNeighbors(n_neighbors=k).kneighbors` 语义等价：k 近邻含
自身、逐点 mean、全局 mean/std、阈值 `mean + multiplier·std`；低维精确 kNN
两实现结果一致）。

契约主文档：`../README.md` §5 与 `log/citynav/contract/result-20261117.md`
（深度 32FC1 米 axial / 内参 CameraInfo 实测 / 五目批次新鲜度 / 动作映射锁定值）。
本包实现其中 planner 侧全部约定；接口字段名不得偏离契约。

## 模块

| 文件 | 职责 | 逐字/等价移植来源 |
| --- | --- | --- |
| `perception.py` | 五目批次 mask 像素反投影 odom ENU（optical→FLU 静态旋转 D6→位姿）、`isfinite & 0<d<50 m` 过滤、cKDTree 统计滤波、类均值 + alpha=0.6 收缩 + `min_altitude_m` 下限 | `upstream/utils/maps.py` L258-339、L53-67；`SimRun.py` L292-306 |
| `hspm.py` | landmark 抽取（op=landmark，payload 仅 instruction）→ viewpoint 路线查询（op=viewpoint，payload 结构化字段 + view_stamps，图像与 prompt 由代理构造）→ 解析 → 选中视角 grounding → 点云 → waypoint；is_found 可见即推进、卡死判据逐字保留；本包保留 `parse_viewpoint_response_v2` 逐字复制（tests AST 比对 upstream 防漂移） | `SimRun.py` L227-321、L534；`prompt_builder.py` L30-54；`utils/landmark_extraction.py` L56-65 |
| `memory.py` | 在线拓扑图（节点=位置+front/back 两图 bytes+可选文本；边=欧氏距离；20 m 合并）、包内最小无向图 `UndirectedGraph`（D24 替代 networkx）、JSON 落盘/加载、`find_closest_node`、`dijskra_transform` 逐字移植、LM-Nav 搜索与 exploit | `maps.py` L10-31；`external/lm_nav/navigation_graph.py` L45-58、`optimal_route.py` L17-102、`pipeline.py` |
| `trajectory.py` | `calculate_movement_steps` 逐字移植（15°/5 m/2 m，分解顺序=时间顺序）→ FollowTimedPath 分段（偏航段 t=Δyaw/yaw_rate 且 15° 子步合并单段、水平段 t=d/v、垂直段 vz=v·dz/dxy 比例+下限保护） | `utils/utils.py` L195-277；契约 §5 |
| `llm_client.py` | `citynav/llm/query|result` ROS 话题客户端（权威契约字段、request_id 匹配、`max_result_age_s=90`、超时） | 代理侧契约（`llm_proxy/.../node.py` L13-17）；仿 `spf/ros2_adapter` vlm_client |
| `grounding_client.py` | `POST /models/grounded_sam/greedy_mask_predict`（字段名锁定：image_b64/text_prompt/box_threshold/text_threshold → mask_b64/seg_success/phrases）+ 纯 Python PNG 编解码 | grounded_sam README 服务契约 |
| `clip_client.py` | `POST /models/clip/score`：`{"images_b64","texts"} → {"similarity","model_version"}`（并行任务建设，按固定契约实现；服务端 ≤16 图/请求，客户端分块） | 任务规格 + `models/clip/schemas.py` |
| `node.py` | 状态机 IDLE/RUNNING/STOPPING/STOPPED/FAULT（仿 spf）+ generation/revision 时序、`citynav/goal`（String JSON `{instruction,max_steps,use_memory}`）、`citynav/cancel`（Trigger）、`follow_timed_path` Action 客户端、`citynav/status|events`、`citynav/llm/query|result` 转发；订阅 5×(rgb/depth_planar/camera_info)+`self_localization/pose|twist`；批次新鲜度 `view_freshness_s=2.0`、连续失败 3 次 FAULT、卡死判据升级 FAULT 收口 | spf/yopo adapter 模式 |

参数默认值（任务规格）：`base_velocity=1.0, base_yaw_rate=30.0, min_altitude_m=1.2,
alpha=0.6, merge_radius=20.0, max_steps=60`；另有 `view_cameras`（五目阵相机名，
顺序对应 left/slightly_left/front/slightly_right/right，默认
`LeftCamera,SlightLeftCamera,FrontCamera,SlightRightCamera,RightCamera`）、
`view_yaws_deg`（ROS 逆时针约定，默认 `90,45,0,-45,-90`）、`grounding_url`、
`clip_url`、`memory_graph_path`（默认 `citynav_memory.json`，跨任务复用 D4）。

## LLM 话题 JSON 契约（权威：uav_citynav_llm_proxy，node.py L13-17 定型）

- query（`citynav/llm/query`，std_msgs/String JSON）：

  | 字段 | 类型 | 说明 |
  | --- | --- | --- |
  | `request_id` | str | uuid hex，result 原样回带 |
  | `op` | str | `"viewpoint"` \| `"landmark"` \| `"caption"` |
  | `instruction` | str | 导航指令（三 op 通用） |
  | `landmarks` | list[str] | 全部 landmark 序列（viewpoint/caption 用） |
  | `traversed` | list[str] | 已 traversed 前缀（viewpoint 用） |
  | `next` | str | 下一 landmark（viewpoint 用） |
  | `view_stamps` | dict[label, int ns] | 各视角请求时图像 stamp（label=left/…/right，代理按此从其相机缓存取帧） |
  | `max_age_s` | float | 帧龄包络（默认 90 s=D21） |

  **prompt 与图像都不由 planner 发送**：代理按 `view_stamps` 从自身相机缓存取帧
  并内部构造上游同构消息（route_planning_prompt_builder /
  landmark_extraction_messages 均在代理包逐字复制）。
- result（`citynav/llm/result`）：`{"request_id","op","ok","text","error_code",
  "latency_s"}`；`error_code ∈ ok|stale|timeout|http|parse|busy`（`text` 为模型
  原始文本；viewpoint 的 JSON 字段解析在 planner `hspm.parse_viewpoint_response_v2`
  完成，代理仅校验可解析性）。
- 跨包不互相 import：planner（onboard-computer 容器）与代理（工作站容器）独立
  构建安装；共享源段以各自逐字复制 + 测试期 AST 比对 upstream 锁定提交防漂移
  （`tests/test_prompts_alignment.py`）。

## 偏离记录（本包新增，编号接契约文档）

- **D22（动作图=记忆图本身）**：上游 exploit 用作者 pkl 动作图
  （`mem_graphs/{env}.pkl`）经 `calculate_movement_steps_mem` 逐边分解；本期
  在线建图（D4）无独立动作图，`memory.exploit_waypoints` 直接取 walk 相邻节点
  位置逐段 `calculate_movement_steps` 分解。walk 回溯的连续重复节点归一化去除
  （上游对重复项分解出零位移段，行为等价、省步数预算）。
- **D23（rectify_and_crop_v2 不移植）**：上游 CLIP 评分前对节点图做
  `rectify_and_crop_v2`（AirVLN 全景条带校正，依赖 cv2/PIL）。Blocks 五目阵
  图像非全景条带，校正无对应语义；`memory.nodes_landmarks_similarity` 直接用
  节点原图（PNG bytes）经 `/models/clip/score` 服务评分。"A photo of " 前缀与
  逐节点 max-over-images 语义逐字保留。另：五目阵（Yaw -90..+90）无 back 相机，
  节点 back 图像槽位留空（JSON 中为 null），评分跳过缺失图。
- 每规划周期为独立 `is_final` FollowTimedPath goal（SPF 同先例：observe-decide-move
  循环无法保证滚动 modify 的重叠连续性，见 spf `_send_revision` 注释）。
- **D24（networkx → 包内最小图实现）**：上游 `navigation_graph.py` 与作者 pkl
  图消费 networkx.Graph；onboard-computer v0.1.1 容器（py3.10）实测无
  networkx，按项目约定不重建共享镜像、不新增基础环境依赖，`memory.py` 内实现
  最小无向图 `UndirectedGraph`（add_node/add_edge/neighbors/get_edge_data/
  has_edge/number_of_nodes/nodes(data)/edges(data)/`__getitem__`，欧氏边权由
  `MemoryGraph.add_edge` 写入）。`dijskra_transform` 逐字移植本就不依赖 nx
  算法（heapq + 邻接接口），接口面不变。语义等价性由
  `tests/test_graph_equivalence.py` 对照 networkx 断言证明（图操作顺序、
  随机图 dijskra next/prev 逐元素一致、find_optimal_route walk/score 一致、
  JSON 边序一致）；networkx 仅测试期安装，运行时零依赖。
- 上游 `z>-5→z=5` 高度规则 → `min_altitude_m` 参数化（契约 §5）；上游
  `semantic_pc[0]` 只取首个非空类的行为逐字保留（勿"改进"为全类均值）。
- **LLM 契约对齐修订（2026-11-17）**：本包初版曾按"planner 发送 prompt+图像
  b64"设计 query；代理侧已定型为权威契约（prompt 与图像都不由 planner 发送，
  代理按 view_stamps 自取帧），planner 侧已改对齐、代理包未动。结果字段
  `text`/`latency_s` 与 error_code 六元集同步；自建 prompt 文本全部删除，
  仅保留 `parse_viewpoint_response_v2` 逐字复制（字段消费在 planner）。
- **rclpy logger 参数风格修复（阶段 6 首飞，2026-11-17，run-20261117b 发现）**：
  `node.py`/`hspm.py` 共 5 处使用 stdlib logging 风格的
  `logger.info("...%s", x)` 调用；rclpy `RcutilsLogger` 只接受单个 msg 参数，
  真实运行中 grounding 成功命中 `hspm.py` L154 即抛 `TypeError` 杀死 worker
  线程（失败路径 `_count_failure`、memory 读/写异常分支同病）。单测漏检原因：
  测试注入的是 stdlib 风格 mock logger。已改为 `%` 预格式化（语义不变，最小
  修改）；修复后单测 45 passed / 1 skipped 与基线一致。
- **depth 通道轴修复（阶段 6 首飞，2026-11-17，run-20261117c 发现）**：
  `node._on_image` 将 32FC1 深度经 `image_to_array(..., channels=1)` 存为
  `(H, W, 1)`，而 `perception.backproject_view` 按契约 §1 期望 `(H, W)`，
  真实运行 `h, w = depth.shape` 抛 `ValueError` 再次杀死 worker。回调处压掉
  通道轴（`[:, :, 0]`）；单测直接构造 2D 深度故漏检。
- **exploit 命中面修复（阶段 6 审计发现，2026-11-17；修复方向修正
  2026-11-18）**：在线建图（D4）在任务执行中 `add_or_merge` 本任务轨迹
  节点，exploit 检查若命中自身刚访问位置（≤20 m）会产生多余 goal/提前
  `task_complete`，违背上游"exploit 只命中既往任务记忆（episode 内图
  静态）"语义并破坏跨任务复用验收。首版修复（`min_index=_memory_floor`）
  方向写反：无既往记忆（floor=0）时不过滤、有既往记忆时反而排除既往只搜
  本任务节点（run-20261118d 实测 exploit_events=2 自命中 + 第 3 goal
  rejected）。最终修复：`_init_memory` 记录 `_memory_floor`（加载时的既往
  节点数），exploit 仅在 `floor > 0` 时检查，`MemoryGraph.closest_node`
  新增 `until`（默认 None 全搜）限定只搜既往节点 `[0, floor)`。

## 运行（阶段 6 Demo 前为开发入口）

```bash
# onboard-computer 容器内，共享接口包 overlay 已 source
colcon build --packages-select uav_citynav_planner   # 需 uav_planner_interfaces 先建
ros2 run uav_citynav_planner citynav_planner --ros-args -p vehicle_namespace:=drone0 \
  --remap __ns:=/drone0
# 任务：ros2 topic pub --once citynav/goal std_msgs/String \
#   '{\"instruction\": \"fly past the orange ball to the building\", \"max_steps\": 60, \"use_memory\": true}'
# 取消：ros2 service call citynav/cancel std_srvs/srv/Trigger
```

前置：grounded_sam 服务（默认 `127.0.0.1:8100`）与 CLIP 服务（默认
`127.0.0.1:8101`）在 PC GPU 侧运行；`uav_citynav_llm_proxy` 在工作站运行；
传感器桥按 CityNav Demo 变体配置五目阵 + ImageType 1 深度（契约 §1 缺口项，
阶段 6 落地）。

## 测试

```bash
uv venv --python 3.12 .venv-test && uv pip install --python .venv-test numpy scipy pytest networkx
.venv-test/Scripts/python.exe -m pytest tests -q    # Windows 开发机
python3 -m pytest tests -q                          # 容器/Linux（无 networkx 时对照文件自动 skip）
```

networkx 仅对照测试（`test_graph_equivalence.py`）需要，运行时零依赖（D24）。

2026-11-17 本机（D24 图替换后复跑）：45 passed, 1 skipped（`test_node.py`
为阶段 6/7 node 级 ROS 测试占位）。对照断言 4 项（test_graph_equivalence，
networkx 已装实测 PASSED）：图操作序一致、随机图 dijskra next/prev 逐元素
一致、find_optimal_route walk/score 一致、JSON 边序一致。覆盖：LLM query/result 权威契约字段
（无 prompt/图像、view_stamps ns 整数、error_code 六元集）、
`parse_viewpoint_response_v2` 与 upstream 源段 AST 比对（test_prompts_alignment，
仅豁免首行归属 docstring）、非恒等位姿/旋转反投影手算 ENU 对照（契约 §3 等价性证明项）、
深度无效值过滤、cKDTree 统计滤波已知小样本手算对照、动作分解顺序/步数/时长、
15° 子步合并单段时长不变、垂直 vz 比例与下限、min_altitude 钳制不改时序、
建图/20 m 合并/最近节点、JSON 落盘往返、dijskra 小图手算对照、find_optimal_route
小图 walk 端点、exploit 逐段、is_found 可见即推进、卡死判据、PNG 编解码往返
（含手工 Sub-filter 构造）、grounding/clip HTTP 客户端（monkeypatch，含 ≤16 分块）。
以上为纯逻辑单测（M2 的一部分）；不代表真实 DDS 生命周期、仿真传感器输入或
端到端验收（阶段 6/7，M3-M8）。
