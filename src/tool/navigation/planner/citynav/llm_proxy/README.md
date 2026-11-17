"""uav_citynav_llm_proxy — CityNavAgent 工作站云端 LLM 代理 ROS 2 包（迁移方案阶段 3）。

职责（README §2/§3）：算力容器 `uav_citynav_planner` 不直连公网；本代理在工作站
订阅五目相机阵 RGB、缓存各 view 最新帧，按 `citynav/llm/query` 请求构造与上游
`query_llm.py` 同构的 OpenAI 兼容多模态消息，调用云端模型（默认智谱
glm-4v-flash，D6），结果经 `citynav/llm/result` 回发。提示词与
`parse_viewpoint_response_v2` 逐字复制自 upstream 锁定提交
（4b6e926c3325995483576523b18c9d0de5eb0710），见 `prompts.py` 文件头。

## 接口契约（std_msgs/String JSON，话题相对节点 namespace）

入 `citynav/llm/query`：

```json
{"request_id": "…", "op": "viewpoint|landmark|caption",
 "instruction": "…", "landmarks": ["…"], "traversed": ["…"], "next": "…",
 "view_stamps": {"left": 1234567890000000000, "…": 0}, "max_age_s": 90.0}
```

- `viewpoint`：`route_planning_prompt_builder(instruction, landmarks, traversed,
  next)` + 五视角图（顺序=cameras 参数顺序，即上游 left/slightly_left/front/
  slightly_right/right）。代理侧先跑 `parse_viewpoint_response_v2` 校验响应
  JSON 可解析，不可解析回 `parse`。
- `caption`：`landmark_caption_prompt_builder(landmarks)` + `view_stamps` 中
  第一个视角的单帧（上游 captioner 即逐图单调用；多图由 planner 循环发起）。
- `landmark`：决策 7 的 LLM 指令抽取，消息结构逐字取
  `utils/landmark_extraction.py`（system+user，无图像）。
- 取帧：仅缓存每 label 最新帧；`view_stamps` 精确匹配即该帧，否则退化为最新；
  帧龄（节点时钟，`use_sim_time` 下为仿真时间）超 `max_age_s`（缺省 90 s，
  D21 包络）或缺帧 → `stale`。

出 `citynav/llm/result`：

```json
{"request_id": "…", "op": "…", "ok": true, "text": "模型原始文本",
 "error_code": "ok|stale|timeout|http|parse|busy", "latency_s": 8.1}
```

入 `sensor_measurements/<Cam>/rgb/image`（sensor_msgs/Image rgb8，sensor-data
QoS）；相机映射参数 `cameras`（`"label=camera"` 字符串列表，默认
CityLeft/CitySlightlyLeft/CityFront/CitySlightlyRight/CityRight ↔
left/slightly_left/front/slightly_right/right）。

## 参数

`use_sim_time`（rclpy 内建）、`vehicle_namespace`（仅日志；namespace 由 `__ns`
给定）、`cameras`、`model`（默认 glm-4v-flash）、`base_url`（默认
`https://open.bigmodel.cn/api/paas/v4`）、`api_key_env`（默认
`OPENAI_API_KEY`，密钥不入库 D9）、`cache_dir`（默认空=禁用）、
`request_timeout_s`（默认 30.0）。

## 构建与运行（工作站容器补编译 + install overlay，不重建共享镜像）

```bash
colcon build --packages-select uav_citynav_llm_proxy
. install/setup.bash
export OPENAI_API_KEY=…
ros2 run uav_citynav_llm_proxy citynav_llm_proxy --ros-args \
  -r __ns:=/drone0 \
  -p cache_dir:=log/citynav/llm/cache -p use_sim_time:=true
```

查询示例（viewpoint，帧 stamp 由 planner 批次给出）：

```bash
ros2 topic pub --once /drone0/citynav/llm/query std_msgs/msg/String \
  "{data: '{\"request_id\":\"q1\",\"op\":\"viewpoint\",\"instruction\":\"fly past the building, then to the orange ball\",\"landmarks\":[\"building\",\"orange ball\"],\"traversed\":[],\"next\":\"building\",\"view_stamps\":{\"left\":0,\"slightly_left\":0,\"front\":0,\"slightly_right\":0,\"right\":0},\"max_age_s\":90.0}'}"
ros2 topic echo /drone0/citynav/llm/result
```

## 离线测试（不依赖 rclpy 的纯逻辑层）

```bash
uv venv .venv-test --python 3.12   # 临时 venv，不入库
uv pip install --python .venv-test pytest
.venv-test/Scripts/python.exe -m pytest tests -q   # Linux 下为 bin/python
```

`tests/test_node_ros.py` 为 rclpy 级集成测试占位，阶段 6/7 执行。

## 与上游的偏离（本代理范围）

- **PNG 传输编码**：本包禁 cv2/requests/pydantic，rgb8→base64 用 zlib 最小
  PNG 编码，data URL 为 `image/png`（上游 `image/jpeg`）。仅为传输编码偏离，
  prompt 文本与消息块结构逐字同构，不影响语义；glm-4v-flash 接受 png。
- **缓存键不含图像**：`sha256(prompt+model)` → `{prompt, response}` JSON 落盘，
  命中不请求。与上游 `LLM.get_cache` 的 prompt 前缀键语义一致（图像同样不在
  键内）；同一 prompt 换图会返回缓存响应，需要逐帧真实调用时保持
  `cache_dir:=""`（默认即禁用，阶段 6 验收 M-1 对照跑须禁用）。
- **有限重试**（D10）：上游 sleep 10 s 无限递归 → 3 次线性退避后回 `http`/
  `timeout`。
- **busy 单飞**：处理中收到新 query 立即回 `busy`（不排队），由 planner 决定
  重试节奏。
- landmark 抽取提示词逐字来源为 `utils/landmark_extraction.py`（README §2 误记
  为 `src/llm/prompt_builder.py`，该文件无抽取 builder；已在 prompts.py 头注声明）。

多机外溢：request_id/缓存键本期未按 vehicle 隔离（README §6，本期单机验收）。
