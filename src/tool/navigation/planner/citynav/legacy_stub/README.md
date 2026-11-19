# CityNavAgent legacy 冒烟桩（阶段 1，路线 B）

上游代码原样运行于标准 AirSim Blocks，不做 AirVLN 基准复现（计划决策 1/4）。
本目录提供上游入口缺失的三样东西，全部**不修改子模块文件**：

| 文件 | 作用 |
| --- | --- |
| `scene_server.py` | AirVLN 场景服务器最小桩（msgpackrpc 30000：`ping`/`reopen_scenes`/`close_scenes`），把场景映射到 Blocks AirSim RPC 45451（偏离 D2） |
| `make_data.py` | 生成 `upstream/data/` 最小数据集：2 条 Blocks 指令 episode（OrangeBall 真值 (34.25, 33.10, -3.68)）+ 空记忆图/动作图 pkl（偏离 D3/D4：landmark 由指令给出、无作者图；explore-only） |
| `settings.citynav_legacy.json` | Blocks settings 变体：`Drone_1`（SimpleFlight，上游 RPC 调用固定 vehicle_name='Drone_1'）+ `front_0` 相机 512×512 FOV90（上游 camera_id 固定；512 统一 RGB/DEPTH 断言，偏离 D20）；ApiServerPort 45451 与 PX4 拓扑 45481 分离 |
| `run_legacy.py` | 运行器：`.env` 注入 key/base_url/模型（D6/D9）、LLM 有限重试（D10）、GroundedSam→Inference 服务客户端（D17/D18）、`--alt-floor ned5` 可选 NED 高度钳制修正 |
| `.env` | 本地密钥配置（gitignored） |

## 运行顺序

```powershell
# 0) 一次性：数据 + 环境
.venv\Scripts\python.exe make_data.py

# 1) Blocks：备份 Documents\AirSim\settings.json，拷入 settings.citynav_legacy.json，
#    启动本机 Blocks 安装目录下的 Blocks.exe（个人路径不入库）

# 2) grounded_sam Inference 服务（见 src/basic_function/inference/models/grounded_sam/README.md）

# 3) 场景服务器
.venv\Scripts\python.exe scene_server.py --airsim-port 45451

# 4) 冒烟
.venv\Scripts\python.exe run_legacy.py --max-step-size 60 --alt-floor upstream
```

## 环境（py3.8，uv venv，无 conda）

裁剪自上游 requirements（偏离 D12）：剔除 ROS1 pip 包与训练依赖；
torch 为 **CPU 版**（仅供 `clip` 模块导入链，legacy 不跑 GPU 推理；GPU 实测在
Inference 服务侧完成，D17）；`airsim==1.7.0`/`msgpack-rpc-python==0.4.1` 以
`--no-deps` 安装并配 `msgpack==1.0.5`（msgpack-python 0.5.6 无 cp38 win 轮子）。

## 已知行为注意

- 上游 sam 分支高度规则 `z>-5 → z=5` 在 AirVLN 数据约定下成立；Blocks NED 下会把
  waypoint 压入地下。默认 `--alt-floor upstream` 保真观察，异常时切 `ned5`（只改
  返回位姿 z，不改步数计数）。
- 全景取图 = 8 次 teleport 偏航（45°/步）后回到原位置附近；teleport 用
  `simSetVehiclePose(ignore_collision=False)`，SimpleFlight 下开放空间安全。
- 上游 `env_id` 全局名依赖 `__main__`，runner 已显式置 0。
- LLM 缓存写 `upstream/llm/cache/{model}/navigation.json`（untracked）。
