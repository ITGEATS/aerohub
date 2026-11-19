# Grounded-Sam Inference（GroundingDINO + SAM vit_h）

CityNavAgent 复现迁移的开放词汇分割 worker：`greedy_mask_predict` 语义逐字移植自
`src/tool/navigation/planner/citynav/upstream/external/Grounded_Sam_Lite/grounded_sam_api.py`
（`get_dino_output` L55-87、`greedy_mask_predict` L104-185）：caption 小写化并补 `.`、
`box_threshold`/`text_threshold` 过滤、`get_phrases_from_posmap` 短语解码、
`in_phrases` 按 `.` 拆分后 soft-contain 选 `best_phrase`、匹配框收集、
SAM `predict_torch` 多框 mask 取并集、失败返回（小数组, False）。默认阈值 box=0.3、text=0.25。
本模块只返回掩膜与短语；深度反投影、语义点云与导航消费留在 Planner/Tool 侧。

## 后端替换声明（重要）

上游钉定的原版 GroundingDINO 自定义 CUDA 算子（multi-scale deformable attention，
需 nvcc/MSVC 编译且无 CPU 路径）在本机（RTX 5060 Laptop，Blackwell sm_120）**不可运行**。
因此后端替换为：

- 检测：HF transformers `GroundingDinoForObjectDetection`（纯 PyTorch deformable attention）。
  **默认权重为 `artifacts/grounding-dino-swin-ogc/`：由本地
  `weights/groundingdino_swint_ogc.pth` 经 `convert_checkpoint.py` 转换而来**（键映射
  依据 transformers 官方转换脚本
  `src/transformers/models/grounding_dino/convert_grounding_dino_to_hf.py`
  （huggingface/transformers main，Apache-2.0；上游源码不复制入库，按
  AGENTS Fork/拷贝约定以 URL 溯源），
  其 grounding-dino-tiny 变体即 SwinT-OGC：Swin-T embed_dim 96、depths [2,2,6,2]、
  window 7、text tower BERT——与上游 label_encoder 同源，可完整映射）。
  `artifacts/grounding-dino-base/`（`IDEA-Research/grounding-dino-base`，Swin-B 骨干，
  经 `https://hf-mirror.com` 下载）保留为备选后端（`worker.yaml` 的 `dino_model` 可切换）。
- 分割：`facebookresearch/segment_anything` 原版 SAM `vit_h`，
  加载本地 `weights/sam_vit_h_4b8939.pth`（`sam_model_registry['vit_h']`）。

**与原版 CUDA 算子实现无数值对照（原版在本机不可运行）；SAM 权重同源；
DINO 默认权重与上游 SwinT-OGC checkpoint 同源（本地 HF 转换）。**
转换注意：HF 端口对文本 padding 列输出 `-inf` 类分数（官方 grounding-dino-base
端口同样如此，实测两者一致），worker 的 sigmoid+max 过滤将其视为 0，不消费这些列；
`convert_checkpoint.py` 的自检只检查真实 token 列。
短语解码使用 transformers 4.57.6 自带的 `get_phrases_from_posmap`
（`transformers/models/grounding_dino/processing_grounding_dino.py`，logits 为
(nq, 256) 文本 token 头，与上游注释一致）；IDEA-Research 原版
`groundingdino/util/utils.py` 语义核对自上游仓库
`https://github.com/IDEA-Research/GroundingDINO`（本机经
`https://gh-proxy.com` 镜像获取阅读；上游源码不复制入库）。

## 环境

- Windows 主机、PowerShell/Git Bash；uv 0.13.7；无 conda。
- Python 3.12（`requires-python >=3.12,<3.13`），锁文件固定：
  torch 2.9.1+cu128（`https://download.pytorch.org/whl/cu128` 直连可用，
  未使用 SJTU 镜像）、torchvision 0.24.1+cu128、transformers 4.57.6、
  segment-anything 1.0（git 安装 `https://gh-proxy.com/https://github.com/facebookresearch/segment-anything`，
  直连 GitHub 不稳定时的既定回退路径，未使用 vendor/ 克隆）。
- 环境目录为本模型被忽略的 `.venv/`（`UV_PROJECT_ENVIRONMENT` 显式指向）。
- 公共 runtime 以仓库相对路径依赖接入；本 README 不复制完整依赖表（以
  `pyproject.toml`/`uv.lock` 为准）。

## 权重与校验清单（`artifacts.json`）

| 文件 | 大小 | sha256 | 状态 |
| --- | --- | --- | --- |
| `weights/sam_vit_h_4b8939.pth` | 2564550879 | `a7bf3b02f3ebf1267aba913ff637d9a2d5c33d3173bb679e46d9f338c26f262e` | 使用（官方发布哈希一致） |
| `artifacts/grounding-dino-swin-ogc/model.safetensors` | 689359096 | `1a2412ef99bd74bcd3c2a246fa1e48581f8889a1300c9051974741314fc042f3` | **默认**（本地转换自 swint_ogc.pth，`convert_checkpoint.py` 生成后固定） |
| `artifacts/grounding-dino-base/model.safetensors` | 933400872 | `5548f844c928c4b6f411fa8cbcc2bfa8dbbba437cb1d513975519f93c2a9ed21` | 备选（hf-mirror 下载） |
| `weights/groundingdino_swint_ogc.pth` | 693997677 | `3b3ca2563c77c69f651d7bd133e97139c186df06231157a64c507099c52bc799` | 转换源（保留校验） |

`weights/` 与 `artifacts/`、`.venv/` 均被 git 忽略。SAM 权重是前置条件：
`setup.sh` 只校验不自动下载（缺失时明确报错并给出官方 URL）。

## 准备

从仓库根目录（Git Bash；Windows 上 `bash` 若解析到未安装的 WLS 桩需显式用 Git Bash）：

```bash
bash src/basic_function/inference/models/grounded_sam/setup.sh            # 校验+下载+uv sync+CPU 加载预热
bash src/basic_function/inference/models/grounded_sam/setup.sh --check-cuda  # 另做真实 GPU 加载/前向预热
```

`setup.sh` 顺序：校验 uv 与本地权重 sha256 → 将 HF DINO base 端口文件准备到被忽略的
`artifacts/grounding-dino-base/`（先临时文件、sha256 校验后放入；已存在且正确的文件
复用，不符时明确失败不覆盖）→ `uv sync --locked` → `convert_checkpoint.py` 从本地
`weights/groundingdino_swint_ogc.pth` 确定性生成默认后端
`artifacts/grounding-dino-swin-ogc/`（pinned sha256 命中即跳过）→ 加载默认权重预热。
CPU 预热检查在 SAM vit_h 上较慢（分钟级），仅为准备检查，不代表 GPU 验收。
不启动任何服务。也可经统一入口：
`python deployment/inference/setup_models.py --config <yaml> [--check-cuda]`
（manifest ID：`grounded_sam`）。

## 服务契约与启动

优先接入公共 `aerohub_inference` runtime：`serve.py` 用
`create_app(Predictor, GreedyMaskRequest, GreedyMaskResponse, path=...)` 注册。
公共 runtime 原只暴露 `/infer`，为携带本契约 URL 对 runtime 做了最小扩展
（`create_app` 新增可选 `path` 关键字参数，默认 `/infer`，
yolo26n_seg/diffphys/yopo 行为不变）。请求 schema 由模型提供，`text_prompt`
等字段天然可携带。

启动（模型目录内；GPU 设备用 `GS_DEVICE=0`，CPU 用 `GS_DEVICE=cpu`）：

```bash
GS_DEVICE=0 .venv/Scripts/python.exe -m uvicorn serve:app --host 127.0.0.1 --port 8100
```

- `POST http://127.0.0.1:8100/models/grounded_sam/greedy_mask_predict`
  请求 `{"image_b64": BGR PNG/JPEG base64, "text_prompt": str, "box_threshold": 0.3, "text_threshold": 0.25}`
  响应 `{"mask_b64": 单通道PNG(0/255) base64, "seg_success": bool, "phrases": [str]}`。
  字段名与 CityNav planner 客户端（`src/tool/navigation/planner/citynav/ros2_adapter/uav_citynav_planner/grounding_client.py`）
  一致，不可改（阶段 1 legacy 冒烟桩已随 Tool 化交付移除，历史见 Git 提交 8f84f3c）。
- `seg_success=false` 时 `mask_b64` 为空串（客户端仅在成功时解码）；
  内部保持上游“小数组 + False”失败形态。
- `GET /health/live`、`GET /health/ready`（含 model_id/model_version/device/processing）。
- runtime 忙时 429；非法输入 422；设备错误（如 `GS_DEVICE=7`）在加载时报
  `device error: requested cuda:7, only 1 CUDA device(s) visible`。
- 图像限制：≤4096×4096，base64 ≤16 MiB。

## 验证边界（2026-11-17 本机实测）

- `setup.sh --check-cuda`（Git Bash、仓库根目录执行）：prepare 复用已校验文件不重写
  （重复执行前后 `artifacts/` sha256 一致）；CPU 加载预热 28.6 s；RTX 5060 Laptop
  真实加载 DINO+SAM 并前向预热成功（warmup 8.4–10.8 s，显存 allocated 3416 MiB /
  reserved 7602 MiB）。
- 真实帧端到端 HTTP 分割：`log/spf/legacy/run-002-smoke-pass/frame_airsim_test.jpg`
  （AirSim Blocks FPV 实拍 1920×1080），prompt `orange ball` 与 `building` 均
  `seg_success=true`，短语解码正确；证据（输入图、mask、叠加可视化、JSON 报告）在
  `log/citynav/perception/`，复现命令：

  ```bash
  .venv/Scripts/python.exe test_e2e.py --image <frame> --out-dir <log/citynav/perception>
  ```

- 失败路径：prompt `helicopter`（场景无该物）返回 `seg_success=false` 不崩溃；
  提高 `box_threshold` 至 0.95 同样得 false。DINO 校准偏松：无中生有的
  `zzz qqq wwww` 在该帧可得分>0.3（与上游同阈值行为一致，非移植缺陷）。
- SwinT-OGC 本地转换后端（默认）GPU 实测（2026-11-17）：同帧 adapter 直连，
  `orange ball` true 面积 42723（base 42724，几乎一致）、`building` true 面积
  903013（base 1023726，骨干容量不同所致，属预期差异）、`helicopter` false；
  warmup 6.63 s，显存 allocated 3185 MiB / reserved 7230 MiB。证据：
  `log/citynav/perception/swint/`（含与 base 的逐 prompt 对照 JSON）。
  HTTP 级 swin-ogc 验证未做——当时 8100 服务以 base 后端在跑且约定不干扰；
  下次按新 `worker.yaml` 默认重启服务即走 swin-ogc。
- 以上不覆盖：与原版 CUDA 算子的数值对照（本机不可运行）、ROS/AS2 接口、
  AirSim 闭环下游消费与飞行验收；这些归 CityNav 阶段 2/3。

## 偏离记录

0. **GS_EMPTY_CACHE（2026-11-18，CityNav 阶段 6 阻塞修复）**：本机 8 GB 显存下
   fp32 warmup 使 caching allocator reserved 达 ~7.6 GB，UE4 Blocks 被 WDDM 驱逐
   崩溃。新增默认开启的环境开关：warmup 后与每次推理后 `torch.cuda.empty_cache()`，
   实测混合尺寸冲击稳态 ~3.7 GB；设 `GS_EMPTY_CACHE=0` 恢复原行为。代价为每请求
   重分配延迟（百 ms 级），服务空闲期无影响。与 CityNav 飞行验收耦合：Demo 运行
   前提为本开关开启。

1. DINO 后端为 HF transformers 端口 + 本地转换的 SwinT-OGC 权重（与上游 checkpoint
   同源、骨干配置一致）；非上游钉定的自定义 CUDA 算子版，无数值对照（本机不可运行）。
   曾评估的 `grounding-dino-base`（Swin-B）保留为备选。Swint-OGC 本地 HF 转换已落地：
   早前"forward 非有限 logits"诊断为自检脚本切片维度错误 + HF 端口 padding 列固有
   `-inf`，非转换缺陷（官方 base 端口行为相同）。
2. 公共 runtime `create_app` 增加可选 `path` 参数（默认值保持旧行为）。
3. Windows 上 `deployment/inference/setup_models.py` 以裸 `bash` 调用 `setup.sh`，
   若 PATH 中 `bash` 是 WSL 桩则会失败；本机用 Git Bash 直接执行 `setup.sh` 验证。
