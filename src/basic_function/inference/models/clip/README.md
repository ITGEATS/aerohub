# CLIP ViT-L/14 Inference（图文相似度节点评分）

CityNavAgent 记忆 exploit 路径的节点评分 worker：为
`src/tool/navigation/planner/citynav/upstream/external/lm_nav/optimal_route.py`
`nodes_landmarks_similarity`（L40-76）提供图文余弦相似度矩阵服务，
落实迁移方案 D19 的 CLIP 服务化承诺（见
`.trae/documents/citynavagent-migration-plan.md` 决策表 D19/D23）。

上游消费语义保留在调用方：`text = "A photo of " + desc` 前缀、每节点多图取
max、矩阵按节点聚合——本服务只返回 `similarity[i][j] = cos(图 i, 文本 j)`
原始矩阵，不做前缀、不做 max 归约。`rectify_and_crop_v2` 预处理**不移植**
（D23：该裁剪绑定 AirVLN 全景几何，服务只收原图，调用方已声明偏离）。

## 后端与权重来源

- transformers `CLIPModel`/`CLIPProcessor`，权重
  `openai/clip-vit-large-patch14`（HF 官方端口，与上游
  `clip.load("ViT-L/14")` 同源同一官方权重；上游为 openaipublic CDN
  `VIT-L-14.pt`）。
- 下载端点：`https://hf-mirror.com`（本机 `huggingface.co` 直连不可达，
  与 grounded_sam 相同的既定回退）。`model.safetensors` sha256 取自 HF
  tree API 的 LFS 哈希；小文件 sha256 由镜像下载实测记录（见
  `artifacts.json`，全部固定）。
- 预处理对应关系：HF `CLIPProcessor` 的图像链路（短边 resize 224、中心裁剪、
  CLIP mean/std 归一化）与文本 BPE 链路（截断/补齐 77 token）语义对应上游
  `clip.preprocess`/`clip.tokenize`。特征均显式 L2 归一化后点积，与上游
  `features /= features.norm(...)` 一致。
- 与 openai-clip 包未做逐数值对照（本机 legacy 环境为 torch 2.2.2+cpu，
  D17 约束下不跑 GPU 推理；权重同源、预处理语义等价，如实声明边界）。

## 环境

- Windows 主机；uv 0.13.7；无 conda。
- Python 3.12（`requires-python >=3.12,<3.13`），torch 2.9.1+cu128
  （`https://download.pytorch.org/whl/cu128`，RTX 5060 Laptop sm_120），
  transformers 4.x（以 `uv.lock` 为准）。
- 环境目录为本模型被忽略的 `.venv/`（`UV_PROJECT_ENVIRONMENT` 显式指向）。
- 公共 runtime 以仓库相对路径依赖接入；依赖表以
  `pyproject.toml`/`uv.lock` 为准。

## 准备

从仓库根目录（Git Bash；Windows 上 `bash` 若解析到 WSL 桩需显式用 Git Bash）：

```bash
bash src/basic_function/inference/models/clip/setup.sh            # 校验+下载+uv sync+CPU 加载预热
bash src/basic_function/inference/models/clip/setup.sh --check-cuda  # 另做真实 GPU 加载/前向预热
```

`setup.sh` 顺序：校验 uv → `prepare_artifacts.py` 将 HF 端口文件准备到被忽略的
`artifacts/clip-vit-large-patch14/`（先临时文件、sha256 校验后放入；已存在且
正确的文件复用，不符时明确失败不覆盖）→ `uv sync --locked` → 加载默认权重
CPU 预热（仅准备检查）→ `--check-cuda` 时真实 GPU 加载/前向预热，不静默降级。
不启动任何服务。也可经统一入口：
`python deployment/inference/setup_models.py --config <yaml> [--check-cuda]`
（manifest ID：`clip`）。

## 服务契约与启动

`serve.py` 用 `create_app(Predictor, ClipScoreRequest, ClipScoreResponse,
path='/models/clip/score')` 注册公共 runtime。

启动（模型目录内；GPU 设备用 `CLIP_DEVICE=0`，CPU 用 `CLIP_DEVICE=cpu`）：

```bash
CLIP_DEVICE=0 .venv/Scripts/python.exe -m uvicorn serve:app --host 127.0.0.1 --port 8101
```

- `POST http://127.0.0.1:8101/models/clip/score`
  请求 `{"images_b64": [PNG/JPEG base64, ...], "texts": [str, ...]}`
  响应 `{"similarity": [[...]], "model_version": str}`；
  `similarity[i][j]` = 图 i 与文本 j 的余弦相似度（float）。
  此契约固定，调用方（CityNav adapter）按此实现。
- 限制：`images_b64` 与 `texts` 均须非空且各 ≤16 项；单图长边 ≤4096px；
  空列表/超限/base64 或图像解码失败 → 422。`extra` 字段禁止。
- `model_version` 为 `{repo, revision, model_sha256, torch, transformers}`
  规范化 JSON 的 sha256（同 grounded_sam 惯例），重复执行幂等。
- `GET /health/live`、`GET /health/ready`（含 model_id/model_version/device/
  processing/max_images/max_texts）。
- runtime 忙时 429；设备错误（如 `CLIP_DEVICE=7`）在加载时报
  `device error: requested cuda:7, only 1 CUDA device(s) visible`。

## 验证边界（2026-11-17 本机实测）

- `setup.sh --check-cuda`（Git Bash、仓库根目录执行）：prepare 复用已校验文件
  不重写（重复执行 sha256 不变）；CPU 加载预热 2.0 s；RTX 5060 Laptop 真实加载
  并前向预热成功（warmup 1.2–1.8 s，显存 allocated 1641 MiB / reserved 1698 MiB）。
- 权重下载注意：首次多连接分段下载因宿主带宽受限中途退出留下零洞文件
  （输出 NaN、哈希不符）。已按零区间扫描补齐 Range，最终
  `model.safetensors` sha256 与官方 git-LFS 指针 `a2bf730a…` 一致
  （详见 `log/citynav/perception/clip/NOTES.md`）。`prepare_artifacts.py`
  单连接下载 + sha256 校验 + 残件删除重试可防此类残缺；若镜像持续不可达，
  如实报告不静默降级。
- 真实帧端到端 HTTP（`POST /models/clip/score`，cuda:0）：
  `rgb_view0.png`（平台检查 512×512）与 `frame_airsim_test.jpg`（Blocks 橙球帧
  1920×1080），texts=`["orange ball","building","helicopter"]`；矩阵、两次相同
  请求幂等（`idempotent: true`）、往返耗时（首包约 245 ms、热态约 169 ms）与
  422 拒绝路径（空列表/17 图/17 文本）见
  `log/citynav/perception/clip/clip_score_report.json`；另以上游
  `"A photo of "+desc` 前缀语义对照运行
  `clip_score_report_upstream_prefix.json`：球帧 orange ball 高于 helicopter
  （margin +0.0033，方向符合预期；数值记录不设门槛，不夸大为"显著"）。

复现命令（服务启动后）：

```bash
.venv/Scripts/python.exe test_e2e.py --image <rgb_view0.png> --image <frame_airsim_test.jpg> \
    --out-dir log/citynav/perception/clip
```

证据目录：`log/citynav/perception/clip/`（JSON 报告 + 说明）。真实帧：
`log/citynav/legacy/platform-check-*/rgb_view0.png` 与
`log/spf/legacy/run-002-smoke-pass/frame_airsim_test.jpg`，
texts=`["orange ball","building","helicopter"]`；数值记录不设门槛。

- 以上不覆盖：ROS/AS2 接口、`rectify_and_crop_v2` 语义对照（明确不移植，D23）、
  与 openai-clip 包的逐数值对照、AirSim 闭环下游消费与飞行验收；
  这些归 CityNav 后续阶段。

## 偏离记录

1. 不移植 `rectify_and_crop_v2`（D23 既定偏离）：服务收原图，调用方负责
   上游语义（前缀与多图 max）。
2. 后端为 HF transformers 端口而非 openai-clip 包（D17 连锁：上游
   torch 2.0.1+cu118 在 sm_120 不可用）；权重同源，无逐数值对照，边界如实声明。
3. `huggingface.co` 直连不可达，权重经 `hf-mirror.com` 获取；
   `model.safetensors` 以 HF tree API 的 LFS sha256 固定。
