"""CityNavAgent legacy 冒烟运行器（Blocks + 场景服务器桩 + LLM/grounding 外置）。

不修改 upstream 子模块任何文件；全部替换经 sys.modules 桩与模块属性 monkey-patch：
  1. OpenAI_LLM_v2 -> PatchedLLM：模型名/base_url/key 由环境配置（计划 D6/D9），
     重试由无限递归改为有限次（D10），提示词与调用语义逐字保留。
  2. external.Grounded_Sam_Lite.* -> 桩：原版 CUDA 算子在本机不可运行（D17），
     GroundedSam 改为调用 Inference 模块 grounded_sam 服务（D18）。
  3. RGB/DEPTH 尺寸断言统一 512（上游默认 224/256 自相矛盾，单相机无法同时满足，D20）。
  4. --alt-floor ned5：可选把 sam 分支 waypoint 高度钳制按 NED 修正为"海拔≥5 m"
     （上游 `z>-5 → z=5` 在其数据约定下成立，Blocks NED 下会把机血压入地下；
     默认 upstream 保持原样）。

前置：Blocks.exe 以 settings.citynav_legacy.json 运行（ApiServerPort 45451）；
      scene_server.py 已启动（30000）；grounded_sam 服务已启动；
      make_data.py 已生成 upstream/data/。
"""
import argparse
import base64
import json
import os
import sys
import time
import types
import urllib.request

STUB_DIR = os.path.dirname(os.path.abspath(__file__))
UPSTREAM = os.path.abspath(os.path.join(STUB_DIR, os.pardir, "upstream"))


def load_env(path):
    if os.path.isfile(path):
        with open(path, encoding="utf-8") as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith("#") and "=" in line:
                    k, v = line.split("=", 1)
                    os.environ.setdefault(k.strip(), v.strip())


own = argparse.ArgumentParser()
own.add_argument("--episodes", type=int, default=1, help="只跑 json 前 N 条 episode（桩数据生成器控制总数）")
own.add_argument("--max-step-size", type=int, default=60)
own.add_argument("--vlm-name", type=str, default="sam", choices=["sam"])
own.add_argument("--alt-floor", type=str, default="upstream", choices=["upstream", "ned5"])
own.add_argument("--grounding-url", type=str, default=os.environ.get("CITYNAV_GROUNDING_URL", ""))
own.add_argument("--out-dir", type=str, default=os.path.join(STUB_DIR, os.pardir, os.pardir, os.pardir, os.pardir, os.pardir, os.pardir, "log", "citynav", "legacy", "run-" + time.strftime("%Y%m%d-%H%M%S")))
args = own.parse_args()

load_env(os.path.join(STUB_DIR, ".env"))
llm_model = os.environ.get("CITYNAV_LLM_MODEL", "glm-4v-flash")
llm_base = os.environ.get("OPENAI_BASE_URL", "https://open.bigmodel.cn/api/paas/v4")
llm_key = os.environ.get("OPENAI_API_KEY", "")
grounding_url = args.grounding_url or os.environ.get("CITYNAV_GROUNDING_URL", "")
if not llm_key:
    raise SystemExit("OPENAI_API_KEY missing (.env)")

# param.py 在 import 时解析 sys.argv；上游默认 224/256 断言自相矛盾，统一 512（D20）
sys.argv = ["SimRun.py", "--run_type", "eval",
            "--Image_Height_RGB", "512", "--Image_Width_RGB", "512",
            "--Image_Height_DEPTH", "512", "--Image_Width_DEPTH", "512"]

os.chdir(UPSTREAM)
if UPSTREAM not in sys.path:
    sys.path.insert(0, UPSTREAM)
# utils/utils.py L13 是 py2 风格隐式相对导入 `from maps import ...`（上游既有行为）。
# 不能把 utils/ 加入 sys.path（utils/utils.py 会遮蔽 utils 包），改为预注册别名。
import utils.maps as _utils_maps  # noqa: E402
sys.modules["maps"] = _utils_maps


# ---------- external.Grounded_Sam_Lite 桩（含父包，避免真实 groundingdino import） ----------
def _mk(name):
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m


pkg = _mk("external.Grounded_Sam_Lite")
pkg.__path__ = [os.path.join(UPSTREAM, "external", "Grounded_Sam_Lite")]
gd = _mk("external.Grounded_Sam_Lite.groundingdino")
gd.__path__ = []
gd_util = _mk("external.Grounded_Sam_Lite.groundingdino.util")
gd_util.__path__ = []
gd_inf = _mk("external.Grounded_Sam_Lite.groundingdino.util.inference")
gd_ds = _mk("external.Grounded_Sam_Lite.groundingdino.datasets")
gd_ds.__path__ = []
gd_t = _mk("external.Grounded_Sam_Lite.groundingdino.datasets.transforms")


def _no_dino(*a, **k):
    raise RuntimeError("in-process GroundingDINO disabled: backend replaced by inference service (D17/D18)")


gd_inf.load_model = _no_dino
gd_inf.predict = _no_dino


class _TCompose:
    def __init__(self, ops):
        self.ops = ops

    def __call__(self, image, target=None):
        import torch as _t
        arr = _t.as_tensor(image)
        return arr, target


gd_t.Compose = _TCompose
gd_t.ToTensor = lambda *a, **k: None
gd_t.Normalize = lambda *a, **k: None

gs_api = _mk("external.Grounded_Sam_Lite.grounded_sam_api")


class GroundedSamClient:
    """greedy_mask_predict 语义不变，本地 CUDA 后端 -> grounded_sam Inference 服务。"""

    def __init__(self, dino_checkpoint_path=None, sam_checkpoint_path=None, **kwargs):
        self.url = grounding_url
        if not self.url:
            raise RuntimeError("CITYNAV_GROUNDING_URL missing")

    def greedy_mask_predict(self, image, text_prompt, box_threshold=0.3, text_threshold=0.25, visualize=False):
        import cv2
        import numpy as np
        ok, buf = cv2.imencode(".png", image)
        if not ok:
            raise RuntimeError("png encode failed")
        payload = {
            "image_b64": base64.b64encode(buf.tobytes()).decode("ascii"),
            "text_prompt": text_prompt,
            "box_threshold": box_threshold,
            "text_threshold": text_threshold,
        }
        req = urllib.request.Request(
            self.url, data=json.dumps(payload).encode("utf-8"),
            headers={"Content-Type": "application/json"}, method="POST")
        with urllib.request.urlopen(req, timeout=120) as resp:
            out = json.loads(resp.read().decode("utf-8"))
        # 服务契约：{"mask_b64": base64 PNG(单通道 0/255), "seg_success": bool}
        # 若 grounded_sam 模块最终契约不同，只改此处解码。
        mask = np.zeros(image.shape[:2], dtype=bool)
        seg_success = bool(out.get("seg_success", False))
        if seg_success and out.get("mask_b64"):
            mb = cv2.imdecode(np.frombuffer(base64.b64decode(out["mask_b64"]), np.uint8), cv2.IMREAD_GRAYSCALE)
            mask = mb > 127
        return mask, seg_success


gs_api.GroundedSam = GroundedSamClient

# `import a.b.c.d as X` 沿父模块属性链解析：显式挂接
pkg.groundingdino = gd
pkg.grounded_sam_api = gs_api
gd.util = gd_util
gd.datasets = gd_ds
gd_util.inference = gd_inf
gd_ds.transforms = gd_t

# lm_nav/landmark_extraction.py 顶层 import spacy（text_to_landmarks_gpt3 在本路径
# 永不被调——full_pipeline 始终显式传 landmarks；且 spacy 3.7.6 要求 py>=3.9）。桩掉。
_spacy = _mk("spacy")


def _no_spacy(*a, **k):
    raise RuntimeError("spacy stubbed: unused on CityNavAgent eval path (landmarks always provided)")


_spacy.load = _no_spacy

# ---------- cv2.imwrite 非 ASCII 路径补丁 ----------
# Windows 上 cv2.imwrite 对含中文的路径（本仓库在 Desktop/无人机/ 下）静默失败返回
# False；上游 captioner 依赖 obs_imgs/*.png 回读，必须修。不改上游，全局包装。
import cv2 as _cv2  # noqa: E402

_orig_imwrite = _cv2.imwrite


def _safe_imwrite(path, img, *a, **k):
    try:
        if _orig_imwrite(path, img, *a, **k):
            return True
    except Exception:
        pass
    ext = os.path.splitext(str(path))[1] or ".png"
    ok, buf = _cv2.imencode(ext, img, *a, **k)
    if ok:
        with open(path, "wb") as f:
            f.write(buf.tobytes())
    return bool(ok)


_cv2.imwrite = _safe_imwrite

# ---------- LLM 替换：模型/端点/密钥配置化，有限重试；提示词与消息结构逐字保留 ----------
import src.llm.query_llm as ql  # noqa: E402


class PatchedLLM(ql.LLM):
    def __init__(self, model_name, api_key, client_type="openai", logit_bias=None,
                 max_tokens=64, finish_reasons=None, **kwargs):
        from openai import OpenAI
        self.client = OpenAI(api_key=llm_key, base_url=llm_base, max_retries=0)
        self.logit_bias = logit_bias
        self.finish_reasons = finish_reasons if finish_reasons is not None else ["stop", "length"]
        super().__init__(api_key, llm_model, max_tokens, **kwargs)

    def encode_image(self, image_path):
        with open(image_path, "rb") as f:
            return base64.b64encode(f.read()).decode("utf-8")

    def _create(self, messages):
        return self.client.chat.completions.create(messages=messages, model=llm_model)

    def _query_capped(self, query_func, retries=3):
        last = None
        for i in range(retries):
            try:
                return query_func()
            except Exception as e:  # D10：有限重试
                last = e
                print(f"[llm] attempt {i + 1}/{retries} failed: {e}", flush=True)
                time.sleep(2 * (i + 1))
        raise RuntimeError(f"LLM query failed after {retries} retries: {last}")

    def query_viewpoint_api(self, prompt, image_paths=None, show_response=True):
        def query_func():
            content_block = []
            if image_paths is not None:
                for viewpoint, img_p in image_paths.items():
                    content_block.append({"type": "text", "text": f"{viewpoint} image: "})
                    content_block.append({"type": "image_url", "image_url":
                                          {"url": f"data:image/jpeg;base64,{self.encode_image(img_p)}"}})
            content_block.append({"type": "text", "text": f"{prompt}"})
            completion = self._create([{"role": "user", "content": content_block}])
            import unicodedata
            return unicodedata.normalize("NFKC", completion.choices[0].message.content)

        response = self._query_capped(query_func)
        if show_response:
            print("API Response:", response)
        return response

    def query_api(self, prompt, image_path=None, system=None, show_response=True):
        def query_func():
            query_messages = {"role": "user", "content": [{"type": "text", "text": prompt}]}
            if image_path is not None:
                query_messages["content"].append({"type": "image_url", "image_url":
                                                  {"url": f"data:image/jpeg;base64,{self.encode_image(image_path)}"}})
            completion = self._create([query_messages])
            import unicodedata
            return unicodedata.normalize("NFKC", completion.choices[0].message.content)

        response = self._query_capped(query_func)
        if show_response:
            print("API Response:", response)
        return response


# ---------- 导入上游主模块并打补丁 ----------
import SimRun as SR  # noqa: E402

SR.OpenAI_LLM_v2 = PatchedLLM
SR.GroundedSam = GroundedSamClient
SR.env_id = 0  # CityNavAgent() 内部数据路径使用全局 env_id

if args.alt_floor == "ned5":
    _orig_explore = SR.explore_pipeline_by_sam

    def _explore_ned5(*a, **k):
        sz, pose, found = _orig_explore(*a, **k)
        if pose.position.z_val > -5.0:  # NED：海拔低于 5 m 时钳到 5 m（步数不重算，见 README）
            pose = SR.convert_airsim_pose([pose.position.x_val, pose.position.y_val, -5.0,
                                          pose.orientation.x_val, pose.orientation.y_val,
                                          pose.orientation.z_val, pose.orientation.w_val])
        return sz, pose, found

    SR.explore_pipeline_by_sam = _explore_ned5

# ---------- 输出 tee ----------
out_dir = os.path.abspath(args.out_dir)
os.makedirs(out_dir, exist_ok=True)
logf = open(os.path.join(out_dir, "console.log"), "w", encoding="utf-8")


class Tee:
    def __init__(self, stream):
        self.stream = stream

    def write(self, s):
        self.stream.write(s)
        logf.write(s)
        logf.flush()

    def flush(self):
        self.stream.flush()


sys.stdout = Tee(sys.stdout)

meta = {
    "started": time.strftime("%Y-%m-%d %H:%M:%S"),
    "llm_model": llm_model, "llm_base_url": llm_base,
    "grounding_url": grounding_url,
    "max_step_size": args.max_step_size, "alt_floor": args.alt_floor,
    "episodes": args.episodes, "upstream": SR.__file__,
    "deviations": ["D6/D9/D10 llm", "D17/D18 grounding service", "D20 512x512", "D2 scene stub",
                   "alt_floor=" + args.alt_floor],
}
with open(os.path.join(out_dir, "run_meta.json"), "w", encoding="utf-8") as f:
    json.dump(meta, f, indent=2)

print("[run_legacy]", json.dumps(meta), flush=True)
SR.CityNavAgent(0, "val_seen", data_dir="./data", max_step_size=args.max_step_size,
                vlm_name=args.vlm_name, record=False)
print("[run_legacy] done", flush=True)
logf.close()
