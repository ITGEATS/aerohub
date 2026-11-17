"""CLIP 评分服务客户端（纯 stdlib HTTP；服务由并行任务建设，契约固定）。

契约（任务规格锁定）：POST /models/clip/score，请求
{"images_b64": [...], "texts": [...]} → {"similarity": [[...]],
"model_version": str}，similarity[i][j]=第 i 图对第 j 文本的余弦相似度。
服务端 schema 限制单次 images/texts 各 ≤16，本客户端按 16 图分块串行请求。

用途：exploit 路径节点图 ×剩余 landmark 评分（memory.py
nodes_landmarks_similarity 的注入回调；D19/D23 见包 README）。
"""
import json
import urllib.error
import urllib.request

from .grounding_client import ClientError

MAX_IMAGES_PER_REQUEST = 16


class ClipScoreClient:
    def __init__(self, url: str, timeout_s: float = 30.0):
        self.url = url
        self.timeout_s = timeout_s

    def _post(self, images_b64, texts):
        payload = json.dumps({"images_b64": list(images_b64),
                              "texts": list(texts)}).encode("utf-8")
        req = urllib.request.Request(
            self.url, data=payload,
            headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=self.timeout_s) as resp:
                out = json.loads(resp.read().decode("utf-8"))
        except (urllib.error.URLError, OSError, ValueError) as exc:
            raise ClientError(f"clip service failure: {exc}") from exc
        sim = out.get("similarity")
        if not isinstance(sim, list) or len(sim) != len(images_b64):
            raise ClientError("clip service bad similarity shape")
        return sim

    def score(self, images_b64, texts):
        """→ similarity 矩阵（list[list[float]]，行序与 images_b64 一致）。"""
        rows = []
        for start in range(0, len(images_b64), MAX_IMAGES_PER_REQUEST):
            rows.extend(self._post(images_b64[start:start + MAX_IMAGES_PER_REQUEST],
                                   texts))
        return rows
