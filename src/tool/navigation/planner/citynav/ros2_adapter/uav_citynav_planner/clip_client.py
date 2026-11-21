"""CLIP ROS client for scoring memory images against landmark text."""
from .grounding_client import ClientError
from .inference_client import call_inference

MAX_IMAGES_PER_REQUEST = 16


class ClipScoreClient:
    def __init__(self, node, timeout_s: float = 30.0):
        from uav_inference_interfaces.srv import InferClipScores

        self.request_type = InferClipScores.Request
        self.client = node.create_client(InferClipScores, 'inference/clip/score')
        self.timeout_s = timeout_s

    def _post(self, images_b64, texts):
        request = self.request_type(images_b64=list(images_b64), texts=list(texts),
                                    timeout=self.timeout_s)
        sim = list(call_inference(self.client, request, self.timeout_s).similarity)
        width = len(texts)
        if not width or len(sim) != len(images_b64) * width:
            raise ClientError("clip service bad similarity shape")
        return [sim[i:i + width] for i in range(0, len(sim), width)]

    def score(self, images_b64, texts):
        """→ similarity 矩阵（list[list[float]]，行序与 images_b64 一致）。"""
        rows = []
        for start in range(0, len(images_b64), MAX_IMAGES_PER_REQUEST):
            rows.extend(self._post(images_b64[start:start + MAX_IMAGES_PER_REQUEST],
                                   texts))
        return rows
