"""Serve the locked YOPO-Tracker network through the shared inference runtime."""

import base64
import hashlib
import os
from pathlib import Path
import sys

import numpy as np
from pydantic import BaseModel, Field
import torch

from aerohub_inference.app import create_app


MODEL_DIR = Path(__file__).resolve().parent
IMAGE_SHAPE = (1, 4, 192, 320)
STATE_SHAPE = (1, 6, 3, 5)
HEADS = ('endstate', 'target', 'cost', 'score', 'duration')


class InferRequest(BaseModel):
    request_id: str = Field(min_length=1, max_length=128)
    rgbd: str = Field(min_length=1_310_720, max_length=1_310_720)
    observation: list[float] = Field(min_length=90, max_length=90)


class InferResponse(BaseModel):
    request_id: str
    device: str
    weight_sha256: str
    endstate: list[float]
    target: list[float]
    cost: list[float]
    score: list[float]
    duration: list[float]


class Predictor:
    def __init__(self):
        root = Path(os.environ['YOPO_TRACKER_UPSTREAM']).resolve() / 'YOPO'
        sys.path.insert(0, str(root))
        from config.config import cfg
        from policy.yopo_network import YopoNetwork

        if not torch.cuda.is_available():
            raise RuntimeError('YOPO-Tracker worker requires CUDA')
        path = Path(os.environ.get('YOPO_TRACKER_WEIGHTS', MODEL_DIR / 'artifacts/epoch50.pth'))
        digest = hashlib.sha256(path.read_bytes()).hexdigest()
        if digest != (MODEL_DIR / 'weights.sha256').read_text().split()[0]:
            raise ValueError('Unexpected YOPO-Tracker checkpoint SHA256')
        cfg['train'] = False
        self.model = YopoNetwork()
        self.model.load_state_dict(torch.load(path, map_location='cpu', weights_only=True))
        self.model.to('cuda').eval()
        self.metadata = dict(device='cuda', weight_sha256=digest)
        with torch.inference_mode():
            self.model(torch.zeros(IMAGE_SHAPE, device='cuda'), torch.zeros(STATE_SHAPE, device='cuda'))
        torch.cuda.synchronize()

    @torch.inference_mode()
    def infer(self, body):
        image = np.frombuffer(base64.b64decode(body['rgbd'], validate=True), dtype='<f4').copy()
        observation = np.asarray(body['observation'], dtype=np.float32)
        heads = self.model(torch.from_numpy(image.reshape(IMAGE_SHAPE)).to('cuda'),
                           torch.from_numpy(observation.reshape(STATE_SHAPE)).to('cuda'))
        values = {name: tensor.flatten().cpu().tolist() for name, tensor in zip(HEADS, heads)}
        return dict(request_id=body['request_id'], **self.metadata, **values)


app = create_app(Predictor, InferRequest, InferResponse)
