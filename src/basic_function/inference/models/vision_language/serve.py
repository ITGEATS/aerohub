from aerohub_inference.app import create_app
from adapter import Predictor
from schemas import (InferFramesRequest, InferFramesResponse, InferRequest,
                     InferResponse)

app = create_app(
    Predictor, InferRequest, InferResponse,
    extra_routes=[('/infer_frames', InferFramesRequest, InferFramesResponse,
                   'infer_frames')])
