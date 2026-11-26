from aerohub_inference.app import create_app
from adapter import Predictor
from schemas import Request, Response

app = create_app(Predictor, Request, Response)
