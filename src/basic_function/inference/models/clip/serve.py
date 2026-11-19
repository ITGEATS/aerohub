from aerohub_inference.app import create_app
from adapter import Predictor
from schemas import ClipScoreRequest, ClipScoreResponse

app = create_app(Predictor, ClipScoreRequest, ClipScoreResponse,
                 path='/models/clip/score')
