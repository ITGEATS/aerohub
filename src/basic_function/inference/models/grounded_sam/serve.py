from aerohub_inference.app import create_app
from adapter import Predictor
from schemas import GreedyMaskRequest, GreedyMaskResponse

app = create_app(Predictor, GreedyMaskRequest, GreedyMaskResponse,
                 path='/models/grounded_sam/greedy_mask_predict')
