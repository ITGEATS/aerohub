from aerohub_inference.app import create_app
from adapter import Predictor
from schemas import InferRequest, InferResponse

app = create_app(Predictor, InferRequest, InferResponse)

# FastAPI's default error response echoes invalid NaN/Infinity inputs, which its
# JSON serializer cannot encode. Keep bounded validation diagnostics, not payloads.
from fastapi.exceptions import RequestValidationError
from fastapi.responses import JSONResponse


@app.exception_handler(RequestValidationError)
async def invalid_request(_request, error):
    return JSONResponse(status_code=422, content={'detail': [
        dict(loc=list(item['loc']), msg=item['msg'], type=item['type'])
        for item in error.errors()]})
