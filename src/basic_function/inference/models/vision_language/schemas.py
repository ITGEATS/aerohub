from pydantic import BaseModel, ConfigDict, Field


class InferRequest(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    request_id: str = Field(min_length=1, max_length=128)
    model_name: str = Field(min_length=1, max_length=128)
    prompt: str = Field(min_length=1, max_length=65536)
    image_base64: str = Field(min_length=1, max_length=16777216)
    stamp_ns: int = Field(ge=0)
    frame: str = Field(min_length=1, max_length=256)
    timeout: float = Field(gt=0)


class InferResponse(BaseModel):
    request_id: str
    model_name: str
    stamp_ns: int
    frame: str
    text: str
    error_code: str
    message: str
    latency_ms: float
