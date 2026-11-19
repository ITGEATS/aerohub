"""Bounded, explicit open-vocabulary box inference contract."""
from typing import Dict, List, Literal
from pydantic import BaseModel, ConfigDict, Field, field_validator


class InferRequest(BaseModel):
    model_config = ConfigDict(extra='forbid', strict=True, allow_inf_nan=False)
    request_id: str = Field(min_length=1, max_length=128)
    stamp_ns: int = Field(gt=0)
    generation: int = Field(ge=0, lt=2**64)
    frame: str = Field(min_length=1, max_length=256)
    width: int = Field(gt=0, le=2048)
    height: int = Field(gt=0, le=2048)
    encoding: Literal['png_rgb8']
    image_base64: str = Field(min_length=1, max_length=16777216)
    prompt: str = Field(min_length=1, max_length=1024)
    box_threshold: float = Field(gt=0, lt=1)
    text_threshold: float = Field(gt=0, lt=1)

    @field_validator('prompt')
    @classmethod
    def valid_prompt(cls, value):
        if not value.strip() or any(ord(c) < 32 for c in value):
            raise ValueError('prompt must contain printable text')
        return value


class Box(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    xyxy: List[float] = Field(min_length=4, max_length=4)
    confidence: float = Field(ge=0, le=1)
    category: str  # Grounded token phrase, not a closed-set class ID.


class InferResponse(BaseModel):
    model_config = ConfigDict(extra='forbid', allow_inf_nan=False)
    request_id: str
    stamp_ns: int
    generation: int
    frame: str
    width: int
    height: int
    prompt: str
    normalized_prompt: str
    box_threshold: float
    text_threshold: float
    model_version: str
    worker_generation: int
    boxes: List[Box] = Field(max_length=900)
    timings_ms: Dict[str, float]
