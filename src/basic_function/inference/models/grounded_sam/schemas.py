from typing import List
from pydantic import BaseModel, ConfigDict, Field


class GreedyMaskRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')
    image_b64: str = Field(min_length=1, max_length=16777216)
    text_prompt: str = Field(min_length=1, max_length=512)
    box_threshold: float = Field(default=0.3, ge=0, le=1)
    text_threshold: float = Field(default=0.25, ge=0, le=1)


class GreedyMaskResponse(BaseModel):
    mask_b64: str
    seg_success: bool
    phrases: List[str]
