from typing import List
from pydantic import BaseModel, ConfigDict, Field


class ClipScoreRequest(BaseModel):
    model_config = ConfigDict(extra='forbid')
    images_b64: List[str] = Field(min_length=1, max_length=16)
    texts: List[str] = Field(min_length=1, max_length=16)


class ClipScoreResponse(BaseModel):
    similarity: List[List[float]]
    model_version: str
