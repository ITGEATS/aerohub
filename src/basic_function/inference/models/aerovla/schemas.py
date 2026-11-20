"""AeroVLA model contract (Houmo xh2 board worker).

Request carries the two raw observation images (front, down) as encoded PNG or
JPEG base64 plus pre-tokenized prompt ids; tokenization and prompt assembly
belong to the planner (Tool) layer, action dequantization and landing
decisions are parsed from ``text`` by the planner as well. The worker returns
the model's raw autoregressive output only.
"""

from typing import Dict, List

from pydantic import BaseModel, ConfigDict, Field, field_validator

MAX_INPUT_IDS = 512
MAX_IMAGE_B64 = 4 * 1024 * 1024  # 4 MiB base64 per view (256x256 PNG fits far below)


class InferRequest(BaseModel):
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    request_id: str = Field(default="", max_length=128)
    front_b64: str = Field(min_length=64, max_length=MAX_IMAGE_B64)
    down_b64: str = Field(min_length=64, max_length=MAX_IMAGE_B64)
    input_ids: List[int] = Field(min_length=1, max_length=MAX_INPUT_IDS)

    @field_validator("input_ids")
    @classmethod
    def check_ids(cls, ids):
        if any(not isinstance(v, int) or v < 0 or v > 32064 for v in ids):
            raise ValueError("input_ids must be vocab indices in 0..32064")
        return ids


class InferResponse(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    request_id: str
    model_version: str
    device: str
    text: str
    gen_ids: List[int]
    timings_ms: Dict[str, float]
