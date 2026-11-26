from typing import Annotated, List, Optional

from pydantic import BaseModel, ConfigDict, Field, model_validator

def vector(size):
    return Annotated[list[float], Field(min_length=size, max_length=size)]

class Request(BaseModel):
    """Inference request; the depth frame has two equivalent wire encodings.

    depth_m: JSON 2-D float array (metres), the original encoding.
    depth_b64_gz + depth_shape: base64(zlib(float32 row-major, little-endian))
    with [rows, cols] — the compact encoding used by the ROS adapter since
    2026-11-23 (remote-worker uplink budget, log/flowpilot-sitl-01).
    Exactly one representation is accepted per request.
    """
    model_config = ConfigDict(extra="forbid", allow_inf_nan=False)
    request_id: str = Field(min_length=1, max_length=128)
    depth_m: Optional[List[List[float]]] = Field(default=None, min_length=1, max_length=2048)
    depth_b64_gz: Optional[str] = Field(default=None, max_length=2_800_000)
    depth_shape: Optional[List[int]] = Field(default=None, min_length=2, max_length=2)
    state_goal: vector(25)
    speed_mps: float = Field(gt=0, le=10)

    @model_validator(mode="after")
    def exactly_one_depth(self):
        binary = self.depth_b64_gz is not None
        tabular = self.depth_m is not None
        if binary == tabular:
            raise ValueError("provide exactly one of depth_m or depth_b64_gz")
        if binary and self.depth_shape is None:
            raise ValueError("depth_shape is required with depth_b64_gz")
        if tabular and self.depth_shape is not None:
            raise ValueError("depth_shape is only valid with depth_b64_gz")
        return self

class Response(BaseModel):
    model_config = ConfigDict(allow_inf_nan=False)
    request_id: str
    wp_body: Annotated[list[vector(12)], Field(min_length=48, max_length=48)]
    ctrl_pts_body: Annotated[list[vector(3)], Field(min_length=8, max_length=8)]
