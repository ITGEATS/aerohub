"""AeroVLA CPU preprocessing shared by the board worker.

``prep_mosaic`` is a verbatim port of the validated board orchestrator
preprocessing (log/aerovla/board/orchestrator_board.py, 2026-10-18 round,
TravelUAV training convention): front and down views are resized to 224x224,
vertically stacked and squished back to 224x224, then duplicated under two
normalizations (CLIP mean/std and 0.5/0.5) into 6 CHW channels. The computation
moved from the caller into the model layer unchanged; the numbers are the
prismatic processor contract baked into aerovla_vision.hmm.
"""

import base64

import cv2
import numpy as np

PREPROCESSING_VERSION = "aerovla-mosaic6-squish-v1"

MEAN1 = np.array([0.484375, 0.455078125, 0.40625], np.float32)
STD1 = np.array([0.228515625, 0.2236328125, 0.224609375], np.float32)

IMAGE_TOKEN_TEXT = "<image>"


def decode_image(b64_text):
    """Decode a base64 PNG/JPEG buffer to a BGR uint8 image."""
    try:
        raw = base64.b64decode(b64_text, validate=True)
    except Exception as error:  # noqa: BLE001 - surface as a contract error
        raise ValueError("image is not valid base64: %s" % error) from error
    image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
    if image is None or image.ndim != 3 or image.shape[2] != 3:
        raise ValueError("image does not decode to a 3-channel BGR bitmap")
    return image


def prep_mosaic(front_bgr, down_bgr):
    """[1, 6, 224, 224] float32 tensor from the two BGR views (verbatim port)."""
    fr = cv2.resize(front_bgr, (224, 224), interpolation=cv2.INTER_CUBIC)
    dn = cv2.resize(down_bgr, (224, 224), interpolation=cv2.INTER_CUBIC)
    mos = cv2.resize(np.vstack([fr, dn]), (224, 224), interpolation=cv2.INTER_CUBIC)
    x = mos.astype(np.float32) / 255.0
    return np.concatenate([(x - MEAN1) / STD1, (x - 0.5) / 0.5], axis=-1).transpose(2, 0, 1)[None]


def decode_text(gen_ids, id2token):
    """Board runner decoding: concatenate mapped tokens, unmapped ids vanish."""
    return "".join(id2token.get(int(t), "") for t in gen_ids)
