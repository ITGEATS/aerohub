"""AeroVLA Houmo adapter: prepares paths and constructs the backend only.

CPU preprocessing and request assembly live in the shared model layer
(``processing.py``/``predictor.py``); this module resolves the deployment
directory (the board runtime holding ``models/``, ``assets/`` and
``artifacts.json``) and builds the backend, mirroring the YOPO houmo layout.
"""

import os
from pathlib import Path

from backend_aerovla import AeroVlaBackend
from predictor import AerovlaPredictor


class Predictor(AerovlaPredictor):
    def __init__(self):
        here = Path(__file__).resolve().parent
        infer_dir = os.environ.get("AEROVLA_INFER_DIR", str(here / "aerovla_infer"))
        backend = AeroVlaBackend(
            infer_dir,
            device_id=int(os.environ.get("AEROVLA_DEVICE_ID", "0")),
            verify_sha=os.environ.get("AEROVLA_VERIFY_SHA") == "1",
        )
        super().__init__(backend)
