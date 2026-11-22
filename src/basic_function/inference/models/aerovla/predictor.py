"""AeroVLA request pipeline: raw views + prompt ids -> model text output.

The predictor owns request validation and preprocessing; the backend owns the
NPU modules and the autoregressive loop. Action dequantization and the landing
decision are NOT parsed here: the planner consumes ``text`` verbatim so the
upstream parsing rules stay in the Tool layer (migration ruling 2026-11-19).
"""

import time

from processing import PREPROCESSING_VERSION, decode_image, decode_text, prep_mosaic


class AerovlaPredictor:
    def __init__(self, backend):
        self.backend = backend
        self.metadata = dict(
            backend.metadata,
            preprocessing_version=PREPROCESSING_VERSION,
        )
        # Partial warmup: the vision module is loaded onto the device and run
        # once with a zero mosaic. The LLM prefill/decode path is exercised by
        # the first real request; full-sequence warmup would need a pinned
        # tokenized prompt and is recorded as a boundary, not silently skipped.
        try:
            self.backend.warmup()
        except Exception:
            self.backend.close()
            raise

    def close(self):
        self.backend.close()

    def infer(self, request):
        started = time.perf_counter()
        front = decode_image(request["front_b64"])
        down = decode_image(request["down_b64"])
        pixel_values = prep_mosaic(front, down)
        prepared = time.perf_counter()
        gen_ids = self.backend.generate(list(request["input_ids"]), pixel_values)
        generated = time.perf_counter()
        text = decode_text(gen_ids, self.backend.id2token)
        finished = time.perf_counter()
        return dict(
            request_id=request.get("request_id", ""),
            model_version=self.metadata["model_version"],
            device=self.metadata["device"],
            text=text,
            gen_ids=[int(t) for t in gen_ids],
            timings_ms=dict(
                preprocess=(prepared - started) * 1000,
                forward_and_generate=(generated - prepared) * 1000,
                decode=(finished - generated) * 1000,
                total=(finished - started) * 1000,
            ),
        )
