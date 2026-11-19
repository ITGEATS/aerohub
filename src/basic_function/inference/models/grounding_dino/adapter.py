"""Official Transformers Grounding DINO on the shared named-tensor backend."""
import base64
import hashlib
import io
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
from PIL import Image
import torch
import transformers
from transformers import AutoProcessor, GroundingDinoConfig, GroundingDinoForObjectDetection
from safetensors.torch import load_file
from aerohub_inference.backends.torch import TorchBackend
from prepare_artifacts import ROOT, verify
from schemas import InferRequest


def normalize_prompt(prompt):
    text = prompt.lower().strip()
    return text if text.endswith('.') else text + '.'


class Network(torch.nn.Module):
    def __init__(self, config):
        super().__init__()
        self.model = GroundingDinoForObjectDetection(config)

    def load_state_dict(self, state_dict, strict=True):
        # The snapshot is the complete official model, without wrapper prefixes.
        # Safetensors omits tied aliases. Restore only aliases proven to be the
        # same Parameter in the unmodified official architecture, never missing
        # independent weights. Strict loading still rejects incomplete snapshots.
        state_dict = dict(state_dict)
        groups = {}
        for name, parameter in self.model.named_parameters(remove_duplicate=False):
            groups.setdefault(id(parameter), []).append(name)
        for names in groups.values():
            present = next((name for name in names if name in state_dict), None)
            if present is not None:
                for name in names:
                    state_dict.setdefault(name, state_dict[present])
        return self.model.load_state_dict(state_dict, strict=strict)

    def forward(self, **inputs):
        result = self.model(**inputs)
        return dict(logits=result.logits, pred_boxes=result.pred_boxes)


class Predictor:
    def __init__(self):
        torch.set_num_threads(2)
        torch.backends.cuda.matmul.allow_tf32 = False
        torch.backends.cudnn.allow_tf32 = False
        source = Path(os.environ.get('GROUNDING_DINO_MODEL', ROOT / 'artifacts/snapshot')).resolve()
        manifest = verify(source)
        self.processor = AutoProcessor.from_pretrained(source, local_files_only=True)
        config = GroundingDinoConfig.from_pretrained(source, local_files_only=True)
        # Use the official portable attention implementation, no custom CUDA build.
        config.disable_custom_kernels = True
        self.backend = TorchBackend(lambda: Network(config), source / 'model.safetensors',
            'grounding-dino-tiny',
            dict(pixel_values=dict(shape=[1, 3, None, None], dtype='float32'),
                 pixel_mask=dict(shape=[1, None, None], dtype='int64'),
                 input_ids=dict(shape=[1, None], dtype='int64'),
                 token_type_ids=dict(shape=[1, None], dtype='int64'),
                 attention_mask=dict(shape=[1, None], dtype='int64')),
            dict(logits=dict(shape=[1, config.num_queries, config.max_text_len], dtype='float32'),
                 pred_boxes=dict(shape=[1, config.num_queries, 4], dtype='float32')),
            device=os.environ.get('GROUNDING_DINO_DEVICE', 'cuda:0'), state_loader=load_file)
        semantics = dict(files=manifest['files'], transformers=transformers.__version__,
            torch=torch.__version__, preprocessing='official processor RGB full-image shortest800 longest1333',
            prompt='lowercase+strip+append-final-dot; reject >256 tokens, never truncate',
            postprocess='official grounded postprocess; strict > thresholds; original xyxy pixels; no NMS/no clipping/no masks',
            thresholds='per-request box_threshold and text_threshold, both required in (0,1)',
            disable_custom_kernels=True, tf32=False)
        self.metadata = dict(self.backend.metadata,
            model_version=hashlib.sha256(json.dumps(semantics, sort_keys=True).encode()).hexdigest(),
            worker_generation=time.time_ns(), processing=semantics)
        try:
            inputs = self.prepare(Image.new('RGB', (64, 64)), 'car.')
            self.backend.warmup(inputs)
        except BaseException:
            self.backend.close()
            raise

    def prepare(self, image, text):
        tokens = self.processor.tokenizer(text, return_tensors='np')
        if tokens['input_ids'].shape[1] > 256:
            raise ValueError('prompt exceeds 256 tokens; truncation is forbidden')
        batch = self.processor(images=image, text=text, return_tensors='np')
        return {k: np.asarray(batch[k]) for k in self.backend.inputs}

    def infer(self, body):
        started = time.perf_counter()
        body = InferRequest.model_validate(body).model_dump()
        raw = base64.b64decode(body['image_base64'], validate=True)
        if len(raw) < 24 or raw[:8] != b'\x89PNG\r\n\x1a\n':
            raise ValueError('expected PNG image')
        width, height = int.from_bytes(raw[16:20], 'big'), int.from_bytes(raw[20:24], 'big')
        if (width, height) != (body['width'], body['height']):
            raise ValueError('PNG dimensions differ from request')
        try:
            with Image.open(io.BytesIO(raw)) as image:
                if image.mode != 'RGB':
                    raise ValueError('expected RGB PNG without alpha')
                image.load()
                image = image.copy()
        except (OSError, SyntaxError) as error:
            raise ValueError('invalid PNG image') from error
        text = normalize_prompt(body['prompt'])
        inputs = self.prepare(image, text)
        prepared = time.perf_counter()
        outputs, timings = self.backend.infer(inputs)
        forwarded = time.perf_counter()
        result = self.processor.post_process_grounded_object_detection(
            SimpleNamespace(**{k: torch.from_numpy(v) for k, v in outputs.items()}),
            torch.from_numpy(inputs['input_ids']), box_threshold=body['box_threshold'],
            text_threshold=body['text_threshold'], target_sizes=[(height, width)])[0]
        boxes = [dict(xyxy=box.tolist(), confidence=float(score), category=phrase)
                 for box, score, phrase in zip(result['boxes'], result['scores'], result['labels'])]
        if any(not np.isfinite(b['xyxy']).all() or not np.isfinite(b['confidence']) for b in boxes):
            raise ValueError('nonfinite model output')
        finished = time.perf_counter()
        return dict(**{k: body[k] for k in ('request_id', 'stamp_ns', 'generation', 'frame',
            'width', 'height', 'prompt', 'box_threshold', 'text_threshold')}, normalized_prompt=text,
            model_version=self.metadata['model_version'], worker_generation=self.metadata['worker_generation'],
            boxes=boxes, timings_ms=dict(preprocess=(prepared-started)*1000,
                backend_total=(forwarded-prepared)*1000, postprocess=(finished-forwarded)*1000,
                worker_total=(finished-started)*1000, **timings))

    def close(self):
        self.backend.close()
