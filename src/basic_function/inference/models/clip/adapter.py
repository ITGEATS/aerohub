# SPDX-License-Identifier: GPL-3.0-or-later
"""CLIP ViT-L/14 image-text similarity worker for the Aerohub Inference runtime.

Consumption semantics follow CityNav's pinned upstream
src/tool/navigation/planner/citynav/upstream/external/lm_nav/optimal_route.py
`nodes_landmarks_similarity` (L40-76): the caller prefixes texts with
"A photo of ", passes several images per node, and takes the per-node max over
the returned similarity matrix. This service only returns the raw cosine
similarity matrix similarity[i][j] = <image_i, text_j>; it does NOT apply the
"A photo of " prefix, the max reduction, or `rectify_and_crop_v2` preprocessing
(deviation D23: original frames are scored directly).

Backend: Hugging Face transformers CLIPModel/CLIPProcessor with the official
openai/clip-vit-large-patch14 port (same weights as upstream
clip.load("ViT-L/14")). HF CLIPProcessor performs the standard CLIP image
preprocessing (resize 224 shortest side, center crop, CLIP mean/std
normalization) and BPE tokenization truncated/padded to 77 tokens, mirroring
the openai-clip preprocess/tokenize chain.
"""
import base64
import hashlib
import io
import json
import logging
import os
import time
from pathlib import Path

import numpy as np
import torch
import yaml
from PIL import Image
from transformers import CLIPModel, CLIPProcessor

MODEL_DIR = Path(__file__).resolve().parent
logger = logging.getLogger("clip.worker")
MAX_IMAGE_PX = 4096


def _resolve(path_text):
    path = Path(path_text)
    return path if path.is_absolute() else MODEL_DIR / path


def _sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


class Predictor:
    def __init__(self):
        config_path = _resolve(os.environ.get("CLIP_CONFIG", MODEL_DIR / "worker.yaml"))
        config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
        device_text = str(os.environ.get("CLIP_DEVICE", config.get("device", "0")))
        if device_text != "cpu" and not device_text.replace("cuda:", "").isdigit():
            raise ValueError(f"unsupported device: {device_text}")
        if device_text == "cpu":
            device = "cpu"
        else:
            index = int(device_text.replace("cuda:", ""))
            if not torch.cuda.is_available():
                raise RuntimeError(f"device error: CUDA device {index} requested but torch.cuda.is_available() is False")
            if index >= torch.cuda.device_count():
                raise RuntimeError(
                    f"device error: requested cuda:{index}, only {torch.cuda.device_count()} CUDA device(s) visible")
            device = f"cuda:{index}"
        self.device = device

        record = json.loads((MODEL_DIR / "artifacts.json").read_text(encoding="utf-8"))["clip"]
        model_dir = _resolve(os.environ.get("CLIP_MODEL_DIR", config.get("model", "artifacts/clip-vit-large-patch14")))
        weights = model_dir / "model.safetensors"
        if not weights.is_file():
            raise ValueError(f"CLIP files missing; run setup.sh first: {model_dir}")
        if _sha256(weights) != record["model_sha256"]:
            raise ValueError(f"CLIP checksum mismatch: {weights}")

        self.processor = CLIPProcessor.from_pretrained(str(model_dir), local_files_only=True)
        self.model = CLIPModel.from_pretrained(str(model_dir), local_files_only=True).to(device).eval()
        self.closed = False

        started = time.perf_counter()
        warmup_image = Image.fromarray(np.zeros((64, 64, 3), np.uint8))
        self.score([warmup_image], ["a photo of an object"])
        self.warmup_seconds = time.perf_counter() - started
        if device != "cpu" and torch.cuda.is_available():
            allocated = torch.cuda.memory_allocated() / 2 ** 20
            reserved = torch.cuda.memory_reserved() / 2 ** 20
            logger.info("warmup on %s done in %.2fs, cuda memory allocated %.0f MiB reserved %.0f MiB",
                        device, self.warmup_seconds, allocated, reserved)
            self.cuda_memory_mib = {"allocated": round(allocated), "reserved": round(reserved)}
        else:
            self.cuda_memory_mib = None
        version = {"repo": record["repo"], "revision": record["revision"],
                   "model_sha256": record["model_sha256"],
                   "torch": torch.__version__,
                   "transformers": __import__("transformers").__version__}
        self.metadata = {
            "model_id": "clip",
            "model_version": hashlib.sha256(json.dumps(version, sort_keys=True).encode()).hexdigest(),
            "device": device,
            "processing": version,
            "encoding": "png/jpeg base64 (RGB decoded by PIL)",
            "max_image": [MAX_IMAGE_PX, MAX_IMAGE_PX],
            "max_images": 16,
            "max_texts": 16,
            "warmup_seconds": round(self.warmup_seconds, 2),
            "cuda_memory_mib": self.cuda_memory_mib,
        }

    @torch.no_grad()
    def score(self, images, texts):
        """Cosine similarity matrix (n_images, n_texts), float32 semantics like upstream."""
        image_inputs = self.processor(images=images, return_tensors="pt")
        image_inputs = {key: value.to(self.device) for key, value in image_inputs.items()}
        text_inputs = self.processor(text=texts, return_tensors="pt", padding=True, truncation=True)
        text_inputs = {key: value.to(self.device) for key, value in text_inputs.items()}
        image_features = self.model.get_image_features(**image_inputs).float()
        text_features = self.model.get_text_features(**text_inputs).float()
        image_features = image_features / image_features.norm(dim=-1, keepdim=True)
        text_features = text_features / text_features.norm(dim=-1, keepdim=True)
        return (image_features @ text_features.T).cpu().numpy()

    def infer(self, body):
        if self.closed:
            raise RuntimeError("worker closed")
        start = time.perf_counter()
        images = []
        for index, blob in enumerate(body["images_b64"]):
            try:
                raw = base64.b64decode(blob, validate=True)
            except Exception as error:
                raise ValueError(f"images_b64[{index}] is not valid base64") from error
            try:
                image = Image.open(io.BytesIO(raw)).convert("RGB")
            except Exception as error:
                raise ValueError(f"images_b64[{index}] PNG/JPEG decoding failed") from error
            if min(image.size) < 1 or max(image.size) > MAX_IMAGE_PX:
                raise ValueError(f"images_b64[{index}] out of bounds (<= {MAX_IMAGE_PX}px)")
            images.append(image)
        texts = []
        for index, text in enumerate(body["texts"]):
            if not text.strip():
                raise ValueError(f"texts[{index}] is blank")
            texts.append(text)
        similarity = self.score(images, texts)
        timings_ms = {"worker_total": (time.perf_counter() - start) * 1000}
        logger.info("score images=%d texts=%d timings=%s",
                    len(images), len(texts), {k: round(v, 1) for k, v in timings_ms.items()})
        return {"similarity": [[float(v) for v in row] for row in similarity],
                "model_version": self.metadata["model_version"]}

    def close(self):
        if self.closed:
            return
        self.closed = True
        for name in ("model",):
            if hasattr(self, name):
                delattr(self, name)
        if self.device != "cpu":
            torch.cuda.empty_cache()
