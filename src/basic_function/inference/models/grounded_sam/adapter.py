# SPDX-License-Identifier: GPL-3.0-or-later
"""Grounded-Sam greedy mask worker for the Aerohub Inference runtime.

`greedy_mask_predict` semantics are ported verbatim from CityNav's pinned
upstream src/tool/navigation/planner/citynav/upstream/external/
Grounded_Sam_Lite/grounded_sam_api.py (get_dino_output L55-87,
greedy_mask_predict L104-185): lowercase caption + trailing ".", box/text
threshold filtering, get_phrases_from_posmap phrase decoding, "."-split
in_phrases with soft-contain best_phrase selection, per-box SAM
predict_torch masks unioned, and the (small array, False) failure return.

Backend substitution (see README): the upstream CUDA custom-op GroundingDINO
build cannot run on this host (Blackwell sm_120, no nvcc/MSVC toolchain, no
CPU path for the custom kernels). The detector is the Hugging Face
transformers port GroundingDinoForObjectDetection (pure-PyTorch deformable
attention); the segmenter is the original facebookresearch segment_anything
SAM vit_h. No numerical cross-check against the original CUDA implementation
is possible on this machine.
"""
import base64
import hashlib
import json
import logging
import os
import time
from pathlib import Path

import cv2
import numpy as np
import torch
import yaml
from PIL import Image
from segment_anything import SamPredictor, sam_model_registry
from transformers import AutoProcessor, GroundingDinoForObjectDetection
from transformers.models.grounding_dino.processing_grounding_dino import get_phrases_from_posmap

MODEL_DIR = Path(__file__).resolve().parent
logger = logging.getLogger("grounded_sam.worker")


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
        config_path = _resolve(os.environ.get("GS_CONFIG", MODEL_DIR / "worker.yaml"))
        config = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
        device_text = str(os.environ.get("GS_DEVICE", config.get("device", "0")))
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

        record = json.loads((MODEL_DIR / "artifacts.json").read_text(encoding="utf-8"))
        sam_path = _resolve(config.get("sam_checkpoint", "weights/sam_vit_h_4b8939.pth"))
        if not sam_path.is_file() or _sha256(sam_path) != record["sam"]["sha256"]:
            raise ValueError(f"SAM checkpoint missing or checksum mismatch: {sam_path}")
        dino_dir = _resolve(config.get("dino_model", "artifacts/grounding-dino-swin-ogc"))
        dino_weights = dino_dir / "model.safetensors"
        if not dino_weights.is_file():
            raise ValueError(f"GroundingDINO files missing; run setup.sh first: {dino_dir}")
        dino_record = record["dino_swin_ogc"] if dino_dir.name == "grounding-dino-swin-ogc" else record["dino"]
        if dino_record.get("model_sha256") and _sha256(dino_weights) != dino_record["model_sha256"]:
            raise ValueError(f"GroundingDINO checksum mismatch: {dino_weights}")

        self.processor = AutoProcessor.from_pretrained(str(dino_dir), local_files_only=True)
        self.dino = GroundingDinoForObjectDetection.from_pretrained(
            str(dino_dir), local_files_only=True).to(device).eval()
        self.sam = SamPredictor(sam_model_registry["vit_h"](checkpoint=str(sam_path)).to(device))
        self.closed = False

        started = time.perf_counter()
        warmup = np.zeros((320, 320, 3), np.uint8)
        self.get_dino_output(cv2.cvtColor(warmup, cv2.COLOR_BGR2RGB), "object.", 0.3, 0.25)
        self.sam.set_image(warmup)
        dummy = torch.tensor([[10.0, 10.0, 300.0, 300.0]])
        self.sam.predict_torch(point_coords=None, point_labels=None,
                               boxes=self.sam.transform.apply_boxes_torch(dummy, warmup.shape[:2]).to(device),
                               multimask_output=False)
        self.warmup_seconds = time.perf_counter() - started
        if device != "cpu" and torch.cuda.is_available():
            allocated = torch.cuda.memory_allocated() / 2 ** 20
            reserved = torch.cuda.memory_reserved() / 2 ** 20
            logger.info("warmup on %s done in %.2fs, cuda memory allocated %.0f MiB reserved %.0f MiB",
                        device, self.warmup_seconds, allocated, reserved)
            self.cuda_memory_mib = {"allocated": round(allocated), "reserved": round(reserved)}
        else:
            self.cuda_memory_mib = None
        version = {"dino": {"repo": record["dino"]["repo"], "sha256": _sha256(dino_weights)},
                   "sam": {"file": sam_path.name, "sha256": record["sam"]["sha256"]},
                   "torch": torch.__version__, "transformers": __import__("transformers").__version__,
                   "thresholds": [0.3, 0.25]}
        self.metadata = {
            "model_id": "grounded_sam",
            "model_version": hashlib.sha256(json.dumps(version, sort_keys=True).encode()).hexdigest(),
            "device": device,
            "processing": version,
            "encoding": "png_bgr8 or jpeg_bgr8 base64",
            "max_image": [4096, 4096],
            "warmup_seconds": round(self.warmup_seconds, 2),
            "cuda_memory_mib": self.cuda_memory_mib,
        }

    def get_dino_output(self, image, caption, box_threshold, text_threshold):
        caption = caption.lower()
        caption = caption.strip()
        if not caption.endswith("."):
            caption = caption + "."
        inputs = self.processor(images=[Image.fromarray(image)], text=[caption], return_tensors="pt")
        inputs = {key: value.to(self.device) for key, value in inputs.items()}
        with torch.no_grad():
            outputs = self.dino(**inputs)
        logits = outputs.logits.cpu().sigmoid()[0]  # (nq, 256)
        boxes = outputs.pred_boxes.cpu()[0]  # (nq, 4) cxcywh, normalized
        filt_mask = logits.max(dim=1)[0] > box_threshold
        logits_filt = logits.clone()[filt_mask]
        boxes_filt = boxes.clone()[filt_mask]
        input_ids = inputs["input_ids"].cpu()[0]
        pred_phrases = []
        for logit in logits_filt:
            token_ids = [[int(t) for t in row]
                         for row in get_phrases_from_posmap((logit > text_threshold)[None, :], input_ids)]
            pred_phrases.append(self.processor.batch_decode(token_ids, skip_special_tokens=True)[0])
        return boxes_filt, pred_phrases

    def _greedy(self, image, text_prompt, box_threshold, text_threshold):
        """Ported greedy_mask_predict; also returns phrases for the HTTP contract."""
        seg_success = True
        h, w = image.shape[:2]
        in_phrases = text_prompt.split(".")
        in_phrases = [inp.strip(" ") for inp in in_phrases]

        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        pred_boxes, pred_phrases = self.get_dino_output(image, text_prompt, box_threshold, text_threshold)
        pred_boxes = pred_boxes.cpu()

        if len(pred_phrases) == 0:
            seg_success = False
            return np.zeros_like((h, w), dtype=np.bool_), seg_success, pred_phrases

        best_phrase = "<inf>"
        best_bboxes = []

        def _soft_contain(ele, ele_list):
            res = False
            for e in ele_list:
                if ele in e:
                    res = True
                    break
            return res

        for inp in in_phrases:
            if _soft_contain(inp, pred_phrases):
                best_phrase = inp
                break

        # no object is segmented
        if best_phrase not in in_phrases:
            seg_success = False
            return np.zeros_like((h, w), dtype=np.bool_), seg_success, pred_phrases

        # collect boxes
        for i, pp in enumerate(pred_phrases):
            if best_phrase in pp:
                best_bboxes.append(pred_boxes[i:i + 1, :])

        boxes_filt = torch.cat(best_bboxes, dim=0)
        self.sam.set_image(image)
        for i in range(boxes_filt.size(0)):
            boxes_filt[i] = boxes_filt[i] * torch.Tensor([w, h, w, h])
            boxes_filt[i][:2] -= boxes_filt[i][2:] / 2
            boxes_filt[i][2:] += boxes_filt[i][:2]

        transformed_boxes = self.sam.transform.apply_boxes_torch(boxes_filt, image.shape[:2]).to(self.device)

        masks, _, _ = self.sam.predict_torch(
            point_coords=None,
            point_labels=None,
            boxes=transformed_boxes.to(self.device),
            multimask_output=False,
        )

        masks = masks.cpu()
        final_mask = torch.any(masks[0], dim=0).numpy()
        return final_mask, seg_success, pred_phrases

    def greedy_mask_predict(self, image, text_prompt, box_threshold=0.3, text_threshold=0.25):
        mask, seg_success, _ = self._greedy(image, text_prompt, box_threshold, text_threshold)
        return mask, seg_success

    def infer(self, body):
        if self.closed:
            raise RuntimeError("worker closed")
        start = time.perf_counter()
        raw = base64.b64decode(body["image_b64"], validate=True)
        if len(raw) < 8:
            raise ValueError("invalid image payload")
        image = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if image is None or min(image.shape[:2]) < 4 or max(image.shape[:2]) > 4096:
            raise ValueError("PNG/JPEG decoding failed or image out of bounds")
        mask, seg_success, phrases = self._greedy(
            image, body["text_prompt"], body["box_threshold"], body["text_threshold"])
        mask_b64 = ""
        if seg_success:
            single = (mask.astype(np.uint8)) * 255
            if single.ndim == 1:  # upstream failure-shaped tiny array, kept for parity
                single = single.reshape(1, -1)
            ok, png = cv2.imencode(".png", single)
            if not ok:
                raise ValueError("mask PNG encoding failed")
            mask_b64 = base64.b64encode(png.tobytes()).decode("ascii")
        timings_ms = {"worker_total": (time.perf_counter() - start) * 1000}
        logger.info("greedy_mask_predict prompt=%r seg_success=%s boxes=%d mask_area=%d timings=%s",
                    body["text_prompt"], seg_success, len(phrases),
                    int(mask.sum()) if seg_success else 0, {k: round(v, 1) for k, v in timings_ms.items()})
        return {"mask_b64": mask_b64, "seg_success": bool(seg_success), "phrases": phrases}

    def close(self):
        if self.closed:
            return
        self.closed = True
        for name in ("dino", "sam"):
            if hasattr(self, name):
                delattr(self, name)
        if self.device != "cpu":
            torch.cuda.empty_cache()
