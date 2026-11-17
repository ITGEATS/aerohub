"""Convert the local SwinT-OGC checkpoint to Hugging Face transformers format.

Runs inside the model uv environment (torch + transformers). Uses the key
mapping of the official transformers conversion script, kept unmodified under
reference/convert_grounding_dino_to_hf_main.py (its "grounding-dino-tiny"
variant converts exactly groundingdino_swint_ogc.pth, with SwinT backbone
config embed_dim 96, depths [2,2,6,2], num_heads [3,6,12,24], window 7,
image_size 224; the GroundingDinoConfig defaults d_model 256 / 6 encoder and
decoder layers / 8 heads / 4 feature levels / two_stage match the original
GroundingDINO_SwinT_OGC.py). Only the checkpoint loading (local file instead
of torch.hub URL), the COCO verification download (skipped) and the hub push
(removed) differ from the upstream script.

Output: artifacts/grounding-dino-swin-ogc/ (model.safetensors + config.json
from model.save_pretrained, tokenizer/processor files copied from the base
port, which are plain bert-base-uncased + default image processor files).
Idempotent: a verified target with the pinned checksum is never rewritten.
"""
import hashlib
import json
import shutil
import sys
from pathlib import Path

MODEL = Path(__file__).resolve().parent
sys.path.insert(0, str(MODEL / "reference"))

import types  # noqa: E402,F401

import huggingface_hub.utils  # noqa: E402

# The transformers main script imports httpx via huggingface_hub.utils (for
# its COCO verification image, which this driver never calls) and newer hub
# versions no longer expose the alias. Provide a stub so the reference file
# stays unmodified.
if not hasattr(huggingface_hub.utils, "httpx"):
    huggingface_hub.utils.httpx = types.ModuleType("httpx")

import torch  # noqa: E402
from transformers import (  # noqa: E402
    AutoProcessor,
    GroundingDinoForObjectDetection,
)

import convert_grounding_dino_to_hf_main as conv  # noqa: E402

TARGET_DIRNAME = "grounding-dino-swin-ogc"
COPIED_FILES = ("preprocessor_config.json", "tokenizer_config.json",
                "tokenizer.json", "vocab.txt", "special_tokens_map.json")


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main():
    record = json.loads((MODEL / "artifacts.json").read_text(encoding="utf-8"))
    source = record["swint_reference"]
    pth = MODEL / "weights" / source["filename"]
    if not pth.is_file() or sha256(pth) != source["sha256"]:
        raise SystemExit(f"source checkpoint missing or checksum mismatch: {pth}")

    target = MODEL / "artifacts" / TARGET_DIRNAME
    pinned = record.get("dino_swin_ogc", {}).get("model_sha256")
    weights = target / "model.safetensors"
    if pinned and weights.is_file() and sha256(weights) == pinned:
        print(f"Verified existing {weights} ({pinned}); nothing to do")
        return

    # Fixed seed: the unused BERT pooler head is randomly initialized by the
    # HF constructor, and a deterministic artifact keeps the pinned sha256 valid.
    torch.manual_seed(0)
    config = conv.get_grounding_dino_config("grounding-dino-tiny")
    print(f"SwinT backbone config: embed_dim={config.backbone_config.embed_dim} "
          f"depths={config.backbone_config.depths} num_heads={config.backbone_config.num_heads} "
          f"window={config.backbone_config.window_size} image_size={config.backbone_config.image_size}")

    blob = torch.load(pth, map_location="cpu", weights_only=False)
    original = {k.replace("module.", ""): v for k, v in blob["model"].items()}
    new_state = dict(original)
    rename_keys = conv.create_rename_keys(original, config)
    absent = [src for src, _ in rename_keys if src not in new_state]
    for src, dest in rename_keys:
        if src in new_state:
            conv.rename_key(new_state, src, dest)
    for stage in (conv.read_in_q_k_v_encoder, conv.read_in_q_k_v_text_enhancer, conv.read_in_q_k_v_decoder):
        stage(new_state, config)
    if absent:
        print(f"warning: {len(absent)} mapped source keys absent from checkpoint, e.g. {absent[:5]}")

    model = GroundingDinoForObjectDetection(config)
    model.eval()
    missing_keys, unexpected_keys = model.load_state_dict(new_state, strict=False)
    print(f"load_state_dict: missing={len(missing_keys)} unexpected={len(unexpected_keys)}")
    # Same accepted residue as the official transformers conversion (its
    # grounding-dino-tiny artifact behaves identically):
    # - relative_position_index: derived Swin buffer, recomputed at HF init;
    # - label_enc: frozen COCO class lookup, unused by the open-vocab forward;
    # - text_backbone.pooler.*: BERT pooler head, not used by GroundingDINO;
    # - text_backbone.embeddings.position_ids: buffer rebuilt by HF BertEmbeddings.
    def allowed_missing(key):
        return key.endswith(".attention.self.relative_position_index")

    def allowed_unexpected(key):
        return (key.endswith(".attn.relative_position_index") or key == "label_enc.weight"
                or key.startswith("model.text_backbone.pooler.")
                or key == "model.text_backbone.embeddings.position_ids")

    bad_missing = [k for k in missing_keys if not allowed_missing(k)]
    bad_unexpected = [k for k in unexpected_keys if not allowed_unexpected(k)]
    if bad_missing or bad_unexpected:
        raise SystemExit(f"unexpected mapping residue: missing={bad_missing[:10]} unexpected={bad_unexpected[:10]}")

    temporary = MODEL / "artifacts" / (TARGET_DIRNAME + ".tmp")
    if temporary.exists():
        shutil.rmtree(temporary)
    temporary.mkdir(parents=True)
    model.save_pretrained(temporary, safe_serialization=True)
    base_dir = MODEL / "artifacts" / "grounding-dino-base"
    for name in COPIED_FILES:
        shutil.copy2(base_dir / name, temporary / name)

    # Reload and one CPU forward through the processor to prove the artifact is usable.
    # Only the real text-token columns are checked: the HF GroundingDINO port emits
    # -inf class scores for padded text positions (columns beyond the tokenized
    # length), identical to the official grounding-dino-base artifact; the worker's
    # sigmoid+max filtering treats those as zero and never consumes them.
    reloaded = GroundingDinoForObjectDetection.from_pretrained(temporary, local_files_only=True)
    processor = AutoProcessor.from_pretrained(temporary, local_files_only=True)
    import numpy as np
    from PIL import Image
    image = Image.fromarray(np.zeros((64, 64, 3), np.uint8))
    inputs = processor(images=[image], text=["a cat."], return_tensors="pt")
    with torch.no_grad():
        outputs = reloaded(**inputs)
    n_tokens = inputs["input_ids"].shape[-1]
    real_columns = outputs.logits[:, :, :n_tokens]
    if not torch.isfinite(real_columns).all():
        raise SystemExit("converted model produced non-finite logits on real text-token columns")
    reference = GroundingDinoForObjectDetection.from_pretrained(
        MODEL / "artifacts" / "grounding-dino-base", local_files_only=True)
    with torch.no_grad():
        base_out = reference(**inputs)
    base_columns = base_out.logits[:, :, :n_tokens]
    if not torch.isfinite(base_columns).all():
        raise SystemExit("reference base port produced non-finite logits; harness invalid")
    del reference
    print(f"forward check ok: logits {tuple(outputs.logits.shape)}, "
          f"real columns finite (padding columns -inf as in the official port)")

    digest = sha256(temporary / "model.safetensors")
    if target.exists():
        shutil.rmtree(target)
    temporary.replace(target)
    print(f"converted {pth.name} -> {target}")
    print(f"model.safetensors sha256: {digest}")


if __name__ == "__main__":
    main()
