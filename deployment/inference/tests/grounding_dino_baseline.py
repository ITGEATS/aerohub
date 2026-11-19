"""Real CUDA direct official Transformers vs migrated full-output comparison."""
import argparse
import base64
import hashlib
import json
import io
from pathlib import Path
import sys
import time

import numpy as np
from PIL import Image
import torch
from transformers import AutoProcessor, GroundingDinoForObjectDetection

ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / 'src/basic_function/inference/models/grounding_dino'
sys.path.insert(0, str(MODEL))
from adapter import Predictor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = MODEL / 'artifacts/snapshot'
    torch.set_num_threads(2)
    torch.backends.cuda.matmul.allow_tf32 = False
    torch.backends.cudnn.allow_tf32 = False
    assert torch.cuda.is_available(), 'real GPU required'
    processor = AutoProcessor.from_pretrained(source, local_files_only=True)
    direct = GroundingDinoForObjectDetection.from_pretrained(source,
        local_files_only=True, use_safetensors=True, disable_custom_kernels=True).cuda().eval()
    image = Image.open(args.image).convert('RGB')
    png = io.BytesIO()
    image.save(png, format='PNG')
    prompt = 'car. truck. pickup truck.'
    inputs = processor(images=image, text=prompt, return_tensors='pt').to('cuda')
    with torch.inference_mode():
        direct(**inputs)
        torch.cuda.synchronize()
        started = time.perf_counter()
        reference = direct(**inputs)
        torch.cuda.synchronize()
        direct_ms = (time.perf_counter()-started)*1000
    decoded = processor.post_process_grounded_object_detection(reference, inputs.input_ids,
        box_threshold=.30, text_threshold=.25, target_sizes=[(image.height, image.width)])[0]
    raw = {k: getattr(reference, k).cpu().numpy() for k in ('logits', 'pred_boxes')}
    reference_boxes = [dict(xyxy=b.tolist(), confidence=float(s), category=p)
        for b, s, p in zip(decoded['boxes'].cpu(), decoded['scores'].cpu(), decoded['labels'])]
    del direct, reference, inputs
    torch.cuda.empty_cache()
    predictor = Predictor()
    try:
        feed = predictor.prepare(image, prompt)
        migrated, backend_ms = predictor.backend.infer(feed)
        for key in raw:
            np.testing.assert_allclose(migrated[key], raw[key], atol=1e-5, rtol=1e-5)
        body = dict(request_id='pickup-baseline', stamp_ns=1234567890, generation=7,
            frame='saved_camera_optical', width=image.width, height=image.height, prompt=prompt,
            box_threshold=.30, text_threshold=.25, encoding='png_rgb8',
            image_base64=base64.b64encode(png.getvalue()).decode())
        result = predictor.infer(body)
        assert len(result['boxes']) == len(reference_boxes)
        assert [x['category'] for x in result['boxes']] == [x['category'] for x in reference_boxes]
        np.testing.assert_allclose([x['xyxy'] for x in result['boxes']],
            [x['xyxy'] for x in reference_boxes], atol=.02, rtol=1e-5)
        np.testing.assert_allclose([x['confidence'] for x in result['boxes']],
            [x['confidence'] for x in reference_boxes], atol=1e-5, rtol=1e-5)
        np.savez_compressed(args.output.with_suffix('.npz'),
            **{'direct_'+k:v for k,v in raw.items()}, **{'migrated_'+k:v for k,v in migrated.items()})
        report = dict(passed=True, image_sha256=hashlib.sha256(args.image.read_bytes()).hexdigest(),
            metadata=predictor.metadata, direct_forward_ms=direct_ms, migrated_backend_ms=backend_ms,
            direct_boxes=reference_boxes, migrated=result, gpu=torch.cuda.get_device_name(),
            thresholds=dict(raw_atol=1e-5, raw_rtol=1e-5, pixel_atol=.02, confidence_atol=1e-5),
            limitation='same locked Transformers environment; not original 4.40.0.dev0 environment')
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps(dict(passed=True, boxes=result['boxes'], direct_forward_ms=direct_ms,
            migrated_worker_ms=result['timings_ms'])))
    finally:
        predictor.close()
        predictor.close()
    try:
        predictor.backend.infer(feed)
    except RuntimeError:
        pass
    else:
        raise AssertionError('closed backend accepted execution')


if __name__ == '__main__':
    main()
