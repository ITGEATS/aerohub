"""Identity-checked open-vocabulary HTTP client; ROS and Torch independent."""
import base64
import io
import math
import time
from PIL import Image
from .model_client import ModelClient


def validate_parameters(prompt, box_threshold, text_threshold):
    if (not isinstance(prompt, str) or not 0 < len(prompt) <= 1024 or
            not prompt.strip() or any(ord(c) < 32 for c in prompt)):
        raise ValueError('invalid prompt')
    for value in (box_threshold, text_threshold):
        if not math.isfinite(value) or not 0 < value < 1:
            raise ValueError('thresholds must be finite and in (0,1)')


class GroundingDinoClient:
    def __init__(self, endpoint, timeout):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout_s must be positive finite')
        self.client = ModelClient(endpoint, timeout)
        self.timeout = timeout
        self.identity = None

    def ready(self):
        metadata = self.client.request('/health/ready')
        if metadata.get('model_id') != 'grounding-dino-tiny':
            raise ValueError('wrong model worker')
        identity = (metadata['model_version'], metadata['worker_generation'])
        if not identity[0] or not isinstance(identity[1], int) or identity[1] <= 0:
            raise ValueError('invalid worker identity')
        if self.identity is None:
            self.identity = identity
        elif identity != self.identity:
            raise ValueError('worker changed; restart adapter and invalidate caller generation')
        return metadata

    def infer(self, image, request_id, stamp_ns, frame, generation, prompt, box_threshold, text_threshold):
        validate_parameters(prompt, box_threshold, text_threshold)
        self.ready()
        png = io.BytesIO()
        Image.fromarray(image[:, :, ::-1]).save(png, format='PNG')  # common converter returns BGR
        body = dict(request_id=request_id, stamp_ns=stamp_ns, frame=frame, generation=generation,
            width=image.shape[1], height=image.shape[0], encoding='png_rgb8',
            image_base64=base64.b64encode(png.getvalue()).decode('ascii'), prompt=prompt,
            box_threshold=float(box_threshold), text_threshold=float(text_threshold))
        started = time.monotonic()
        result = self.client.request('/infer', body)
        if time.monotonic() - started > self.timeout:
            raise ValueError('late result rejected')
        for key in ('request_id', 'stamp_ns', 'frame', 'generation', 'width', 'height',
                    'prompt', 'box_threshold', 'text_threshold'):
            if result.get(key) != body[key]:
                raise ValueError('response identity/parameters mismatch: ' + key)
        normalized = prompt.lower().strip()
        if not normalized.endswith('.'):
            normalized += '.'
        if result.get('normalized_prompt') != normalized:
            raise ValueError('response prompt semantics mismatch')
        if (result.get('model_version'), result.get('worker_generation')) != self.identity:
            raise ValueError('response worker identity mismatch')
        boxes = result.get('boxes')
        if not isinstance(boxes, list) or len(boxes) > 900:
            raise ValueError('invalid boxes')
        for box in boxes:
            xy = box['xyxy']
            if (len(xy) != 4 or not all(math.isfinite(v) for v in xy) or
                    not xy[0] < xy[2] or not xy[1] < xy[3] or
                    not math.isfinite(box['confidence']) or not 0 <= box['confidence'] <= 1 or
                    not isinstance(box['category'], str) or 'mask_png_base64' in box):
                raise ValueError('invalid grounded box')
        return result
