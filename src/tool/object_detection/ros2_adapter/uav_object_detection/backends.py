"""Deployment-selected detector adapters; query semantics stay out of the Tool node."""
import math
from uav_inference_interfaces.srv import InferDetections, InferGroundingDino
from uav_inference.yolo_contract import validate_frame


class QueryError(ValueError):
    def __init__(self, code, reason):
        super().__init__(reason)
        self.code = code


def normalize_query(query):
    if (not isinstance(query, str) or not query.strip() or len(query) > 1024
            or any(ord(c) < 32 for c in query)):
        raise QueryError('INVALID_QUERY', 'query must be nonempty text, at most 1024 characters without control characters')
    return query.strip().lower()


class YoloBackend:
    service_type = InferDetections
    service_name = 'inference/yolo/infer'
    status_topic = 'inference/yolo/status'
    position_method = 'yolo_mask_gemini2_reference'
    supported_queries = ('person', 'car', 'truck', 'bus')

    def resolve(self, query):
        normalized = normalize_query(query)
        if normalized not in self.supported_queries:
            raise QueryError('UNSUPPORTED_QUERY', 'deployed YOLO supports: '+', '.join(self.supported_queries))
        return normalized

    def request(self, task, query, *, tracking_stream=''):
        self.resolve(query)
        return self.service_type.Request(request_id=task.request_id, generation=task.generation,
                                         image=task.image, tracking_stream=tracking_stream)

    def frame(self, response, task, query):
        frame = response.result
        validate_frame(frame, task)
        if any(d.category not in self.supported_queries for d in frame.detections):
            raise ValueError('unsupported model label')
        frame.detections = [d for d in frame.detections if d.category == self.resolve(query)]
        return frame


class GroundingDinoBackend:
    service_type = InferGroundingDino
    service_name = 'inference/grounding_dino/infer'
    status_topic = 'inference/grounding_dino/status'
    position_method = 'box_foreground_surface_v1'

    def __init__(self, box_threshold=.30, text_threshold=.25):
        for value in (box_threshold, text_threshold):
            if not math.isfinite(value) or not 0 < value < 1:
                raise ValueError('DINO thresholds must be finite and in (0,1)')
        self.box_threshold, self.text_threshold = box_threshold, text_threshold

    def resolve(self, query):
        normalized = normalize_query(query)
        return normalized if normalized.endswith('.') else normalized+'.'

    def request(self, task, query):
        return self.service_type.Request(request_id=task.request_id, generation=task.generation,
            image=task.image, prompt=self.resolve(query), box_threshold=self.box_threshold,
            text_threshold=self.text_threshold)

    def frame(self, response, task, query):
        normalized = self.resolve(query)
        if (response.prompt != normalized or response.normalized_prompt != normalized or
                response.box_threshold != self.box_threshold or response.text_threshold != self.text_threshold):
            raise ValueError('inference query/threshold mismatch')
        # DINO produces unclipped boxes and optional empty token phrases, no masks.
        validate_frame(response.result, task, mask_required=False, allow_unclipped=True, allow_empty_label=True)
        return response.result


def make_backend(name, **kwargs):
    if name == 'yolo':
        return YoloBackend()
    if name == 'grounding_dino':
        return GroundingDinoBackend(**kwargs)
    raise ValueError('unknown detector_backend: '+str(name))
