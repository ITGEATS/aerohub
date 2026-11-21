"""Model-name routing for vision-language generation; no navigation interpretation."""
import os
import time
from pathlib import Path

from aerohub_inference.engines import create_engine
from aerohub_inference.cloud_config import read_model_catalog, select_model
from aerohub_inference.backends.cloud import CloudError


def read_bindings(path, model_names=None):
    catalog = read_model_catalog(path)
    names = ['vlm'] if model_names is None else model_names
    bindings = {name: select_model(catalog, name) for name in names}
    # Engine declarations are not implementations: there is no local SPF VLM yet.
    if any(binding['engine'] != 'cloud' for binding in bindings.values()):
        raise ValueError('vision-language adapter has no implementation for selected local engine')
    return bindings


class Predictor:
    def __init__(self):
        default = Path(__file__).resolve().parents[5] / 'config/inference.cloud.local.yaml'
        path = os.environ.get('INFERENCE_MODEL_CONFIG', str(default))
        names = os.environ.get('INFERENCE_MODEL_NAMES', 'vlm').split(',')
        self.engines = {}
        try:
            for name, binding in read_bindings(path, names).items():
                self.engines[name] = create_engine(binding['engine'], **binding['configuration'])
        except Exception:
            self.close()
            raise
        self.metadata = dict(model_names=list(self.engines),
                             engines={name: engine.metadata for name, engine in self.engines.items()})

    def infer(self, request):
        name = request['model_name']
        if name not in self.engines:
            raise ValueError('unsupported model_name')
        started = time.monotonic()
        result = dict(request_id=request['request_id'], model_name=name,
                      stamp_ns=request['stamp_ns'], frame=request['frame'], text='',
                      error_code='', message='')
        try:
            result['text'] = self.engines[name].infer(request['prompt'], request['image_base64'], request['timeout'])
        except CloudError as error:
            result.update(error_code=error.code, message=str(error))
        result['latency_ms'] = (time.monotonic() - started) * 1000
        return result

    def infer_frames(self, request):
        """Ordered caller-supplied frames; empty list is a text-only request."""
        name = request['model_name']
        if name not in self.engines:
            raise ValueError('unsupported model_name')
        started = time.monotonic()
        result = dict(request_id=request['request_id'], model_name=name,
                      images=request['images'], text='', error_code='', message='')
        try:
            frames = [(image['label'], image['image_base64'])
                      for image in request['images']]
            engine = self.engines[name]
            if not hasattr(engine, 'infer_frames'):
                raise ValueError('selected engine has no frames implementation')
            result['text'] = engine.infer_frames(request['prompt'], frames,
                                                 request['timeout'],
                                                 request.get('system_prompt', ''))
        except CloudError as error:
            result.update(error_code=error.code, message=str(error))
        result['latency_ms'] = (time.monotonic() - started) * 1000
        return result

    def close(self):
        for engine in self.engines.values():
            engine.close()
