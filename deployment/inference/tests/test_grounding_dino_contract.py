"""Failure semantics without a model or ROS import; image/backend checks run separately."""
import copy
from pathlib import Path
import sys
import time
import unittest
from unittest.mock import patch
import numpy as np

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/basic_function/inference/ros2_adapter'))
sys.path.insert(0, str(ROOT / 'src/basic_function/inference/models/grounding_dino'))
from uav_inference.grounding_dino_client import GroundingDinoClient
from schemas import InferRequest


class ContractTest(unittest.TestCase):
    def setUp(self):
        self.client = GroundingDinoClient('http://unused', .1)
        self.metadata = dict(model_id='grounding-dino-tiny', model_version='hash', worker_generation=1)
        self.mutate = lambda x: x
        def request(route, body=None):
            if body is None:
                return self.metadata.copy()
            result = {k:v for k,v in body.items() if k not in ('encoding', 'image_base64')}
            result.update(model_version='hash', worker_generation=1, normalized_prompt='car.', boxes=[], timings_ms={})
            return self.mutate(result)
        self.client.client.request = request

    def infer(self):
        return self.client.infer(np.zeros((4, 6, 3), np.uint8), 'test', 123, 'optical', 3, 'Car', .3, .25)

    def test_identity_and_changed_worker(self):
        self.assertEqual(self.infer()['normalized_prompt'], 'car.')
        for key in ('request_id', 'stamp_ns', 'generation', 'frame', 'width', 'height',
                    'model_version', 'worker_generation', 'prompt', 'normalized_prompt', 'box_threshold', 'text_threshold'):
            def mutate(value, key=key):
                value[key] = None
                return value
            self.mutate = mutate
            with self.assertRaises(ValueError, msg=key):
                self.infer()
        self.metadata['worker_generation'] = 2
        with self.assertRaisesRegex(ValueError, 'worker changed'):
            self.client.ready()

    def test_late_result_drained_then_recovery(self):
        def late(result):
            time.sleep(.12)
            return result
        self.mutate = late
        with self.assertRaisesRegex(ValueError, 'late result'):
            self.infer()
        self.mutate = lambda x:x
        self.infer()

    def test_schema_bounds(self):
        body = dict(request_id='x', stamp_ns=1, generation=0, frame='f', width=6, height=4,
            prompt='car.', box_threshold=.3, text_threshold=.25, encoding='png_rgb8', image_base64='x')
        InferRequest(**body)
        for key, values in dict(prompt=['', ' ', 'car\n', 'x'*1025],
                box_threshold=[0., 1., float('nan')], text_threshold=[-1., float('inf')],
                width=[0, 2049], encoding=['jpeg'], generation=[-1,2**64]).items():
            for value in values:
                bad = copy.copy(body)
                bad[key] = value
                with self.assertRaises(ValueError, msg=(key,value)):
                    InferRequest(**bad)


class ReadinessTest(unittest.IsolatedAsyncioTestCase):
    async def test_not_ready_before_load_and_after_close(self):
        from aerohub_inference.app import create_app
        from fastapi import HTTPException
        from schemas import InferResponse
        closed = []
        class Predictor:
            metadata = dict(model_id='grounding-dino-tiny')
            def close(self):
                closed.append(True)
        app = create_app(Predictor, InferRequest, InferResponse)
        ready = next(r.endpoint for r in app.routes if r.path == '/health/ready')
        with self.assertRaises(HTTPException) as error:
            await ready()
        self.assertEqual(error.exception.status_code, 503)
        async with app.router.lifespan_context(app):
            self.assertEqual((await ready())['model_id'], 'grounding-dino-tiny')
        with self.assertRaises(HTTPException) as error:
            await ready()
        self.assertEqual(error.exception.status_code, 503)
        self.assertEqual(closed, [True])


if __name__ == '__main__':
    unittest.main()
