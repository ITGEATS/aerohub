"""Real DDS tests with a deterministic box-only Engine, not model validation."""
from array import array
import copy
import threading
import time
import unittest
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from uav_inference_interfaces.msg import Detection, DetectionFrame
from uav_inference_interfaces.srv import InferGroundingDino
from uav_object_detection_interfaces.srv import DetectSnapshot
from uav_object_detection.node import ObjectDetectionTool
from uav_object_detection.backends import make_backend, QueryError
from test_snapshot_ros import Fixture
import test_snapshot_ros


class DinoFixture(Fixture):
    def __init__(self):
        super().__init__()
        self.destroy_service(self.service)
        self.service = self.create_service(InferGroundingDino, 'inference/grounding_dino/infer', self.infer,
                                           callback_group=ReentrantCallbackGroup())
        self.version, self.worker, self.bad_identity = 'dino-fixture-v1', 11, False
        self.requests = []
        self.bad_prompt = False
        self.label = 'pickup truck'

    def infer(self, request, response):
        self.calls += 1
        self.requests.append(copy.deepcopy(request))
        time.sleep(self.delay)
        response.success = True
        response.prompt = response.normalized_prompt = request.prompt
        response.box_threshold, response.text_threshold = request.box_threshold, request.text_threshold
        response.result = DetectionFrame(header=request.image.header, request_id=request.request_id,
            generation=request.generation, worker_generation=self.worker, model_version=self.version,
            width=64,height=64,valid=True,timings_json='{}', detections=[Detection(
                category=self.label, confidence=.8, xyxy=[1.,1.,63.,63.], mask=Image(header=request.image.header))])
        if self.bad_prompt:
            response.normalized_prompt='wrong.'
        if self.bad_identity:
            response.result.request_id = 'wrong-request'
        return response


class BackendRosTest(unittest.TestCase):
    wait = staticmethod(test_snapshot_ros.SnapshotTest.wait)

    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args','-r','__ns:=/drone0','-p','registered_depth:=true',
                         '-p','detector_backend:=grounding_dino','-p','query:=car'])
        cls.fixture, cls.tool = DinoFixture(), ObjectDetectionTool()
        cls.client = cls.fixture.create_client(DetectSnapshot, 'tools/object_detection/observe',
                                               callback_group=ReentrantCallbackGroup())
        cls.executor = MultiThreadedExecutor(num_threads=8)
        cls.executor.add_node(cls.fixture); cls.executor.add_node(cls.tool)
        cls.thread = threading.Thread(target=cls.executor.spin,daemon=True);cls.thread.start()
        cls.wait(lambda:cls.client.service_is_ready() and cls.tool.client.service_is_ready())

    @classmethod
    def tearDownClass(cls):
        cls.tool.stop_event.set();cls.tool.worker.join(2)
        cls.executor.shutdown(timeout_sec=3);cls.thread.join(3)
        cls.tool.destroy_node();cls.fixture.destroy_node();rclpy.shutdown()

    def call(self, query=' Car ', position=False, timeout=1.):
        future=self.client.call_async(DetectSnapshot.Request(query=query, estimate_position=position, timeout=timeout))
        self.wait(future.done)
        return future.result()

    def tearDown(self):
        self.fixture.version, self.fixture.worker = 'dino-fixture-v1', 11
        self.fixture.bad_identity = False
        self.fixture.bad_prompt = False
        self.fixture.label = 'pickup truck'
        self.fixture.rgb_enabled = True
        self.wait(lambda:not self.tool.snapshot.gate.locked())

    def test_phrase_mismatch_preserved_and_normalized(self):
        for query in [' Car ', 'CAR.', 'pickup truck. car.']:
            response = self.call(query)
            self.assertTrue(response.success,response.reason)
            self.assertEqual(response.result.original_query,query)
            self.assertEqual(response.result.normalized_query,self.tool.backend.resolve(query))
            self.assertEqual(response.result.detections[0].detection.category,'pickup truck')
            self.assertEqual(response.result.worker_generation,11)
            self.assertEqual(response.result.model_version,'dino-fixture-v1')
            self.assertFalse(response.result.detections[0].detection.mask.data)
            self.assertEqual(self.fixture.requests[-1].prompt,self.tool.backend.resolve(query))
            self.assertFalse(hasattr(DetectSnapshot.Request(),'category'))

    def test_empty_and_arbitrary_query_not_yolo_fallback(self):
        calls=self.fixture.calls
        self.assertEqual(self.call('   ').code,'INVALID_QUERY')
        self.assertEqual(self.fixture.calls,calls)
        response=self.call('purple delivery vehicle')
        self.assertTrue(response.success,response.reason)
        self.assertEqual(self.fixture.requests[-1].prompt,'purple delivery vehicle.')

    def test_model_and_worker_changes_and_request_identity_rejected(self):
        self.assertTrue(self.call().success)
        for version,worker in [('changed',11),('dino-fixture-v1',12)]:
            self.fixture.version,self.fixture.worker=version,worker
            response=self.call()
            self.assertFalse(response.success)
            self.assertIn('MODEL_GENERATION_CHANGED',response.reason)
        self.fixture.version,self.fixture.worker='dino-fixture-v1',11
        self.fixture.bad_identity=True
        self.assertIn('identity/geometry mismatch',self.call().reason)

    def test_ambiguous_box_depth_is_partial_not_fake_mask(self):
        self.tool._ensure_position_inputs();self.wait(lambda:self.tool.projector.depth)
        response=self.call(position=True)
        self.assertTrue(response.success,response.reason)
        self.assertEqual(response.code,'PARTIAL')
        measured=response.result.detections[0]
        self.assertFalse(measured.position_valid)
        self.assertIn('background ambiguity',measured.position_error)
        self.assertEqual(measured.position_method,'box_foreground_surface_v1')

    def test_prompt_echo_mismatch_rejected_empty_phrase_preserved(self):
        self.fixture.bad_prompt=True
        self.assertIn('query/threshold mismatch',self.call().reason)
        self.fixture.bad_prompt=False;self.fixture.label=''
        response=self.call()
        self.assertTrue(response.success,response.reason)
        self.assertEqual(response.result.detections[0].detection.category,'')

    def test_old_stamp_after_acceptance_is_not_fresh_capture(self):
        self.fixture.rgb_enabled=False
        time.sleep(.1)
        image=Image(width=64,height=64,encoding='rgb8',step=192,data=bytes(64*64*3))
        image.header.frame_id='camera_optical'
        image.header.stamp=self.tool.get_clock().now().to_msg()
        image.header.stamp.sec-=1
        calls=self.fixture.calls
        future=self.client.call_async(DetectSnapshot.Request(query='car', timeout=.15))
        self.wait(lambda:self.tool.snapshot.waiting)
        for _ in range(3):self.fixture.rgb.publish(image);time.sleep(.01)
        self.wait(future.done)
        self.assertEqual(future.result().code,'TIMEOUT')
        self.assertEqual(self.fixture.calls,calls)


class QueryContractTest(unittest.TestCase):
    def test_yolo_exact_lookup_and_no_aliases(self):
        backend=make_backend('yolo')
        self.assertEqual(backend.resolve(' CAR '),'car')
        for query in ['cars','pickup truck','car.','unknown']:
            with self.assertRaises(QueryError) as result:backend.resolve(query)
            self.assertEqual(result.exception.code,'UNSUPPORTED_QUERY')
        for query in ['',' ',None,'\ncar']:
            with self.assertRaises(QueryError) as result:backend.resolve(query)
            self.assertEqual(result.exception.code,'INVALID_QUERY')
        with self.assertRaises(ValueError):make_backend('auto')

    def test_dino_dot_once(self):
        backend=make_backend('grounding_dino')
        for query in [' CAR ','car.']:
            self.assertEqual(backend.resolve(query),'car.')


if __name__=='__main__':unittest.main(verbosity=2)
