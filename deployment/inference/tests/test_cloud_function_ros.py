"""Actual Function -> public HTTP runtime -> Cloud plugin -> local protocol peer.

Uses synthetic RGB and a local fake provider. SPF's actual prompt/text consumer is
exercised, but no real cloud, flight, GPU or simulator acceptance is claimed.
"""
import importlib.util
import json
import os
from pathlib import Path
import socket
import tempfile
import threading
import time
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from unittest.mock import patch

import rclpy
import uvicorn
import yaml
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from uav_inference_interfaces.action import InferVisionLanguage
from uav_inference.vision_language_node import VisionLanguageNode
from uav_spf_planner.prompt import build_prompt
from uav_spf_planner.vlm_client import VlmQueryClient
from aerohub_inference.app import create_app

ROOT = Path(__file__).resolve().parents[3]
MODEL = ROOT / 'src/basic_function/inference/models/vision_language'


def load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CloudFunctionContract(unittest.TestCase):
    def setUp(self):
        self.workspace = tempfile.TemporaryDirectory()
        self.entered = threading.Event()
        self.release = threading.Event()
        self.delay = 0.0
        self.requests = []
        self.bad_identity = False
        self.text = '[{"point":[500,500],"depth":5,"label":"ball"}]'
        owner = self

        class Provider(BaseHTTPRequestHandler):
            def do_POST(self):
                body = json.loads(self.rfile.read(int(self.headers['Content-Length'])))
                owner.requests.append(body)
                owner.entered.set()
                owner.release.wait(3)
                time.sleep(owner.delay)
                self.send_response(200)
                self.end_headers()
                try:
                    self.wfile.write(json.dumps({'choices':[{'message':{'content':owner.text}}]}).encode())
                except BrokenPipeError:
                    pass

            def log_message(self, *args):
                pass

        self.provider = ThreadingHTTPServer(('127.0.0.1', 0), Provider)
        self.provider_thread = threading.Thread(target=self.provider.serve_forever, daemon=True)
        self.provider_thread.start()
        config = Path(self.workspace.name)/'bindings.yaml'
        config.write_text(yaml.safe_dump(dict(models={'test-vlm':dict(cloud=dict(
            provider='openai', model='actual-test-remote-name',
            base_url='http://127.0.0.1:{}'.format(self.provider.server_port),
            api_key_env='TEST_CLOUD_KEY', timeout_s=3., allow_http_localhost=True))})))
        self.env = patch.dict(os.environ, dict(INFERENCE_MODEL_CONFIG=str(config), INFERENCE_MODEL_NAMES='test-vlm', TEST_CLOUD_KEY='not-real'))
        self.env.start()
        adapter = load('vision_language_adapter', MODEL/'adapter.py')
        schemas = load('vision_language_schemas', MODEL/'schemas.py')
        self.app = create_app(adapter.Predictor, schemas.InferRequest, schemas.InferResponse)
        sock = socket.socket()
        sock.bind(('127.0.0.1', 0))
        self.port = sock.getsockname()[1]
        sock.close()
        self.web = uvicorn.Server(uvicorn.Config(self.app, host='127.0.0.1', port=self.port, log_level='error'))
        self.web_thread = threading.Thread(target=self.web.run, daemon=True)
        self.web_thread.start()
        deadline = time.monotonic()+5
        while not self.web.started and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertTrue(self.web.started)
        ros_args=['--ros-args', '-p', 'endpoint:=http://127.0.0.1:{}'.format(self.port)]
        if os.environ.get('INFERENCE_TEST_NAMESPACE'):
            ros_args += ['-r', '__ns:=/' + os.environ['INFERENCE_TEST_NAMESPACE']]
        rclpy.init(args=ros_args)
        self.function = VisionLanguageNode()
        self.driver = Node('cloud_function_driver')
        self.executor = MultiThreadedExecutor(num_threads=5)
        self.executor.add_node(self.function)
        self.executor.add_node(self.driver)
        self.ros_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.ros_thread.start()
        self.images = self.driver.create_publisher(Image, 'sensor_measurements/FrontCamera/rgb/image', 10)
        self.client = ActionClient(self.driver, InferVisionLanguage, 'inference/vision_language/infer')
        self.assertTrue(self.client.wait_for_server(timeout_sec=5))
        self.image_stamp = None
        self.publish_enabled = True
        self.bad_encoding = False
        self.driver.create_timer(.025, self.publish_image)
        deadline=time.monotonic()+3
        while self.function._frame is None and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertIsNotNone(self.function._frame)

    def tearDown(self):
        self.release.set()
        deadline=time.monotonic()+4
        while self.function._busy and time.monotonic()<deadline:
            time.sleep(.025)
        self.executor.shutdown(timeout_sec=5)
        self.ros_thread.join(timeout=2)
        self.function.close()
        self.client.destroy()
        self.function.destroy_node()
        self.driver.destroy_node()
        rclpy.shutdown()
        self.web.should_exit = True
        self.web_thread.join(timeout=5)
        self.provider.shutdown()
        self.provider.server_close()
        self.provider_thread.join()
        self.env.stop()
        self.workspace.cleanup()

    def publish_image(self):
        if not self.publish_enabled:
            return
        msg=Image(height=8,width=8,encoding='mono8' if self.bad_encoding else 'rgb8',step=28)
        msg.header.stamp=self.driver.get_clock().now().to_msg()
        msg.header.frame_id='drone0/FrontCamera_optical'
        # Padded rows deliberately exercise the stride contract.
        msg.data=bytes([100, 30, 10]*8+[0]*4)*8
        self.image_stamp=msg.header.stamp
        self.images.publish(msg)

    def wait(self, future, timeout=5):
        deadline=time.monotonic()+timeout
        while not future.done() and time.monotonic()<deadline:
            time.sleep(.01)
        self.assertTrue(future.done())
        return future.result()

    def start(self, name='test-vlm', age=4., timeout=3.):
        return self.wait(self.client.send_goal_async(InferVisionLanguage.Goal(
            model_name=name,prompt='generic image question',timeout=timeout,max_result_age=age)))

    def test_spf_consumes_generic_function_and_preserves_prompt_and_header(self):
        self.release.set()
        consumer=VlmQueryClient(self.driver,'test-vlm')
        self.assertTrue(consumer._client.wait_for_server(timeout_sec=3.))
        result=consumer.request('approach orange ball',3.)
        self.assertTrue(result['ok'], result)
        self.assertEqual(result['point'],[500,500])
        self.assertEqual(result['model'],'test-vlm')
        self.assertGreater(result['image_stamp'],0)
        self.assertEqual(self.requests[0]['model'],'actual-test-remote-name')
        self.assertEqual(self.requests[0]['messages'][0]['content'][0]['text'], build_prompt('approach orange ball'))
        consumer._client.destroy()

    def test_cancel_keeps_busy_until_real_call_finishes(self):
        handle=self.start()
        self.assertTrue(handle.accepted)
        self.assertTrue(self.entered.wait(2))
        self.wait(handle.cancel_goal_async())
        result=self.wait(handle.get_result_async()).result
        self.assertEqual(result.error_code,'CANCELED')
        self.assertFalse(self.start().accepted)
        self.assertTrue(self.app.state.busy)
        self.release.set()
        deadline=time.monotonic()+3
        while self.function._busy and time.monotonic()<deadline:
            time.sleep(.025)
        next_handle=self.start()
        self.assertTrue(next_handle.accepted)
        self.assertTrue(self.wait(next_handle.get_result_async()).result.success)

    def test_stale_result_is_not_restamped_or_executed(self):
        self.release.set()
        self.delay=.25
        handle=self.start(age=.1)
        result=self.wait(handle.get_result_async()).result
        self.assertEqual(result.error_code,'STALE_RESULT')
        self.assertFalse(result.success)
        self.assertEqual(result.observation_header.frame_id,'drone0/FrontCamera_optical')

    def test_unknown_model_does_not_call_provider(self):
        handle=self.start(name='not-deployed')
        result=self.wait(handle.get_result_async()).result
        self.assertEqual(result.error_code,'UNSUPPORTED_MODEL')
        self.assertFalse(self.requests)

    def test_stale_input_is_rejected_before_provider_call(self):
        self.publish_enabled = False
        time.sleep(.2)
        handle=self.start(age=.1)
        result=self.wait(handle.get_result_async()).result
        self.assertEqual(result.error_code,'STALE_INPUT')
        self.assertFalse(self.requests)

    def test_bad_encoding_is_rejected_before_provider_call(self):
        self.bad_encoding = True
        time.sleep(.1)
        handle=self.start()
        result=self.wait(handle.get_result_async()).result
        self.assertEqual(result.error_code,'INVALID_INPUT')
        self.assertFalse(self.requests)

    def test_timeout_does_not_claim_cloud_success(self):
        handle=self.start(timeout=.1)
        self.assertTrue(handle.accepted)
        result=self.wait(handle.get_result_async()).result
        self.assertEqual(result.error_code,'TIMEOUT')
        self.assertFalse(result.success)


if __name__=='__main__':
    unittest.main(verbosity=2)
