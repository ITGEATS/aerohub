"""Actual AS2 Module + ROS Service + RGB-D adapter, deterministic inference.

Synthetic images/depth/TF test contracts, not model accuracy or live sensors.
"""
import copy
from array import array
import threading
import time
import unittest
from unittest.mock import patch

import numpy as np
import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from std_srvs.srv import SetBool
from tf2_ros import StaticTransformBroadcaster
from uav_inference_interfaces.msg import Detection, DetectionFrame
from uav_inference_interfaces.srv import InferDetections
from uav_object_detection.node import ObjectDetectionTool
from uav_object_detection_interfaces.srv import DetectSnapshot


class Fixture(Node):
    def __init__(self):
        super().__init__('snapshot_fixture')
        self.rgb_enabled = self.depth_enabled = True
        self.delay = 0.
        self.empty = False
        self.calls = 0
        self.rgb = self.create_publisher(Image, 'sensor_measurements/FrontCamera/rgb/image', qos_profile_sensor_data)
        self.depth = self.create_publisher(Image, 'sensor_measurements/FrontCamera/depth_planar/image', qos_profile_sensor_data)
        self.rgb_info = self.create_publisher(CameraInfo, 'sensor_measurements/FrontCamera/rgb/camera_info', qos_profile_sensor_data)
        self.depth_info = self.create_publisher(CameraInfo, 'sensor_measurements/FrontCamera/depth_planar/camera_info', qos_profile_sensor_data)
        self.tf = StaticTransformBroadcaster(self)
        transform = TransformStamped()
        transform.header.frame_id = 'drone0/odom'
        transform.child_frame_id = 'camera_optical'
        transform.transform.rotation.w = 1.
        self.tf.sendTransform(transform)
        # Reentrant so a delayed inference does not stop fixture image publication.
        from rclpy.callback_groups import ReentrantCallbackGroup
        self.service = self.create_service(InferDetections, 'inference/yolo/infer', self.infer,
                                           callback_group=ReentrantCallbackGroup())
        self.timer = self.create_timer(.02, self.publish)

    def publish(self):
        stamp = self.get_clock().now().to_msg()
        rgb = Image(width=64, height=64, encoding='rgb8', step=192,
                    data=array('B', np.zeros((64,64,3), np.uint8).tobytes()))
        rgb.header.stamp, rgb.header.frame_id = stamp, 'camera_optical'
        info = CameraInfo(header=rgb.header, width=64, height=64,
                          k=[32.,0.,31.5,0.,32.,31.5,0.,0.,1.], d=[0.]*5)
        self.rgb_info.publish(info); self.depth_info.publish(info)
        if self.depth_enabled:
            self.depth.publish(Image(header=rgb.header, width=64, height=64, encoding='32FC1', step=256,
                                     data=array('B', np.full((64,64),2.,np.float32).tobytes())))
        if self.rgb_enabled:
            self.rgb.publish(rgb)

    def infer(self, request, response):
        self.calls += 1
        time.sleep(self.delay)
        frame = DetectionFrame(header=request.image.header, request_id=request.request_id,
            generation=request.generation, worker_generation=1, model_version='fixture',
            width=64, height=64, valid=True, timings_json='{}')
        if not self.empty:
            for category in ('person', 'car'):
                frame.detections.append(Detection(category=category, confidence=.9, xyxy=[1.,1.,63.,63.],
                    mask=Image(header=request.image.header,width=64,height=64,encoding='mono8',step=64,
                               data=array('B', np.ones((64,64),np.uint8).tobytes()))))
        response.success, response.result = True, frame
        return response


class SnapshotTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init(args=['--ros-args','-r','__ns:=/drone0','-p','registered_depth:=true'])
        cls.fixture, cls.tool = Fixture(), ObjectDetectionTool()
        cls.executor = MultiThreadedExecutor(num_threads=8)
        cls.executor.add_node(cls.fixture); cls.executor.add_node(cls.tool)
        cls.thread = threading.Thread(target=cls.executor.spin,daemon=True); cls.thread.start()
        cls.drone = DroneInterfaceBase('drone0'); cls.drone.load_module('object_detection')
        cls.module = cls.drone.object_detection
        cls.wait(lambda: cls.module.ready and cls.tool.client.service_is_ready())
        time.sleep(.2)

    @staticmethod
    def wait(predicate, timeout=5.):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline: raise AssertionError('wait timed out')
            time.sleep(.005)

    def tearDown(self):
        self.fixture.delay=0.;self.fixture.empty=False
        self.fixture.rgb_enabled=self.fixture.depth_enabled=True
        self.tool.enable(SetBool.Request(data=True),SetBool.Response())
        self.wait(lambda:not self.tool.snapshot.gate.locked())
        time.sleep(.15)

    def test_2d_without_depth_calibration_or_tf(self):
        self.assertIsNone(self.tool.projector)
        self.assertIsNone(self.tool.tf_buffer)
        self.fixture.depth_enabled=False
        for category in ('person','car'):
            response=self.module.detect(category, timeout=1.)
            self.assertTrue(response.success, response.reason)
            self.assertEqual(response.code,'SUCCEEDED')
            self.assertFalse(response.result.position_requested)
            self.assertFalse(response.result.detections[0].position_valid)
            self.assertEqual(response.result.detections[0].position_error,'')
            self.assertEqual(response.result.depth_header.stamp.sec,0)
            self.assertEqual(response.result.header,response.result.image_header)
            self.assertEqual(response.result.header.frame_id,'camera_optical')
        self.assertIsNone(self.tool.projector)
        self.assertIsNone(self.tool.tf_buffer)

    def test_query_and_valid_position(self):
        self.tool._ensure_position_inputs()
        self.wait(lambda:self.tool.projector.depth)
        time.sleep(.1)
        for category in ('person','car'):
            response=self.module.detect(category,timeout=1.,estimate_position=True)
            self.assertTrue(response.success,response.reason)
            self.assertEqual(response.code,'SUCCEEDED')
            self.assertEqual([d.detection.category for d in response.result.detections],[category])
            self.assertTrue(response.result.position_requested)
            self.assertTrue(response.result.detections[0].position_valid)
            self.assertAlmostEqual(response.result.detections[0].position.z,2.)
            stamp=response.result.image_header.stamp
            self.assertGreater(stamp.sec*10**9+stamp.nanosec,0)
            self.assertEqual(response.result.depth_header.stamp,stamp)
            self.assertEqual(response.result.header.frame_id,'drone0/odom')

    def test_normalized_fixed_query_preserves_original_and_label(self):
        response=self.module.detect(query=' CAR ',timeout=1.)
        self.assertTrue(response.success,response.reason)
        self.assertEqual(response.result.original_query,' CAR ')
        self.assertEqual(response.result.normalized_query,'car')
        self.assertEqual(response.result.detector_backend,'yolo')
        self.assertEqual(response.result.detections[0].detection.category,'car')
        self.assertFalse(hasattr(DetectSnapshot.Request(),'category'))
        with self.assertRaises(ValueError):self.module.detect(query=' ')

    def test_no_detection_is_success(self):
        self.fixture.empty=True
        response=self.module.detect('person',1.)
        self.assertTrue(response.success,response.reason)
        self.assertFalse(response.result.detections)

    def test_missing_depth_preserves_2d(self):
        self.tool._ensure_position_inputs()
        self.fixture.depth_enabled=False
        time.sleep(.1)
        with self.tool.projector.lock:self.tool.projector.depth.clear()
        response=self.module.detect('person',1.,estimate_position=True)
        self.assertTrue(response.success,response.reason)
        self.assertTrue(response.result.position_requested)
        self.assertEqual(response.code,'PARTIAL')
        self.assertTrue(response.result.detections)
        self.assertFalse(response.result.detections[0].position_valid)
        self.assertTrue(response.result.detections[0].position_error)
        # The next call may opt back out even after 3D consumers were activated.
        with patch.object(self.tool.projector, 'project', side_effect=AssertionError('2D projected')):
            response=self.module.detect('person',1.)
        self.assertTrue(response.success,response.reason)
        self.assertEqual(response.code,'SUCCEEDED')
        self.assertFalse(response.result.position_requested)
        self.assertEqual(response.result.detections[0].position_error,'')

    def test_cached_rgb_never_reused(self):
        self.fixture.rgb_enabled=False;time.sleep(.1)
        calls=self.fixture.calls
        response=self.module.detect('person',.1)
        self.assertEqual(response.code,'TIMEOUT')
        self.assertEqual(self.fixture.calls,calls)

    def test_timeout_discards_late_and_busy(self):
        self.fixture.delay=.2
        calls=self.fixture.calls
        response=self.module.detect('person',.08)
        self.assertEqual(response.code,'TIMEOUT')
        self.assertGreater(self.fixture.calls,calls)
        response=self.module.detect('car',1.)
        self.assertEqual(response.code,'BUSY')
        self.wait(lambda:not self.tool.snapshot.gate.locked())
        self.fixture.delay=0.
        response=self.module.detect('car',1.)
        self.assertTrue(response.success,response.reason)
        self.assertEqual(response.result.detections[0].detection.category,'car')

    def test_stop_and_unsupported(self):
        with self.assertRaises(ValueError):self.module.detect(estimate_position='false')
        self.assertEqual(self.module.detect('elephant',1.).code,'UNSUPPORTED_QUERY')
        self.tool.enable(SetBool.Request(data=False),SetBool.Response())
        self.assertEqual(self.module.detect('person',1.).code,'STOPPED')

    def test_stop_invalidates_inflight_result(self):
        self.fixture.delay=.2
        calls=self.fixture.calls
        outcomes=[]
        thread=threading.Thread(target=lambda:outcomes.append(self.module.detect('person',1.)))
        thread.start()
        self.wait(lambda:self.fixture.calls>calls)
        self.tool.enable(SetBool.Request(data=False),SetBool.Response())
        thread.join(timeout=1.)
        self.assertFalse(thread.is_alive())
        self.assertEqual(outcomes[0].code,'STOPPED')
        self.wait(lambda:not self.tool.snapshot.gate.locked())

    @classmethod
    def tearDownClass(cls):
        cls.tool.stop_event.set();cls.tool.worker.join(timeout=2)
        cls.drone.shutdown();cls.drone.destroy_node()
        cls.executor.shutdown(timeout_sec=3);cls.thread.join(timeout=3)
        cls.tool.destroy_node();cls.fixture.destroy_node();rclpy.shutdown()


if __name__=='__main__':unittest.main(verbosity=2)
