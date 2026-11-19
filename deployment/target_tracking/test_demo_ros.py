"""Actual generated ROS launch configuration, without sensors or flight."""
import unittest
import rclpy
from uav_inference.grounding_dino_node import GroundingDinoNode
from uav_inference_interfaces.srv import InferGroundingDino
from uav_object_detection.node import ObjectDetectionTool
from uav_target_tracking.node import TrackingNode


class DemoRosTests(unittest.TestCase):
    def test_demo_params_and_nodes(self):
        rclpy.init(args=['--ros-args', '-r', '__ns:=/demo_config_test',
            '--params-file', '/repo/deployment/target_tracking/params.yaml',
            '-p', 'use_sim_time:=false'])
        nodes = []
        try:
            for constructor in (GroundingDinoNode, ObjectDetectionTool, TrackingNode):
                nodes.append(constructor())
            engine, detector, tracker = nodes
            self.assertEqual(engine.detector.client.endpoint, 'http://127.0.0.1:8003')
            self.assertEqual(engine.detector.timeout, 10.)
            self.assertEqual(detector.backend_name, 'grounding_dino')
            self.assertIs(detector.backend.service_type, InferGroundingDino)
            self.assertEqual(detector.query, 'car')
            self.assertEqual(detector.timeout, 10.)
            self.assertEqual(tracker.get_parameter('snapshot_timeout').value, 1.)
            self.assertEqual(tracker.get_parameter('max_capture_age').value, .5)
            self.assertFalse(any('yolo' in name for name, _ in engine.get_node_names_and_namespaces()))
        finally:
            for node in reversed(nodes):
                if isinstance(node, ObjectDetectionTool):
                    node.stop_event.set(); node.worker.join(2)
                node.destroy_node()
            rclpy.try_shutdown()


if __name__ == '__main__':
    unittest.main()
