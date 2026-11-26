"""DepthPlanar ROS service for a single FlowPilot worker."""
import math
from http.client import HTTPException
import numpy as np
import rclpy
from rclpy.node import Node
from uav_inference_interfaces.srv import InferFlowPilot
from .model_client import ModelClient, WorkerError
from .flowpilot_contract import depth_m, request_payload


class FlowPilotService(Node):
    def __init__(self):
        super().__init__("flowpilot_inference")
        endpoint = self.declare_parameter("endpoint", "http://127.0.0.1:18082").value
        timeout = self.declare_parameter("timeout_s", 1.).value
        self.max_age = self.declare_parameter("max_input_age_s", 0.3).value
        self.max_skew = self.declare_parameter("max_input_skew_s", 0.05).value
        if any(not math.isfinite(value) or value <= 0 for value in (timeout, self.max_age, self.max_skew)):
            raise ValueError("timeout, input age and skew must be finite positive seconds")
        self.client = ModelClient(endpoint, timeout)
        self.service = self.create_service(InferFlowPilot, "inference/flowpilot/infer", self.infer)

    def infer(self, req, res):
        res.request_id = req.request_id
        res.observation_header = req.observation_header
        try:
            now = self.get_clock().now().nanoseconds
            payload = request_payload(req, now, self.max_age, self.max_skew)
            result = self.client.request("/infer", payload)
            wp = np.asarray(result["wp_body"], dtype=float)
            cp = np.asarray(result["ctrl_pts_body"], dtype=float)
            if result["request_id"] != req.request_id or wp.shape != (48,12) or cp.shape != (8,3) or not (
                    np.isfinite(wp).all() and np.isfinite(cp).all()):
                raise ValueError("invalid trajectory or request identity")
            res.waypoints, res.control_points = wp.ravel().tolist(), cp.ravel().tolist()
            res.success = True
        except (ValueError, KeyError, TypeError, WorkerError, HTTPException) as exc:
            res.message = str(exc)
        return res


def main():
    rclpy.init()
    node = FlowPilotService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
