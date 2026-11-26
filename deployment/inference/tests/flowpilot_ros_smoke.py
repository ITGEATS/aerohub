"""ROS 2 service round trip with an HTTP fixture (no GPU or weights)."""
import base64
import json
import copy
import struct
import threading
import zlib
from http.server import BaseHTTPRequestHandler, HTTPServer

import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from uav_inference.flowpilot_node import FlowPilotService
from uav_inference_interfaces.srv import InferFlowPilot


class Fixture(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_POST(self):
        self.server.request_count += 1
        body = json.loads(self.rfile.read(int(self.headers["Content-Length"])))
        self.server.last_body = body
        assert self.path == "/infer"
        assert body["request_id"].startswith("smoke")
        encoded, shape = body.get("depth_b64_gz"), body.get("depth_shape")
        assert encoded is not None and shape == [2, 3], list(body)
        depth = np.frombuffer(zlib.decompress(base64.b64decode(encoded, validate=True)),
                              dtype="<f4").reshape(shape).tolist()
        assert depth == [[1., 2., 3.], [20., 5., 6.]], depth
        assert len(body["state_goal"]) == 25 and body["speed_mps"] == 2.0
        if self.server.fault == "rejected":
            self.send_error(422, "invalid depth")
            return
        if self.server.fault == "busy":
            self.send_error(429, "busy")
            return
        result = dict(request_id=body["request_id"],
                      wp_body=[[i + j / 10. for j in range(12)] for i in range(48)],
                      ctrl_pts_body=[[i + j / 10. for j in range(3)] for i in range(8)])
        if self.server.fault == "identity":
            result['request_id'] = 'wrong-request'
        if self.server.fault == "shape":
            result['ctrl_pts_body'] = [[0.] * 3] * 7
        data = json.dumps(result).encode()
        self.send_response(200)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data[:20] if self.server.fault == "short" else data)


def main():
    with HTTPServer(("127.0.0.1", 0), Fixture) as http:
        http.request_count = 0
        http.last_body = None
        http.fault = None
        http_thread = threading.Thread(target=http.serve_forever, daemon=True)
        http_thread.start()
        rclpy.init(args=["--ros-args", "-p", f"endpoint:=http://127.0.0.1:{http.server_port}",
                         "-p", "timeout_s:=2.0"])
        server = FlowPilotService()
        client_node = rclpy.create_node("flowpilot_smoke_client")
        client = client_node.create_client(InferFlowPilot, "inference/flowpilot/infer")
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(server)
        executor.add_node(client_node)
        try:
            assert client.wait_for_service(timeout_sec=8)
            def call(request):
                future = client.call_async(request)
                executor.spin_until_future_complete(future, timeout_sec=10)
                assert future.done(), 'ROS service did not return'
                return future.result()

            def fresh_request():
                fresh = copy.deepcopy(request)
                current = client_node.get_clock().now().to_msg()
                fresh.depth.header.stamp = current
                fresh.observation_header.stamp = current
                return fresh

            stamp = client_node.get_clock().now().to_msg()
            header = Header(stamp=stamp, frame_id="front_optical")
            request = InferFlowPilot.Request(request_id="smoke-1", depth=Image(
                header=header, height=2, width=3, encoding="32FC1", step=16,
                data=struct.pack("<8f", 1., 2., 3., 0., float("nan"), 5., 6., 0.)),
                observation_header=header, state_goal=[float(i) for i in range(25)], speed_mps=2.0)

            future = client.call_async(request)
            executor.spin_until_future_complete(future, timeout_sec=10)
            response = future.result()
            assert response.success, response.message
            assert response.request_id == "smoke-1" and response.observation_header == header
            assert len(response.waypoints) == 576 and len(response.control_points) == 24
            assert list(response.waypoints[:12]) == [0., .1, .2, .3, .4, .5, .6, .7, .8, .9, 1., 1.1]
            assert list(response.control_points[:3]) == [0., .1, .2]
            encoded = http.last_body["depth_b64_gz"]
            depth = np.frombuffer(zlib.decompress(base64.b64decode(encoded)), dtype="<f4")
            assert depth.reshape(2, 3).tolist()[1][0] == 20., \
                "invalid depth must be filled with 20 m"

            calls = http.request_count
            stale = copy.deepcopy(request)
            stale.request_id = "smoke-stale"
            stale.state_goal = request.state_goal
            stale.speed_mps = 2.0
            stale.depth.header.stamp.sec = max(0, stamp.sec - 10)
            stale.depth.header.stamp.nanosec = 0
            future = client.call_async(stale)
            executor.spin_until_future_complete(future, timeout_sec=10)
            assert not future.result().success and http.request_count == calls

            bad = fresh_request()
            bad.request_id = "smoke-encoding"
            bad.depth.encoding = "rgb8"
            bad.state_goal = request.state_goal
            bad.speed_mps = 2.0
            future = client.call_async(bad)
            executor.spin_until_future_complete(future, timeout_sec=10)
            assert not future.result().success and http.request_count == calls

            for value in (0., float('nan')):
                bad = fresh_request()
                bad.depth.data = struct.pack('<8f', *([value] * 8))
                assert not call(bad).success
                assert http.request_count == calls

            for fault in ("rejected", "busy", "short", "identity", "shape"):
                http.fault = fault
                assert not call(fresh_request()).success, fault
                http.fault = None
            assert call(fresh_request()).success, 'service must recover after a rejected request'
            print("FlowPilot ROS service fixture round trip passed.")
        finally:
            executor.remove_node(server)
            executor.remove_node(client_node)
            server.destroy_node()
            client_node.destroy_node()
            rclpy.shutdown()
            http.shutdown()
            http_thread.join(timeout=5)


if __name__ == "__main__":
    main()
