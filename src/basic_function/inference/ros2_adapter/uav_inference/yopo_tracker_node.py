"""Expose the isolated YOPO-Tracker model worker through a typed ROS service."""

import base64
import math
from threading import Lock

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from uav_inference_interfaces.srv import InferYopoTracker

from .model_client import ModelClient, WorkerError
from .yopo_contract import stamp_ns


HEADS = dict(endstate=135, target=45, cost=15, score=15, duration=15)
RGBD_BYTES = 4 * 192 * 320 * 4


class YopoTrackerInference(Node):
    def __init__(self):
        super().__init__('yopo_tracker_inference')
        endpoint = self.declare_parameter('endpoint', 'http://127.0.0.1:18082').value
        timeout = self.declare_parameter('timeout_s', 2.0).value
        self.max_age = self.declare_parameter('max_input_age_s', 0.3).value
        if not math.isfinite(timeout) or timeout <= 0 or not math.isfinite(self.max_age) or self.max_age < 0:
            raise ValueError('Expected a positive timeout and a nonnegative maximum input age')
        self.client = ModelClient(endpoint, timeout)
        self.lock = Lock()
        self.create_service(InferYopoTracker, 'inference/yopo_tracker/infer', self.infer,
                            callback_group=ReentrantCallbackGroup())

    def infer(self, request, response):
        response.request_id = request.request_id
        headers = request.rgb_header, request.depth_header, request.odometry_header
        response.rgb_header, response.depth_header, response.odometry_header = headers
        if not self.lock.acquire(blocking=False):
            response.error_code, response.message = 'BUSY', 'another request is active'
            return response
        try:
            stamps = [stamp_ns(header.stamp) for header in headers]
            now = self.get_clock().now().nanoseconds
            if (not all(header.frame_id for header in headers) or
                    not 0 < len(request.request_id) <= 128 or len(request.rgbd) != RGBD_BYTES or
                    not all(map(math.isfinite, request.observation))):
                raise ValueError('invalid RGB-D tensor, observation, request ID or frame')
            if max(stamps) - min(stamps) > 50_000_000 or max(stamps) > now:
                raise ValueError('measurement stamps are unsynchronized or in the future')
            if self.max_age and now - min(stamps) > self.max_age * 1e9:
                raise ValueError('measurement is stale')
            result = self.client.request('/infer', {
                'request_id': request.request_id,
                'rgbd': base64.b64encode(request.rgbd).decode('ascii'),
                'observation': [float(value) for value in request.observation],
            })
            try:
                if result['request_id'] != request.request_id or result['device'] != 'cuda':
                    raise ValueError('worker identity mismatch')
                digest = result['weight_sha256']
                if len(digest) != 64 or not all(c in '0123456789abcdef' for c in digest):
                    raise ValueError('invalid worker weight digest')
                for field, size in HEADS.items():
                    values = result[field]
                    if len(values) != size or not all(map(math.isfinite, values)):
                        raise ValueError(f'invalid {field} head')
                    setattr(response, field, [float(value) for value in values])
                response.weight_sha256, response.device = digest, result['device']
            except (KeyError, TypeError, ValueError, AssertionError, OverflowError) as error:
                raise WorkerError('INVALID_RESULT', str(error)) from error
            response.success = True
        except ValueError as error:
            response.error_code, response.message = 'INVALID_INPUT', str(error)
        except WorkerError as error:
            response.error_code, response.message = error.code, str(error)
        finally:
            self.lock.release()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = YopoTrackerInference()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
