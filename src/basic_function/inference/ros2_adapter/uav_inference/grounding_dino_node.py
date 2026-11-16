"""Typed Grounding DINO Engine facade; no Tool, TF, tracking or motion."""
import json
import threading
import time
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from uav_inference_interfaces.msg import Detection, DetectionFrame
from uav_inference_interfaces.srv import InferGroundingDino
from .grounding_dino_client import GroundingDinoClient, validate_parameters
from .yolo_contract import validate_request


class GroundingDinoNode(Node):
    def __init__(self):
        super().__init__('grounding_dino_inference_adapter')
        endpoint = self.declare_parameter('endpoint', 'http://127.0.0.1:8003').value
        timeout = self.declare_parameter('timeout_s', 10.0).value
        self.detector = GroundingDinoClient(endpoint, timeout)
        self.lock = threading.Lock()
        self.status = self.create_publisher(String, 'inference/grounding_dino/status', 10)
        self.create_service(InferGroundingDino, 'inference/grounding_dino/infer', self.infer,
                            callback_group=ReentrantCallbackGroup())
        self.create_timer(.5, self.health)

    def health(self):
        # Serialize identity initialization with service callbacks.
        if not self.lock.acquire(blocking=False):
            return
        try:
            metadata = self.detector.ready()
            status = dict(metadata, ready=True)
        except Exception as error:
            status = dict(ready=False, error=str(error))
        finally:
            self.lock.release()
        self.status.publish(String(data=json.dumps(status)))

    def infer(self, request, response):
        if not self.lock.acquire(blocking=False):
            response.error = 'BUSY'
            return response
        started = time.perf_counter()
        try:
            validate_parameters(request.prompt, request.box_threshold, request.text_threshold)
            image = validate_request(request)
            source = request.image
            result = self.detector.infer(image, request.request_id,
                source.header.stamp.sec * 1_000_000_000 + source.header.stamp.nanosec,
                source.header.frame_id, request.generation, request.prompt,
                request.box_threshold, request.text_threshold)
            response.result = DetectionFrame(header=source.header, request_id=request.request_id,
                generation=request.generation, worker_generation=result['worker_generation'],
                model_version=result['model_version'], width=source.width, height=source.height,
                valid=True, transform_json=json.dumps(dict(coordinates='original_pixels', masks='none',
                    clipping='none', category='grounded token phrase')),
                timings_json=json.dumps(dict(result['timings_ms'], adapter_total_ms=(time.perf_counter()-started)*1000)))
            response.result.detections = [Detection(xyxy=b['xyxy'], confidence=b['confidence'],
                category=b['category'], mask=Image(header=source.header)) for b in result['boxes']]
            for key in ('prompt', 'normalized_prompt', 'box_threshold', 'text_threshold'):
                setattr(response, key, result[key])
            response.success = True
        except Exception as error:
            response.error = str(error)
        finally:
            self.lock.release()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = GroundingDinoNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
