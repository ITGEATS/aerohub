"""ROS entry points for isolated Grounded SAM and CLIP workers."""
import math

import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from uav_inference_interfaces.srv import InferClipScores, InferGroundedSam

from .model_client import ModelClient, WorkerError


class ImageTextNode(Node):
    def __init__(self):
        super().__init__('image_text_inference')
        self.sam = self.declare_parameter('grounded_sam_endpoint', '').value
        self.clip = self.declare_parameter('clip_endpoint', '').value
        self.create_service(InferGroundedSam, 'inference/grounded_sam/infer',
                            self.segment, callback_group=MutuallyExclusiveCallbackGroup())
        self.create_service(InferClipScores, 'inference/clip/score',
                            self.score, callback_group=MutuallyExclusiveCallbackGroup())

    @staticmethod
    def request(endpoint, route, timeout, payload):
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError('timeout must be finite and positive')
        if not endpoint:
            raise WorkerError('NOT_READY', 'worker endpoint is not configured')
        return ModelClient(endpoint, timeout).request(route, payload)

    def segment(self, req, res):
        try:
            body = self.request(self.sam, '/models/grounded_sam/greedy_mask_predict',
                                req.timeout, dict(image_b64=req.image_b64,
                                text_prompt=req.text_prompt, box_threshold=req.box_threshold,
                                text_threshold=req.text_threshold))
            res.seg_success = body['seg_success']
            res.mask_b64 = body['mask_b64']
            res.success = True
        except WorkerError as exc:
            res.error_code, res.message = exc.code, str(exc)
        except (ValueError, TypeError, KeyError) as exc:
            res.error_code, res.message = 'INVALID_PAYLOAD', str(exc)
        return res

    def score(self, req, res):
        try:
            body = self.request(self.clip, '/models/clip/score', req.timeout,
                                dict(images_b64=list(req.images_b64), texts=list(req.texts)))
            rows = body['similarity']
            if len(rows) != len(req.images_b64) or any(len(row) != len(req.texts) for row in rows):
                raise ValueError('wrong similarity dimensions')
            res.similarity = [float(value) for row in rows for value in row]
            res.success = True
        except WorkerError as exc:
            res.error_code, res.message = exc.code, str(exc)
        except (ValueError, TypeError, KeyError) as exc:
            res.error_code, res.message = 'INVALID_PAYLOAD', str(exc)
        return res


def main():
    rclpy.init()
    node = ImageTextNode()
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
