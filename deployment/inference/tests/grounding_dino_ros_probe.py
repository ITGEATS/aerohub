"""Independent ROS service client for full saved real image; no flight imports."""
import argparse
from array import array
import json
from pathlib import Path
import time
import numpy as np
from PIL import Image as PILImage
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from uav_inference_interfaces.srv import InferGroundingDino


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--image', type=Path, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reference = json.loads(args.baseline.read_text())['direct_boxes']
    rgb = np.asarray(PILImage.open(args.image).convert('RGB'))
    rclpy.init()
    node = Node('grounding_dino_independent_probe')
    client = node.create_client(InferGroundingDino, '/probe/inference/grounding_dino/infer')
    try:
        assert client.wait_for_service(timeout_sec=30), 'service unavailable'
        req = InferGroundingDino.Request(request_id='pickup-dds', generation=19,
            prompt='car. truck. pickup truck.', box_threshold=.3, text_threshold=.25)
        req.image = Image(width=rgb.shape[1], height=rgb.shape[0], encoding='rgb8',
            step=rgb.shape[1]*3, data=array('B', rgb.tobytes()))
        req.image.header.frame_id = 'saved_camera_optical'
        req.image.header.stamp.sec = 123
        req.image.header.stamp.nanosec = 456789
        records = []
        def call(request):
            started = time.perf_counter()
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=30)
            assert future.done(), 'DDS timeout'
            return future.result(), (time.perf_counter()-started)*1000
        for index in range(3):
            req.request_id = 'pickup-dds-'+str(index)
            result, latency = call(req)
            assert result.success, result.error
            frame = result.result
            assert frame.header == req.image.header and frame.request_id == req.request_id
            assert frame.generation == req.generation
            assert (frame.width, frame.height) == (rgb.shape[1], rgb.shape[0])
            assert len(frame.detections) == len(reference)
            assert result.prompt == req.prompt and result.normalized_prompt == req.prompt
            assert result.box_threshold == req.box_threshold and result.text_threshold == req.text_threshold
            boxes = []
            for detection, expected in zip(frame.detections, reference):
                assert detection.category == expected['category']
                assert detection.mask.header == req.image.header
                assert detection.mask.width == detection.mask.height == len(detection.mask.data) == 0
                np.testing.assert_allclose(detection.xyxy, expected['xyxy'], atol=.02, rtol=1e-5)
                np.testing.assert_allclose(detection.confidence, expected['confidence'], atol=1e-5, rtol=1e-5)
                boxes.append(dict(xyxy=list(detection.xyxy), confidence=detection.confidence, category=detection.category))
            records.append(dict(request_id=frame.request_id, roundtrip_ms=latency,
                worker_generation=frame.worker_generation, model_version=frame.model_version,
                boxes=boxes, timings=json.loads(frame.timings_json)))
        failures = []
        for field, value in [('prompt',' '), ('box_threshold',0.), ('text_threshold',float('nan'))]:
            original = getattr(req, field)
            setattr(req, field, value)
            rejected, _ = call(req)
            assert not rejected.success and rejected.error
            failures.append(dict(field=field, error=rejected.error))
            setattr(req, field, original)
        req.image.step = 1
        rejected, _ = call(req)
        assert not rejected.success
        failures.append(dict(field='image.step',error=rejected.error))
        req.image.step = rgb.shape[1]*3
        recovered, _ = call(req)
        assert recovered.success, recovered.error
        args.output.write_text(json.dumps(dict(passed=True, calls=records, invalid=failures,
            recovery=True, scope='saved real image Engine DDS/CUDA only'), indent=2)+'\n')
        print(json.dumps(dict(passed=True, roundtrip_ms=[r['roundtrip_ms'] for r in records], boxes=records[0]['boxes'])))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
