"""Saved-image CUDA regression for optional ByteTrack on the actual YOLO worker."""
import argparse
import base64
import json

import cv2
import numpy as np
from ultralytics import __version__ as ultralytics_version

from adapter import Predictor


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('image')
    args = parser.parse_args()
    image = cv2.imread(args.image)
    if image is None:
        raise ValueError('Could not load a real image with detections')

    height, width = image.shape[:2]

    def payload(frame):
        ok, png = cv2.imencode('.png', frame)
        if not ok:
            raise ValueError('Could not encode test frame')
        return base64.b64encode(png).decode('ascii')

    body = dict(request_id='cuda-bytetrack', stamp_ns=100, generation=17,
                frame='optical', width=width, height=height, encoding='png_bgr8',
                image_base64=payload(image))
    detector = Predictor()
    try:
        assert detector.metadata['device'] == 'cuda:0'
        assert ultralytics_version == '8.4.210'
        baseline = detector.infer(body)
        stream = dict(body, tracking_stream='cuda-test')
        first = detector.infer(stream)
        empty = detector.infer(dict(stream, stamp_ns=101, image_base64=payload(np.zeros_like(image))))
        second = detector.infer(dict(stream, stamp_ns=102))
        assert not empty['boxes']
        ids = [box.get('track_id', 0) for box in first['boxes']]
        assert any(ids), 'ByteTrack assigned no ID to real detections'
        assert ids == [box.get('track_id', 0) for box in second['boxes']]
        assert first['tracking_generation'] == empty['tracking_generation'] == second['tracking_generation'] > 0
        for frame in (first, second):
            boxes = [{key: value for key, value in box.items() if key != 'track_id'}
                     for box in frame['boxes']]
            assert boxes == baseline['boxes'], 'ByteTrack changed the original boxes or masks'
        snapshot = detector.infer(body)
        assert snapshot['tracking_generation'] == 0 and snapshot['boxes'] == baseline['boxes']
        reset = detector.infer(dict(stream, generation=18))
        assert reset['tracking_generation'] > first['tracking_generation']
        print(json.dumps(dict(device=detector.metadata['device'], model='yolo26n-seg',
                              decoded_detections=len(ids), persistent_ids=ids,
                              bytetrack_ms=second['timings_ms']['bytetrack'],
                              new_scope_after_restart=True, boxes_and_masks_unchanged=True)))
    finally:
        detector.close()


if __name__ == '__main__':
    main()
