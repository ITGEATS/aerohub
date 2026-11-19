"""Actual FastAPI CUDA inference and bounded invalid-request recovery."""
import argparse
import base64
import copy
import json
import io
from pathlib import Path
import sys
import time
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

import numpy as np
from PIL import Image


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--image', type=Path, required=True)
    p.add_argument('--baseline', type=Path, required=True)
    p.add_argument('--output', type=Path, required=True)
    p.add_argument('--endpoint', default='http://127.0.0.1:18083')
    args = p.parse_args()
    opener = build_opener(ProxyHandler({}))
    def call(route, body=None, timeout=30):
        data = None if body is None else json.dumps(body).encode()
        request = Request(args.endpoint+route, data=data, headers={'Content-Type':'application/json'})
        started = time.perf_counter()
        try:
            with opener.open(request, timeout=timeout) as response:
                return response.status, json.load(response), (time.perf_counter()-started)*1000
        except HTTPError as error:
            return error.code, error.read().decode(), (time.perf_counter()-started)*1000
    assert call('/health/live')[0] == 200
    status, metadata, _ = call('/health/ready')
    assert status == 200 and metadata['device'] == 'cuda:0'
    image = Image.open(args.image).convert('RGB')
    png = io.BytesIO()
    image.save(png, format='PNG')
    body = dict(request_id='pickup-http', stamp_ns=1234567890, generation=11,
        frame='saved_camera_optical', width=image.width, height=image.height,
        encoding='png_rgb8', image_base64=base64.b64encode(png.getvalue()).decode(),
        prompt='car. truck. pickup truck.', box_threshold=.30, text_threshold=.25)
    baseline = json.loads(args.baseline.read_text())['direct_boxes']
    status, result, latency = call('/infer', body)
    assert status == 200, result
    assert len(result['boxes']) == len(baseline)
    for box, expected in zip(result['boxes'], baseline):
        assert box['category'] == expected['category']
        assert set(box) == {'xyxy','confidence','category'}
        np.testing.assert_allclose(box['xyxy'], expected['xyxy'], atol=.02, rtol=1e-5)
        np.testing.assert_allclose(box['confidence'], expected['confidence'], atol=1e-5, rtol=1e-5)
    invalid = []
    cases = [('prompt',' '), ('prompt','word '*205), ('prompt','a '*255), ('box_threshold',0.),
        ('text_threshold',1.), ('box_threshold',float('nan')), ('text_threshold',float('inf')), ('width',2049), ('width',image.width-1), ('encoding','jpeg'),
        ('image_base64','bad!'), ('image_base64',base64.b64encode(b'bad png').decode())]
    for key, value in cases:
        bad = copy.copy(body)
        bad[key] = value
        code, message, _ = call('/infer',bad)
        assert code == 422, (key,code,message)
        assert not call('/health/ready')[1]['busy']
        invalid.append(dict(field=key, status=code, error=message))
    # A real timed-out CUDA call must drain, without immediate retrying the image.
    timed_out = False
    try:
        call('/infer',body,timeout=.001)
    except (OSError, TimeoutError):
        timed_out = True
    deadline = time.monotonic()+30
    busy_observed = False
    busy_rejections = []
    while time.monotonic()<deadline:
        ready = call('/health/ready')[1]
        busy_observed |= ready['busy']
        if not ready['busy']:
            break
        code, _, _ = call('/infer',body)
        busy_rejections.append(code)
        assert code == 429, code
        time.sleep(.01)
    assert not ready['busy']
    assert call('/infer',body)[0] == 200
    args.output.write_text(json.dumps(dict(passed=True, metadata=metadata, http_roundtrip_ms=latency,
        result=result, invalid=invalid, timeout=dict(timed_out=timed_out,busy_observed=busy_observed,
        busy_rejections=busy_rejections,drained=True,recovery=True)),indent=2)+'\n')
    print(json.dumps(dict(passed=True,http_roundtrip_ms=latency, boxes=result['boxes'],invalid=len(invalid))))


if __name__ == '__main__':
    main()
