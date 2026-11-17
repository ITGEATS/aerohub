#!/usr/bin/env python3
"""Feed same-frame YOLO predictions to the official ProjectAirSim ImageDisplay."""
import argparse
import json
from pathlib import Path
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[4]))


def native_frame(folder, row):
    """Convert recorded inference RGB and xyxy predictions, never simulator truth."""
    stamp = str(row['stamp_ns'])
    meta = json.loads((folder/(stamp+'.json')).read_text())
    width, height = meta['width'], meta['height']
    if meta['encoding'] not in ('rgb8', 'bgr8'):
        raise ValueError('Unsupported encoding: '+meta['encoding'])
    raw = (folder/(stamp+'.rgb')).read_bytes()
    step = meta['step']
    if step < width*3 or len(raw)!=step*height:
        raise ValueError('Invalid RGB stride or size')
    packed = bytearray().join(raw[y*step:y*step+width*3] for y in range(height))
    # Official unpack_image leaves raw 3-channel bytes unchanged; HighGUI expects BGR.
    if meta['encoding']=='rgb8':
        packed[0::3], packed[2::3] = packed[2::3], packed[0::3]
    annotations = []
    for detection in row.get('detections', []):
        x1,y1,x2,y2 = detection['xyxy']
        if x2>x1 and y2>y1:
            annotations.append(dict(bbox2d=dict(center=dict(x=(x1+x2)/2,y=(y1+y2)/2),
                size=dict(x=x2-x1,y=y2-y1))))
    # Writable buffer: the official rectangle renderer draws in-place on its copy.
    return dict(width=width,height=height,encoding='BGR8',data=packed,
                time_stamp=row['stamp_ns'],annotations=annotations)


def labeled_frame(folder, row, mode):
    """Draw only on a display copy; official ImageDisplay still owns the window."""
    import cv2
    import numpy as np
    message = native_frame(folder, row)
    image = np.frombuffer(message['data'], dtype=np.uint8).reshape(
        message['height'], message['width'], 3).copy()
    color = (80, 255, 80)
    def text(label, x, y, ink=color):
        x = max(0, min(int(x), max(0, image.shape[1]-len(label)*8)))
        y = max(13, min(int(y), image.shape[0]-3))
        cv2.putText(image, label, (x,y), cv2.FONT_HERSHEY_SIMPLEX, .4,
                    (0,0,0), 3, cv2.LINE_AA)
        cv2.putText(image, label, (x,y), cv2.FONT_HERSHEY_SIMPLEX, .4,
                    ink, 1, cv2.LINE_AA)
    for detection in row.get('detections', []):
        x1,y1,x2,y2 = detection['xyxy']
        if x2<=x1 or y2<=y1:
            continue
        cv2.rectangle(image, (int(x1),int(y1)), (int(x2),int(y2)), color, 1)
        text('{} {:.2f} h={:.0f}px'.format(detection.get('category','?'),
            detection.get('confidence',0),y2-y1),x1,y1-5)
    text('{} sample={:.3f}s inference={:.0f}ms'.format(mode,
        row['stamp_ns']/1e9,row.get('elapsed_s',0)*1000),5,15,(255,255,255))
    if row.get('error') or not row.get('detections'):
        text(row.get('error') or 'NO DETECTIONS',5,32,(0,190,255))
    message['data'] = bytearray(image.tobytes())
    message['annotations'] = []  # Avoid the official renderer drawing thick boxes again.
    return message


def main():
    from functools import partial
    from simulator.projectairsim.image_windows import ImageWindow, ImageWindows, _ProcessDisplay
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('run_dir', type=Path)
    parser.add_argument('--replay', action='store_true')
    parser.add_argument('--start', type=float, default=0, help='Replay offset from first inference, seconds')
    args = parser.parse_args()
    folder = args.run_dir/'detections'
    name = 'YOLO '+('REPLAY' if args.replay else 'LIVE')+' - same frame'
    with (folder/'calls.jsonl').open() as source, ImageWindows(
            window_size=(640,480), row_position=.7,
            display_factory=partial(_ProcessDisplay,with_bounding_box=False)) as windows:
        windows.show([ImageWindow(name,'yolo')])
        position = 0
        pending = []
        first = None
        started = time.monotonic()
        while windows._display.running:
            while True:
                source.seek(position)
                line = source.readline()
                if not line or not line.endswith('\n'):
                    break
                position = source.tell()
                row = json.loads(line)
                if first is None:
                    first = row['stamp_ns']; started = time.monotonic()
                if row['stamp_ns']>=first+args.start*1e9:
                    pending.append(row)
            selected = None
            if args.replay:
                due = (time.monotonic()-started+args.start)*1e9+(first or 0)
                while pending and pending[0]['stamp_ns']<=due:
                    selected = pending.pop(0)
            elif pending:
                selected = pending[-1]; pending.clear()
            if selected is not None:
                windows.receive('yolo',labeled_frame(folder,selected,'REPLAY' if args.replay else 'LIVE'))
            time.sleep(.02)


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass
