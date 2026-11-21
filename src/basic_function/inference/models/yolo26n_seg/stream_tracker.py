# SPDX-License-Identifier: GPL-3.0-or-later
"""Attach Ultralytics ByteTrack IDs to existing detections by source index."""
from pathlib import Path
from types import SimpleNamespace
import time

import numpy as np
import yaml
from ultralytics import __file__ as ultralytics_file
from ultralytics.engine.results import Boxes

CLASSES = {'person': 0, 'car': 1, 'truck': 2, 'bus': 3}


class StreamTracker:
    """One persistent stream per worker. A change of stream starts a new ID scope."""

    def __init__(self):
        self.key = None
        self.tracker = None
        self.generation = 0
        self.last_stamp = 0

    def attach(self, detections, image, *, stream, source_generation, frame, stamp_ns):
        key = stream, source_generation, frame, image.shape[:2]
        if key != self.key:
            # Import only for opted-in streams. lap is pinned in the worker environment.
            from ultralytics.trackers.byte_tracker import BYTETracker

            cfg_path = Path(ultralytics_file).parent / 'cfg/trackers/bytetrack.yaml'
            config = SimpleNamespace(**yaml.safe_load(cfg_path.read_text(encoding='utf-8')))
            self.tracker = BYTETracker(args=config)
            self.key = key
            self.generation = max(time.time_ns(), self.generation + 1)
            self.last_stamp = 0
        if stamp_ns <= self.last_stamp:
            raise ValueError('duplicate or backward ByteTrack frame within one source generation')

        # Preserve each box/mask pair. Filtered tracker boxes are never used for RGB-D projection.
        rows = [[*box['xyxy'], box['confidence'], CLASSES[box['category']]] for box in detections]
        boxes = Boxes(np.asarray(rows, dtype=np.float32).reshape(-1, 6), image.shape[:2])
        tracks = self.tracker.update(boxes, image)
        self.last_stamp = stamp_ns
        for row in tracks:
            track_id, index = int(row[4]), int(row[-1])
            if not 0 <= index < len(detections) or track_id <= 0:
                raise ValueError('invalid ByteTrack result index or ID')
            detections[index]['track_id'] = track_id
        return self.generation
