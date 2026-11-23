# SPDX-License-Identifier: GPL-3.0-or-later
"""Optional 2D association state owned by one Target Tracking action session."""
import time
from pathlib import Path
from types import SimpleNamespace

import numpy as np

CLASSES = {'person': 0, 'car': 1, 'truck': 2, 'bus': 3}


class StreamTracker:
    """Keep ByteTrack history local to a session; never modify detector messages."""

    def __init__(self):
        # Keep optional CPU association dependencies out of snapshot sessions.
        import yaml
        from ultralytics import __file__ as ultralytics_file
        from ultralytics.engine.results import Boxes
        from ultralytics.trackers.byte_tracker import BYTETracker

        config_path = Path(ultralytics_file).parent / 'cfg/trackers/bytetrack.yaml'
        self._config = SimpleNamespace(**yaml.safe_load(config_path.read_text(encoding='utf-8')))
        self._tracker_type = BYTETracker
        self._boxes_type = Boxes
        self.key = None
        self.tracker = None
        self.generation = 0
        self.last_stamp = 0

    def attach(self, frame):
        key = (frame.tool_generation, frame.worker_generation, frame.model_version,
               frame.image_header.frame_id)
        if not all(key):
            raise ValueError('ByteTrack requires a complete detection source identity')
        if key != self.key:
            self.tracker = self._tracker_type(args=self._config)
            self.key = key
            self.generation = max(time.time_ns(), self.generation + 1)
            self.last_stamp = 0
        stamp_ns = frame.image_header.stamp.sec * 10**9 + frame.image_header.stamp.nanosec
        if stamp_ns <= self.last_stamp:
            raise ValueError('duplicate or backward ByteTrack frame within one source generation')

        detections = frame.detections
        rows = [[*item.detection.xyxy, item.detection.confidence,
                 CLASSES[item.detection.category]] for item in detections]
        # ByteTrack needs pixel boxes only; it does not consume the image or masks.
        boxes = self._boxes_type(np.asarray(rows, dtype=np.float32).reshape(-1, 6), (0, 0))
        tracks = self.tracker.update(boxes)
        self.last_stamp = stamp_ns
        ids = [0] * len(detections)
        for row in tracks:
            track_id, index = int(row[4]), int(row[-1])
            if not 0 <= index < len(detections) or track_id <= 0:
                raise ValueError('invalid ByteTrack result index or ID')
            ids[index] = track_id
        return tuple(ids), self.generation

    def close(self):
        """Release history when the owning action ends, including cancel/failure."""
        self.tracker = None
        self.key = None
        self.last_stamp = 0
