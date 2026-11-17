#!/usr/bin/env python3
"""Onboard Tool servers. Platform/algorithm dependencies remain onboard."""
import json
from pathlib import Path
import signal
import threading
import time

import rclpy
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String
from uav_inference.yolo_node import YoloNode
from uav_object_detection.node import ObjectDetectionTool
from uav_system_tool.node import SystemTool
from uav_target_tracking.node import TrackingNode


def main():
    import argparse
    p = argparse.ArgumentParser()
    p.add_argument('--output', type=Path, required=True)
    args = p.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=['--ros-args', '-r', '__ns:=/drone0', '-p', 'use_sim_time:=true',
        '-p', 'endpoint:=http://127.0.0.1:18002', '-p', 'timeout_s:=2.0',
        '-p', 'detector_backend:=yolo', '-p', 'query:=person', '-p', 'stream_enabled:=true',
        '-p', 'observation_mode:=stream', '-p', 'stream_query:=person', '-p', 'timing_enabled:=true',
        '-p', 'registered_depth:=true', '-p', 'depth_is_perspective:=false',
        '-p', 'output_frame:=drone0/odom', '-p', 'camera_reliable:=false',
        '-p', 'position_method:=known_height_vertical_v1', '-p', 'target_height_m:=1.7',
        '-p', 'measurement_stddev_m:=1.0', '-p', 'association_policy:=soft_innovation_v1', '-p', 'position_filter:=none'])
    system, inference, detector, tracking = SystemTool(), YoloNode(), ObjectDetectionTool(), TrackingNode()
    detector._ensure_position_inputs()
    from simulator.projectairsim.udp_diagnostics import udp_snapshot
    udp_publisher=detector.create_publisher(String,'/diagnostics/vision_udp',20)
    def publish_udp():
        row=udp_snapshot({'tool_runtime.py'});row['role']='onboard_tools'
        udp_publisher.publish(String(data=json.dumps(row)))
    udp_timer=detector.create_timer(1.,publish_udp)
    executor = MultiThreadedExecutor(num_threads=10)
    for node in (system, inference, detector, tracking):
        executor.add_node(node)
    # Save only actual request RGB, rather than screenshots unrelated to detection.
    original = detector.infer
    def audited(task, **kwargs):
        image = task.image
        key = str(image.header.stamp.sec*10**9+image.header.stamp.nanosec)
        path = args.output/(key+'.rgb')
        path.write_bytes(bytes(image.data))
        (args.output/(key+'.json')).write_text(json.dumps(dict(width=image.width,
            height=image.height, step=image.step, encoding=image.encoding,
            frame=image.header.frame_id, stamp_ns=int(key))))
        started = time.monotonic()
        try:
            frame = original(task, **kwargs)
            audit = dict(stamp_ns=int(key), elapsed_s=time.monotonic()-started,
                returned_ros_time=detector.now(), valid=frame.valid, error=frame.error,
                timings=frame.timings_json, detections=[dict(category=d.detection.category,
                confidence=d.detection.confidence, xyxy=list(d.detection.xyxy),
                position_valid=d.position_valid, position_error=d.position_error if hasattr(d,'position_error') else '',
                position=[d.position.x,d.position.y,d.position.z]) for d in frame.detections])
        except Exception as error:
            audit = dict(stamp_ns=int(key), elapsed_s=time.monotonic()-started, error=str(error))
            raise
        finally:
            with (args.output/'calls.jsonl').open('a') as stream:
                stream.write(json.dumps(audit)+'\n')
        return frame
    detector.infer = audited
    stopped = threading.Event()
    def stop(*_):
        stopped.set()
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        while not stopped.is_set():
            executor.spin_once(timeout_sec=.1)
    finally:
        executor.shutdown(timeout_sec=15.)
        for node in (tracking, detector, inference, system):
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
