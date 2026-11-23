#!/usr/bin/env python3
"""Onboard-only real WestStreet RGB-D -> YOLO -> DetectSnapshot; no UAV motion."""
import argparse
from collections import deque
import json
from pathlib import Path
from weststreet_ports import PORTS
import threading
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from as2_msgs.msg import PlatformInfo
from sensor_msgs.msg import Image
from uav_inference.yolo_node import YoloNode
from uav_object_detection.node import ObjectDetectionTool
from uav_object_detection.projection import depth_array, stamp_ns
from uav_object_detection_interfaces.srv import DetectSnapshot
from simulator.projectairsim.extension_client import ProjectAirSimExtensionClient


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--range-method',choices=['depth','height'],default='depth')
    parser.add_argument('--target-height',type=float)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    if args.range_method=='height' and (args.target_height is None or args.target_height<=0):
        parser.error('height mode requires explicit positive --target-height')
    ros_args=['--ros-args', '-r', '__ns:=/drone0', '-p', 'use_sim_time:=true',
                     '-p', f"endpoint:=http://127.0.0.1:{PORTS['inference']}", '-p', 'timeout_s:=2.0',
                     '-p', 'detector_backend:=yolo', '-p', 'query:=person',
                     '-p', 'registered_depth:=true', '-p', 'depth_is_perspective:=false',
                     '-p', 'output_frame:=earth', '-p', 'camera_reliable:=false']
    if args.range_method=='height':
        ros_args+=['-p','position_method:=known_height_vertical_v1','-p',f'target_height_m:={args.target_height}']
    rclpy.init(args=ros_args)
    peer = Node('weststreet_person_probe')
    inference, detector = YoloNode(), ObjectDetectionTool()
    detector._ensure_position_inputs()
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (peer, inference, detector):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    platform = deque(maxlen=1)
    peer.create_subscription(PlatformInfo, 'platform/info', platform.append, qos_profile_sensor_data)
    client = peer.create_client(DetectSnapshot, 'tools/object_detection/observe')
    actor = ProjectAirSimExtensionClient(port=PORTS['hud'], scene_port=PORTS['scene'])
    result = dict(success=False, scope='real disarmed observation; no pursuit or absolute-position acceptance',
                  confidence_threshold=.35, range_method=args.range_method, assumed_height_m=args.target_height, observations=[], audits=[])
    captures = {}
    original_infer = detector.infer

    def audited_infer(task, **kwargs):
        # Snapshot's actual RGB input, not a separately requested screenshot.
        image = task.image
        pixels = np.frombuffer(bytes(image.data), np.uint8).reshape(image.height, image.step)
        pixels = pixels[:, :image.width * 3].reshape(image.height, image.width, 3).copy()
        captures[stamp_ns(image.header)] = pixels
        return original_infer(task, **kwargs)

    detector.infer = audited_infer
    original_project = detector.projector.project

    def audited_project(frame, token, active, position_method):
        # Freeze the same depth choice used by Projector before it can leave the queue.
        with detector.projector.lock:
            candidates = list(detector.projector.depth)
        selected = min(candidates, key=lambda p: abs(stamp_ns(p[0].header)-stamp_ns(frame.header))) if candidates else None
        measured = original_project(frame, token, active, position_method)
        audit = dict(rgb_stamp_ns=stamp_ns(frame.header), depth_stamp_ns=stamp_ns(measured.depth_header))
        key = str(audit['rgb_stamp_ns'])
        projection_header = frame.header if args.range_method=='height' else measured.depth_header
        audit['tf_stamp_ns'] = stamp_ns(projection_header)
        if selected is not None and stamp_ns(selected[0].header) == audit['depth_stamp_ns']:
            np.save(args.output / (key + '-depth.npy'), selected[1])
            audit['depth_file'] = key + '-depth.npy'
        try:
            with detector.projector.lock:
                rgb_info = detector.projector._info_at('rgb', audit['rgb_stamp_ns'])
                audit['rgb_k'] = list(rgb_info.k)
                if args.range_method=='depth':
                    depth_info = detector.projector._info_at('depth', audit['depth_stamp_ns'])
                    audit['depth_k'] = list(depth_info.k)
            tf = detector.tf_buffer.lookup_transform('earth', frame.header.frame_id,
                                                      Time.from_msg(projection_header.stamp))
            t, q = tf.transform.translation, tf.transform.rotation
            audit.update(optical_frame=frame.header.frame_id,
                         camera_position=[t.x, t.y, t.z], camera_quaternion=[q.x, q.y, q.z, q.w])
        except Exception as exc:
            audit['audit_error'] = repr(exc)
        result['audits'].append(audit)
        return measured

    detector.projector.project = audited_project

    def wait(predicate, timeout=40):
        deadline = time.monotonic() + timeout
        while not predicate():
            if platform and platform[-1].armed:
                raise RuntimeError('Disarmed observation only')
            if time.monotonic() > deadline:
                raise TimeoutError('Camera/TF/inference readiness timeout')
            time.sleep(.01)

    def observe(phase, count):
        for _ in range(count):
            request = DetectSnapshot.Request(query='person', estimate_position=True, timeout=4.)
            start = time.monotonic()
            future = client.call_async(request)
            wait(future.done, 6)
            response = future.result()
            observation = response.result
            stamp = stamp_ns(observation.image_header)
            entry = dict(phase=phase, success=response.success, code=response.code, reason=response.reason,
                         rgb_stamp_ns=stamp, depth_stamp_ns=stamp_ns(observation.depth_header),
                         age_s=peer.get_clock().now().nanoseconds/1e9-stamp/1e9,
                         latency_s=time.monotonic()-start, request_id=observation.request_id,
                         detections=[dict(category=d.detection.category, confidence=d.detection.confidence,
                                          box=list(d.detection.xyxy), position_valid=d.position_valid,
                                          position_error=d.position_error, position_method=d.position_method,
                                          position=[d.position.x,d.position.y,d.position.z], depth_m=d.depth_m,
                                          valid_pixels=d.valid_depth_pixels, spread_m=d.depth_spread_m)
                                     for d in observation.detections])
            pixels = captures.pop(stamp, None)
            if pixels is not None:
                raw = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
                cv2.imwrite(str(args.output / (str(stamp)+'-rgb.png')), raw)
                for d in observation.detections:
                    box = [int(v) for v in d.detection.xyxy]
                    cv2.rectangle(raw, tuple(box[:2]), tuple(box[2:]), (0,255,0), 2)
                    cv2.putText(raw, f'{d.detection.category} {d.detection.confidence:.2f}',
                                (box[0],max(16,box[1]-4)), cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,0),1)
                    if d.detection.mask.width:
                        mask = np.frombuffer(bytes(d.detection.mask.data),np.uint8).reshape(pixels.shape[:2])
                        np.save(args.output / (str(stamp)+'-mask.npy'),mask)
                cv2.imwrite(str(args.output / (str(stamp)+'-overlay.png')),raw)
            result['observations'].append(entry)
            (args.output/'progress.json').write_text(json.dumps(result,indent=2))
            print(json.dumps(entry),flush=True)

    try:
        result['actor_initial'] = actor.reset_actor()
        wait(lambda: platform and not platform[-1].armed and client.service_is_ready() and
             (detector.projector.depth if args.range_method=='depth' else detector.projector.info['rgb']))
        result['worker'] = inference.detector.client.request('/health/ready')
        assert result['worker']['device'].startswith('cuda')
        # CameraInfo may arrive before AS2 dynamic TF; check actual capture-time availability.
        def ready():
            if args.range_method=='depth':
                msg = detector.projector.depth[-1][0]
                return detector.tf_buffer.can_transform('earth', msg.header.frame_id, Time.from_msg(msg.header.stamp))
            info=detector.projector.info['rgb'][-1]
            stamp=detector.snapshot.last_stamp_ns
            return stamp>100_000_000 and detector.tf_buffer.can_transform('earth',info.header.frame_id,Time(nanoseconds=stamp-100_000_000))
        wait(ready)
        observe('paused', 6)
        static = result['observations']
        result['recognition_passed'] = len({e['rgb_stamp_ns'] for e in static if e['success'] and e['detections']}) >= 4
        result['static_range_passed'] = len({e['rgb_stamp_ns'] for e in static if any(d['position_valid'] for d in e['detections'])}) >= 4
        if result['recognition_passed']:
            result['actor_started'] = actor.start_actor()
            observe('running', 6)
        moving = [e for e in result['observations'] if e['phase']=='running']
        result['running_recognition_passed'] = len({e['rgb_stamp_ns'] for e in moving if e['success'] and e['detections']}) >= 4
        result['running_range_passed'] = len({e['rgb_stamp_ns'] for e in moving if any(d['position_valid'] for d in e['detections'])}) >= 4
        result['success'] = result['recognition_passed'] and result['static_range_passed'] and result['running_recognition_passed'] and result['running_range_passed']
    except Exception as exc:
        result['error'] = repr(exc)
    finally:
        try:
            result['actor_final'] = actor.stop_actor()
        except Exception as exc:
            result['actor_stop_error'] = repr(exc)
        result['disarmed'] = bool(platform and not platform[-1].armed)
        detector.stop_event.set()
        executor.shutdown(timeout_sec=5)
        thread.join(5)
        for node in (detector,inference,peer):
            node.destroy_node()
        rclpy.try_shutdown()
        result['cleanup_complete'] = not thread.is_alive()
        (args.output/'result.json').write_text(json.dumps(result,indent=2))
    if not result['success']:
        raise SystemExit(1)


if __name__ == '__main__':
    main()
