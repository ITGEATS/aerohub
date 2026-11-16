"""Real static AirSim/GPU -> DetectSnapshot -> TrackObject ground consumer.

Predeclared: at least four tracked feedbacks, real frame/stamps/covariance,
standard cancel, disarmed before/after. This does not test moving target accuracy.
"""
import argparse
import json
from pathlib import Path
import threading
import time
import numpy as np
import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from as2_msgs.msg import PlatformInfo
from uav_inference.grounding_dino_node import GroundingDinoNode
from uav_object_detection.node import ObjectDetectionTool
from uav_target_tracking.node import TrackingNode
from uav_target_tracking_interfaces.action import TrackObject


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--estimate-position', action='store_true')
    parser.add_argument('--near-target', action='store_true')
    parser.add_argument('--samples', type=int, default=10)
    args = parser.parse_args()
    args.output.mkdir(exist_ok=True)
    rclpy.init(args=['--ros-args', '-r', '__ns:=/drone0', '-p', 'use_sim_time:=true',
                     '--params-file', '/repo/deployment/target_tracking/params.yaml',
                     '-p', 'camera_reliable:=true'])
    peer = Node('tracking_ground_consumer')
    inference, detector, tracker = GroundingDinoNode(), ObjectDetectionTool(), TrackingNode()
    executor = MultiThreadedExecutor(num_threads=8)
    for node in (peer, inference, detector, tracker): executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True); thread.start()
    platform, feedback = [], []
    peer.create_subscription(PlatformInfo, 'platform/info', platform.append, qos_profile_sensor_data)
    drone = DroneInterfaceBase('drone0', use_sim_time=True)
    drone.load_module('target_tracking')
    result = dict(passed=False, scope='static ground real AirSim/GPU; no moving target, navigation or flight')
    handle = None
    outcome = None
    def wait(predicate, timeout=40):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline: raise TimeoutError('ground readiness/feedback timeout')
            time.sleep(.01)
    def on_feedback(message):
        s = message.feedback.state
        feedback.append(dict(status=s.status, valid=s.valid, session=s.session_id,
            capture=s.capture_id, generation=s.generation, frame=s.header.frame_id,
            stamp=s.header.stamp.sec+s.header.stamp.nanosec*1e-9,
            observation=s.observation_stamp.sec+s.observation_stamp.nanosec*1e-9,
            valid_until=s.valid_until.sec+s.valid_until.nanosec*1e-9,
            p=[s.position.x,s.position.y,s.position.z], v=[s.velocity.x,s.velocity.y,s.velocity.z],
            covariance=list(s.covariance), reason=s.reason))
    try:
        detector._ensure_position_inputs()
        def ready():
            if not platform or not detector.projector.depth: return False
            header = detector.projector.depth[-1][0].header
            return detector.tf_buffer.can_transform('drone0/odom', header.frame_id, Time.from_msg(header.stamp))
        wait(ready)
        assert not platform[-1].armed
        result['initial_disarmed'] = True
        result['health'] = inference.detector.client.request('/health/ready')
        assert result['health']['model_id']=='grounding-dino-tiny' and result['health']['device'].startswith('cuda')
        wait(lambda: drone.target_tracking.ready, 5)
        future = drone.target_tracking.track('car', timeout=12., feedback_callback=on_feedback)
        wait(future.done, 5); handle = future.result(); assert handle.accepted
        outcome = handle.get_result_async()
        wait(lambda: len({s['capture'] for s in feedback if s['valid'] and s['status']=='tracked'})>=4 or outcome.done(), 13)
        if outcome.done():
            terminal = outcome.result()
            result['early_terminal'] = dict(status=terminal.status, code=terminal.result.code, reason=terminal.result.reason)
        tracked = [s for s in feedback if s['valid'] and s['status']=='tracked']
        assert len(tracked)>=4, 'Insufficient confirmed observations'
        assert len({s['capture'] for s in tracked}) >= 4
        assert len({s['observation'] for s in tracked}) >= 4
        for s in tracked:
            assert s['frame']=='drone0/odom' and s['observation']>0 and s['observation']<=s['stamp']<=s['valid_until']
            assert np.isfinite(s['p']+s['v']+s['covariance']).all()
        canceled = handle.cancel_goal_async(); wait(canceled.done, 3)
        assert canceled.result().goals_canceling
        wait(outcome.done, 3)
        assert outcome.result().result.code == 'CANCELED'
        assert not platform[-1].armed
        result.update(passed=True, final_disarmed=True, tracked_count=len({s['capture'] for s in tracked}), tracked_feedback_count=len(tracked))
    except Exception as exc:
        result['error'] = repr(exc)
        raise
    finally:
        if handle is not None and (outcome is None or not outcome.done()):
            handle.cancel_goal_async()
        tracker._stopping.set()
        executor.shutdown(timeout_sec=5); thread.join(5)
        drone.target_tracking.destroy()
        drone.target_tracking.destroy()  # explicit idempotence check before DroneInterface shutdown
        drone.shutdown()
        for node in (tracker, detector, inference, peer): node.destroy_node()
        rclpy.try_shutdown()
        result['cleanup_complete'] = not thread.is_alive()
        result['passed'] = result['passed'] and result['cleanup_complete']
        result['feedback'] = feedback
        (args.output/'result.json').write_text(json.dumps(result, indent=2)+'\n')

if __name__ == '__main__': main()
