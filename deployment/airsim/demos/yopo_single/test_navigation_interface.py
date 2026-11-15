"""Real Action and shared AS2 Module against YOPO + deterministic ROS doubles.

Exercises orchestration only; inference/flight claims require separate SITL runs.
"""
import copy
import json
import threading
import time

import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase
from geometry_msgs.msg import PoseStamped
from rclpy.executors import MultiThreadedExecutor
from std_srvs.srv import Trigger
from uav_planner_interfaces.action import NavigateToPose
from uav_yopo_planner.navigation import YopoNavigation
from uav_yopo_planner.node import YopoPlanner
from test_ros_planner import Fixture


def main():
    rclpy.init(args=['--ros-args', '-r', '__ns:=/drone0'])
    fixture, planner = Fixture(), YopoPlanner()
    planner.depth_model = 'axial'
    planner.optical_frame = 'drone0/yopo_front_optical'
    planner.navigation = YopoNavigation(planner)
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(fixture); executor.add_node(planner)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    drone = DroneInterfaceBase('drone0')
    drone.load_module('navigation')
    nav = drone.navigation

    def wait(predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if predicate():
                return
            time.sleep(.01)
        raise AssertionError((planner.state, planner.reason, planner.timeline.revision))

    def task(x=20., **kwargs):
        wait(lambda: planner.observations_ready(planner.now_s()) and planner.ready())
        assert nav.to_point(x, 0., 0., wait=False, **kwargs)

    def outcome(code):
        wait(lambda: nav.done)
        assert nav.result.code == code, str(nav.result)
        assert nav.result.stop_confirmed
        wait(lambda: not planner.navigation.reserved)

    checks = []
    try:
        wait(lambda: nav.ready and planner.observations_ready(planner.now_s()) and planner.ready())
        original_ready = planner.ready
        planner.ready = lambda: False
        assert nav.to_point(20., 0., 0., wait=False)
        outcome('NOT_READY')
        planner.ready = original_ready
        checks.append('readiness rejection')
        task(frame_id='map'); outcome('INVALID_GOAL')
        task(float('nan')); outcome('INVALID_GOAL')
        task(position_tolerance=1.5); outcome('UNSUPPORTED')
        task(max_speed=1.); outcome('UNSUPPORTED')
        pose = PoseStamped(); pose.header.frame_id = planner.frame; pose.pose.orientation.w = 1.
        assert nav.to_pose(pose, wait=False)
        outcome('UNSUPPORTED')
        checks.append('explicit constraint rejection')
        task(3.); outcome('SUCCEEDED')
        task(3., position_tolerance=5.); outcome('SUCCEEDED')
        checks.append('repeated arrival and confirmed stop')
        task()
        wait(lambda: planner.timeline.revision >= 3)
        # Old goal transport cannot replace an Action-owned task.
        task_id = planner.task
        replacement = copy.deepcopy(pose); replacement.pose.position.x = 3.
        fixture.goal_pub.publish(replacement)
        time.sleep(.15)
        assert planner.task == task_id
        response = planner.cancel(Trigger.Request(), Trigger.Response())
        assert not response.success
        try:
            drone2 = DroneInterfaceBase('drone0')
            drone2.load_module('navigation')
            wait(lambda: drone2.navigation.ready)
            try:
                drone2.navigation.to_point(3., 0., 0., wait=False)
            except RuntimeError as error:
                assert 'rejected' in str(error)
            else:
                raise AssertionError('busy task accepted')
        finally:
            drone2.shutdown(); drone2.destroy_node()
        nav.cancel_async(); outcome('CANCELED')
        checks.append('busy rejection, legacy guard and cancellation')
        task(timeout=.2); outcome('TIMEOUT')
        checks.append('task timeout and stop')
        fixture.reject = True
        task(); outcome('EXECUTION_FAILED')
        fixture.reject = False
        checks.append('execution failure propagation')
        # No false success when the original stop gate fails.
        original_stop = planner.stop_tick
        planner.stop_tick = lambda now: setattr(planner, 'state', 'FAULT')
        task(3.)
        wait(lambda: nav.done)
        assert nav.result.code == 'STOP_UNCONFIRMED' and not nav.result.stop_confirmed, str(nav.result)
        planner.stop_tick = original_stop
        checks.append('unconfirmed stop remains failure')
        print(json.dumps(dict(result='passed', checks=checks)))
    finally:
        fixture.stopped.set()
        drone.shutdown(); drone.destroy_node()
        executor.shutdown(timeout_sec=3); thread.join(timeout=3)
        planner.destroy_node(); fixture.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
