"""Stage-4 PX4 SITL cross-machine flight chain (board client).

Full loop on ROS_DOMAIN_ID 83: PC clock container -> /clock; PX4 SITL (in the
PC flight_board container, bridged by its uXRCE agent) -> board AS2 platform;
AirSim Blocks cameras -> board AS2 sensor topics. Sequence: preflight gate ->
arm -> offboard -> takeoff -> FLYING -> framework START -> aerovla planner
(real Blocks images) -> TimedTrajectorySegment -> OutputGateway ->
FollowTimedPath goal ACCEPTED by the extended behavior -> execution feedback
observed -> framework CANCEL -> stop confirmed -> land -> disarm.

The SITL arming here is simulated flight (PX4 in a container); the physical
board FC is never touched.
"""
import json
import os
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.msg import GoalStatusArray
from as2_msgs.action import Land, Takeoff
from as2_msgs.msg import ControllerInfo, PlatformInfo, PlatformStatus
from std_srvs.srv import SetBool
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommandAck, VehicleControlMode, VehicleOdometry, VehicleStatus
from rosgraph_msgs.msg import Clock
from std_msgs.msg import String
from uav_planner_interfaces.msg import BehaviorResult
from uav_planner_interfaces.srv import ControlPlanner

TASK_PATH = '/work/cfg/task.json'
DEPLOYMENT_PATH = '/work/cfg/deployment.yaml'
CMD_ARM = 400
CMD_OFFBOARD = 176
# Stage-5 case matrix (OOD thresholds locked in validation.md before runs).
# formal: full chain; cancel: early cancel after goal accepted; fault: NPU HTTP
# endpoint unreachable -> planner reports failure and recovery closes the loop.
CASE = os.environ.get('CHAIN_CASE', 'formal').strip().lower()
assert CASE in ('formal', 'cancel', 'fault'), 'unknown CHAIN_CASE: ' + CASE


class FlightChain(Node):
    def __init__(self):
        super().__init__('flight_chain_client')
        self.latest = {}
        self.acks = {}
        self.execution_states = []
        self.statuses = []
        self.goal_statuses = []
        self.motion_refs = 0
        self.planner_states = []
        self.create_subscription(Clock, '/clock', self._latest('/clock'), qos_profile_sensor_data)
        self.create_subscription(VehicleControlMode, '/drone0/fmu/out/vehicle_control_mode',
                                 self._latest('/drone0/fmu/out/vehicle_control_mode'),
                                 qos_profile_sensor_data)
        self.create_subscription(VehicleStatus, '/drone0/fmu/out/vehicle_status_v4',
                                 self._latest('/drone0/fmu/out/vehicle_status_v4'),
                                 qos_profile_sensor_data)
        self.create_subscription(VehicleOdometry, '/drone0/fmu/out/vehicle_odometry',
                                 self._latest('/drone0/fmu/out/vehicle_odometry'),
                                 qos_profile_sensor_data)
        self.create_subscription(VehicleCommandAck, '/drone0/fmu/out/vehicle_command_ack_v1',
                                 self.on_ack, qos_profile_sensor_data)
        self.create_subscription(PoseStamped, '/drone0/self_localization/pose',
                                 self._latest('/drone0/self_localization/pose'),
                                 qos_profile_sensor_data)
        self.create_subscription(PlatformInfo, '/drone0/platform/info',
                                 self._latest('/drone0/platform/info'), qos_profile_sensor_data)
        self.create_subscription(String, 'planner/execution_state',
                                 self.on_execution_state, 10)
        self.create_subscription(String, 'status', self.on_planner_status, 10)
        self.create_subscription(BehaviorResult, 'planner/status', self.on_status, 10)
        self.create_subscription(GoalStatusArray,
                                 '/drone0/follow_timed_path/_action/status',
                                 self.on_goal_status, 10)
        self.create_subscription(ControllerInfo, '/drone0/controller/info',
                                 self.on_motion_reference, qos_profile_sensor_data)
        self.control_client = self.create_client(ControlPlanner, 'planner/control')

    def _latest(self, topic):
        def callback(message):
            self.latest[topic] = message
        return callback

    def on_ack(self, message):
        if message.target_system == 1 and message.target_component == 1:
            self.acks[message.command] = message.result

    def on_execution_state(self, message):
        self.execution_states.append(message.data)

    def on_planner_status(self, message):
        self.planner_states.append(message.data)

    def on_status(self, message):
        self.statuses.append({
            'task_id': message.task_id,
            'overall_status': int(message.overall_status),
            'failure_reason': message.failure_reason,
        })

    def on_goal_status(self, message):
        if message.status_list:
            self.goal_statuses.append(int(message.status_list[-1].status))

    def on_motion_reference(self, message):
        self.motion_refs += 1

    def wait_for(self, predicate, timeout, what):
        deadline = time.monotonic() + timeout
        while not predicate():
            if time.monotonic() > deadline:
                raise TimeoutError('%s (waited %.0fs)' % (what, timeout))
            time.sleep(0.1)

    def as2_service(self, name):
        client = self.create_client(SetBool, '/drone0/' + name)
        assert client.wait_for_service(timeout_sec=20), 'service unavailable: ' + name
        return client

    def px4_setbool(self, name, command, feedback, what):
        client = self.as2_service(name)
        self.acks.pop(command, None)
        future = client.call_async(SetBool.Request(data=True))
        self.wait_for(future.done, 15, 'AS2 ' + name + ' response')
        assert future.result().success, 'AS2 rejected ' + name
        self.wait_for(lambda: command in self.acks, 15, 'PX4 ACK for ' + what)
        assert self.acks[command] == VehicleCommandAck.VEHICLE_CMD_RESULT_ACCEPTED, \
            'PX4 rejected ' + what
        self.wait_for(feedback, 15, what + ' state feedback')
        print(what.upper() + '_OK', flush=True)

    def as2_action(self, kind, name, goal, result_field):
        client = ActionClient(self, kind, '/drone0/' + name)
        assert client.wait_for_server(timeout_sec=20), 'action unavailable: ' + name
        future = client.send_goal_async(goal)
        self.wait_for(future.done, 20, name + ' goal response')
        handle = future.result()
        assert handle.accepted, 'AS2 rejected action ' + name
        result = handle.get_result_async()
        self.wait_for(result.done, 120, name + ' result')
        assert getattr(result.result().result, result_field), 'AS2 action failed: ' + name
        print(name.upper() + '_OK', flush=True)
        return handle


def recover(node):
    """Best-effort recovery on failure: cancel task, land, disarm."""
    print('RECOVERY: cancel + land + disarm', flush=True)
    try:
        request = ControlPlanner.Request()
        request.command = ControlPlanner.Request.CANCEL
        node.control_client.call_async(request)
        time.sleep(2)
    except Exception:
        pass
    try:
        node.as2_action(Land, 'LandBehavior', Land.Goal(land_speed=0.4), 'land_success')
    except Exception as error:
        print('RECOVERY land failed:', error, flush=True)
    try:
        client = node.as2_service('set_arming_state')
        future = client.call_async(SetBool.Request(data=False))
        node.wait_for(future.done, 10, 'disarm response')
    except Exception as error:
        print('RECOVERY disarm failed:', error, flush=True)


def main():
    rclpy.init()
    node = FlightChain()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    import threading
    spinner = threading.Thread(target=executor.spin, daemon=True)
    spinner.start()
    started = time.monotonic()
    try:
        # Cross-machine DDS first: sim time must arrive from the PC clock
        # container before any sim-time stamp comparisons can succeed.
        node.wait_for(lambda: node.latest.get('/clock') is not None
                      and node.latest['/clock'].clock.sec > 0,
                      30, '/clock from PC clock container')
        print('CLOCK_OK', flush=True)
        node.wait_for(lambda: '/drone0/fmu/out/vehicle_odometry' in node.latest,
                      45, 'PX4 odometry via uXRCE bridge')
        node.wait_for(lambda: '/drone0/self_localization/pose' in node.latest,
                      30, 'AS2 self localization')
        node.wait_for(lambda: node.latest.get('/drone0/fmu/out/vehicle_status_v4') is not None
                      and node.latest['/drone0/fmu/out/vehicle_status_v4'].pre_flight_checks_pass,
                      120, 'PX4 preflight checks')
        print('PREFLIGHT_OK', flush=True)
        node.px4_setbool('set_arming_state', CMD_ARM,
                         lambda: node.latest['/drone0/fmu/out/vehicle_control_mode'].flag_armed,
                         'arm')
        node.px4_setbool('set_offboard_mode', CMD_OFFBOARD,
                         lambda: node.latest['/drone0/fmu/out/vehicle_control_mode']
                         .flag_control_offboard_enabled,
                         'offboard')
        node.as2_action(Takeoff, 'TakeoffBehavior',
                        Takeoff.Goal(takeoff_height=1.5, takeoff_speed=0.5),
                        'takeoff_success')
        node.wait_for(lambda: node.latest['/drone0/platform/info'].status.state
                      == PlatformStatus.FLYING, 30, 'FLYING platform state')
        print('FLYING_OK', flush=True)

        request = ControlPlanner.Request()
        request.command = ControlPlanner.Request.START
        request.task_config_path = TASK_PATH
        request.deployment_config_path = DEPLOYMENT_PATH
        future = node.control_client.call_async(request)
        node.wait_for(future.done, 30, 'START response')
        response = future.result()
        assert response is not None and response.accepted, \
            'START rejected: ' + (response.message if response else 'no response')
        print('START_OK', flush=True)

        def prepared():
            for payload in node.execution_states:
                try:
                    state = json.loads(payload)
                except ValueError:
                    continue
                if state.get('backend') == 'follow_path' and state.get('prepared'):
                    return True
            return False
        node.wait_for(prepared, 30, 'execution clients prepared')
        print('EXECUTION_PREPARED', flush=True)

        if CASE == 'fault':
            # NPU HTTP endpoint unreachable: planner must report FAILED with a
            # non-empty reason within max_consecutive_failures, without a crash.
            def failed():
                return any(s['overall_status'] == 4 and s['failure_reason']
                           for s in node.statuses)
            node.wait_for(failed, 120, 'planner FAILED status with reason (NPU unreachable)')
            print('FAULT_DETECTED', node.statuses[-1], flush=True)
            # Framework must stay responsive: CANCEL service answers (accept or
            # reject in terminal state both count as responsive).
            request = ControlPlanner.Request()
            request.command = ControlPlanner.Request.CANCEL
            future = node.control_client.call_async(request)
            node.wait_for(future.done, 20, 'CANCEL response under fault')
            resp = future.result()
            print('FAULT_CANCEL_RESPONSIVE accepted=%s'
                  % (resp.accepted if resp else None), flush=True)
        else:
            # formal / cancel: require real inference + accepted timed goal.
            def planner_stepped():
                return any('"step": 1' in payload for payload in node.planner_states)
            node.wait_for(planner_stepped, 60, 'planner inference step 1 (real Blocks cameras)')
            print('PLANNER_RUNNING', flush=True)

            node.wait_for(lambda: node.goal_statuses, 60,
                          'FollowTimedPath goal accepted by the action server')
            print('GOAL_ACCEPTED status=%d' % node.goal_statuses[-1], flush=True)

            if CASE == 'formal':
                refs_before = node.motion_refs
                time.sleep(12)
                print('MOTION_REFS', node.motion_refs, 'delta',
                      node.motion_refs - refs_before, flush=True)
                print('GOAL_STATUS_LATEST', node.goal_statuses[-5:], flush=True)
                print('PLANNER_STATES', node.planner_states[-2:], flush=True)
            # cancel case: inject CANCEL immediately after goal accepted; formal
            # case injects it after the 12s observation window above.

        if CASE != 'fault':
            request = ControlPlanner.Request()
            request.command = ControlPlanner.Request.CANCEL
            future = node.control_client.call_async(request)
            node.wait_for(future.done, 20, 'CANCEL response')
            response = future.result()
            assert response is not None and response.accepted, 'CANCEL rejected'
            print('CANCEL_OK', flush=True)

            def stopped():
                for payload in reversed(node.execution_states):
                    try:
                        state = json.loads(payload)
                    except ValueError:
                        continue
                    if state.get('stop_requested') and state.get('stop_confirmed'):
                        return True
                return False
            node.wait_for(stopped, 30, 'FollowPath stop confirmation')
            print('STOP_CONFIRMED', flush=True)

        node.as2_action(Land, 'LandBehavior', Land.Goal(land_speed=0.4), 'land_success')
        node.wait_for(lambda: not node.latest['/drone0/fmu/out/vehicle_control_mode'].flag_armed,
                      30, 'disarm after landing')
        print('DISARMED_OK', flush=True)
        print('CASE_%s_PASS wall_s=%.1f' % (CASE.upper(), time.monotonic() - started),
              flush=True)
    except BaseException as error:
        print('FLIGHT_CHAIN_FAIL', type(error).__name__, error, flush=True)
        recover(node)
        raise
    finally:
        # Stop the executor and join its spin thread before tearing down the
        # context; otherwise the C++ executor thread is still joinable when the
        # interpreter exits and the process aborts (exit 134) after PASS.
        try:
            executor.shutdown()
            spinner.join(timeout=5)
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
