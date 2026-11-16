"""Own AS2/navigation and a single EGO session, with live readiness and stop gates."""
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.action import ActionClient
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.msg import PlatformInfo, BehaviorStatus, ControllerInfo
from as2_msgs.action import Takeoff, Land
from px4_msgs.msg import VehicleControlMode
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String, Empty
from rosgraph_msgs.msg import Clock
from rclpy.time import Time
from tf2_ros import Buffer, TransformListener
from uav_planner_interfaces.action import FollowTimedPath
from policy import safe_to_stop, capability_ready
from interfaces import topic_map, probe_prefix

ROOT = Path(os.environ.get('EGO_SESSION_ROOT', Path(__file__).resolve().parent))


class Runtime(Node):
    def __init__(self, out):
        self.config = json.loads((ROOT / 'config.json').read_text())
        self.vehicle = self.config.get('vehicle') or 'drone0'
        super().__init__('ego_single_runtime', namespace=self.vehicle if self.config.get('hybrid') else '',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.out = out
        self.rows = {}
        self.children = []
        self.error = ''
        self.stop_requested = False
        self.last_history = 0.0
        self.config = json.loads((ROOT / 'config.json').read_text())
        self.topic_map = topic_map(self.config)
        self.tf = Buffer()
        self.listener = TransformListener(self.tf, self)
        self.subs = []
        if self.config.get('diagnose_cloud'):
            def diagnose_cloud(msg):
                stamp = Time.from_msg(msg.header.stamp)
                row = dict(wall=time.time(), mono=time.monotonic(),
                    stamp_ns=stamp.nanoseconds, ros_now_ns=self.get_clock().now().nanoseconds,
                    frame=msg.header.frame_id, points=msg.width*msg.height,
                    pose_tf=self.tf.can_transform(self.vehicle+'/odom', self.vehicle+'/base_link', stamp),
                    sensor_tf=self.tf.can_transform(self.vehicle+'/odom', msg.header.frame_id, stamp))
                try:
                    latest = self.tf.lookup_transform(self.vehicle+'/odom', self.vehicle+'/base_link', Time())
                    row['latest_pose_stamp_ns'] = Time.from_msg(latest.header.stamp).nanoseconds
                except Exception as error:
                    row['latest_pose_error'] = str(error)
                with (self.out/'raw-cloud-diagnostic.jsonl').open('a') as stream:
                    stream.write(json.dumps(row)+'\n')
            self.subs.append(self.create_subscription(PointCloud2,
                '/'+self.vehicle+'/sensor_measurements/Lidar/points', diagnose_cloud, qos_profile_sensor_data))
        for key, topic, kind, qos in [
            ('platform', '/drone0/platform/info', PlatformInfo, qos_profile_sensor_data),
            ('mode', '/drone0/fmu/out/vehicle_control_mode', VehicleControlMode, qos_profile_sensor_data),
            ('controller', '/drone0/controller/info', ControllerInfo, qos_profile_sensor_data),
            ('behavior', '/drone0/follow_timed_path/_behavior/behavior_status', BehaviorStatus, 10),
            ('takeoff', '/drone0/TakeoffBehavior/_behavior/behavior_status', BehaviorStatus, 10),
            ('land', '/drone0/LandBehavior/_behavior/behavior_status', BehaviorStatus, 10),
            ('odom', '/drone0/navigation/odom', Odometry, qos_profile_sensor_data),
            ('cloud', '/drone0/navigation/points', PointCloud2, qos_profile_sensor_data),
            ('ego_odom', '/ego/drone0/odom', Odometry, qos_profile_sensor_data),
            ('ego_cloud', '/ego/drone0/points', PointCloud2, qos_profile_sensor_data),
            ('clock', '/clock', Clock, qos_profile_sensor_data),
            ('heartbeat', '/drone0/ego_planner/planning/heartbeat', Empty, 10),
            ('ego', '/ego/status', String, 10)]:
            self.subs.append(self.create_subscription(kind, self.topic_map.get(topic, topic).replace('/drone0/', '/'+self.vehicle+'/'),
                lambda m, k=key: self.rows.update({k: (m, time.monotonic())}), qos))
        self.actions = [ActionClient(self, kind, '/'+self.vehicle+'/' + name) for kind, name in
                        [(Takeoff, 'TakeoffBehavior'), (Land, 'LandBehavior'),
                         (FollowTimedPath, 'follow_timed_path')]]
        self.probe_count = self.probe_invalid = 0
        if self.config.get('hybrid'):
            from traj_utils.msg import MINCOTraj
            self.peer = 'drone1' if self.vehicle == 'drone0' else 'drone0'
            self.subs.append(self.create_subscription(Odometry, '/'+self.peer+'/navigation/odom',
                lambda m:self.rows.update(peer_odom=(m,time.monotonic())), qos_profile_sensor_data))
            prefix=probe_prefix(self.config['run_id'])
            self.probe_pub=self.create_publisher(MINCOTraj,prefix+self.vehicle+'/minco',10)
            def receive_probe(msg):
                if (msg.drone_id==int(self.peer[-1]) and msg.order==5 and list(msg.duration)==[1.,2.]
                    and list(msg.inner_x)==[1.,2.,3.] and list(msg.end_p)==[4.,5.,6.]):
                    self.probe_count+=1
                    self.rows['peer_probe']=(msg,time.monotonic())
                else:self.probe_invalid+=1
            self.subs.append(self.create_subscription(MINCOTraj,prefix+self.peer+'/minco',receive_probe,10))
            self.probe_message=MINCOTraj(drone_id=int(self.vehicle[-1]),order=5,
                duration=[1.,2.],inner_x=[1.,2.,3.],end_p=[4.,5.,6.])
            self.probe_timer=self.create_timer(.1,lambda:self.probe_pub.publish(self.probe_message))

    def snapshot(self):
        now = time.monotonic()
        fresh = lambda keys: all(k in self.rows and now-self.rows[k][1] < .75 for k in keys)
        platform = self.rows.get('platform', (None,))[0]
        mode = self.rows.get('mode', (None,))[0]
        behavior = self.rows.get('behavior', (None,))[0]
        try:
            ego = json.loads(self.rows['ego'][0].data) if 'ego' in self.rows else {}
        except (ValueError, TypeError):
            ego = {}
        inputs = fresh(['clock', 'odom', 'cloud', 'controller'])
        if inputs:
            cloud, odom = self.rows['cloud'][0], self.rows['odom'][0]
            sim = self.get_clock().now().nanoseconds
            inputs = bool(cloud.width*cloud.height and sim > 0 and
                          cloud.header.frame_id == self.vehicle+'/odom' and odom.header.frame_id == self.vehicle+'/odom')
            for msg in (cloud, odom):
                age = (sim - msg.header.stamp.sec*10**9 - msg.header.stamp.nanosec)/1e9
                inputs = inputs and -.01 <= age <= .5
        tf_ready = all(self.tf.can_transform(self.vehicle+'/odom', self.vehicle+'/'+frame, Time())
                       for frame in ('base_link', 'Lidar'))
        idle = fresh(['behavior', 'takeoff', 'land']) and all(
            self.rows[k][0].status == BehaviorStatus.IDLE for k in ('behavior', 'takeoff', 'land'))
        state = dict(written_at=time.time(), fresh=fresh(['platform', 'mode', 'behavior', 'ego']),
            armed=platform.armed if platform else None,
            px4_armed=mode.flag_armed if mode else None,
            behavior_idle=idle, tf_ready=tf_ready,
            inputs_ready=inputs and tf_ready and fresh(['heartbeat', 'ego_odom', 'ego_cloud'])
                         and self.count_publishers('/clock') == 1,
            interfaces_ready=all(a.server_is_ready() for a in self.actions),
            ego_state=ego.get('state'), ego_goals=ego.get('counts', {}).get('goals'),
            stop_confirmed=ego.get('stop_confirmed', False), error=self.error,
            children={name: dict(pid=p.pid, returncode=p.poll()) for name, p, _ in self.children})
        state['ready'] = capability_ready(state) and not self.error and all(p.poll() is None for _, p, _ in self.children)
        state['receipt_ages'] = {key: now-received for key, (_, received) in self.rows.items()}
        state['clock_publishers'] = self.count_publishers('/clock')
        state['vehicle'] = self.vehicle
        state['interface_scope'] = ego.get('interface_scope')
        state['owned_vehicles'] = ego.get('vehicles')
        if self.config.get('hybrid'):
            state['peer_odom_fresh'] = fresh(['peer_odom'])
            state['minco_type_probe'] = dict(received=self.probe_count, invalid=self.probe_invalid,
                fresh=fresh(['peer_probe']), scope='isolated acceptance topic; not native planning output')
        state['input_details'] = {}
        sim = self.get_clock().now().nanoseconds
        for key in ('odom', 'cloud', 'ego_odom', 'ego_cloud'):
            if key in self.rows:
                msg = self.rows[key][0]
                state['input_details'][key] = dict(frame=msg.header.frame_id,
                    stamp_age=(sim-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9)
                if hasattr(msg, 'width'):
                    state['input_details'][key]['points'] = msg.width*msg.height
        return state

    def write(self, state):
        temp = self.out / 'status.tmp'
        temp.write_text(json.dumps(state))
        temp.replace(self.out / 'status.json')
        if time.monotonic()-self.last_history >= .5:
            with (self.out/'status-history.jsonl').open('a') as history:
                history.write(json.dumps(state)+'\n')
            self.last_history = time.monotonic()

    def start(self, name, args):
        if self.config.get('dds_launch_prefix'):
            args = args[:2] + ['--launch-prefix', sys.executable+' '+self.config['dds_launch_prefix']] + args[2:]
        if self.config.get('dds_transport') == 'tcp_board_listener':
            args = args[:2] + ['--launch-prefix', sys.executable+' '+str(ROOT/'dds_exec.py')] + args[2:]
        stream = (self.out / (name + '.log')).open('w')
        proc = subprocess.Popen(args, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
        self.children.append((name, proc, stream))

    def run(self):
        if self.config.get('diagnostic_core_limit_bytes'):
            import resource
            limit=int(self.config['diagnostic_core_limit_bytes'])
            if not 0<limit<=1024**3:raise ValueError('Diagnostic core limit must be at most 1 GiB')
            resource.setrlimit(resource.RLIMIT_CORE,(limit,limit))
            os.chdir(self.out)
        os.environ.update(EGO_GROUND_CONFIG=str(ROOT/'as2.yaml'),
            EGO_GROUND_CONTROLLER=str(ROOT/'controller.yaml'), EGO_GROUND_FLEET=str(ROOT/'fleet.json'),
            EGO_GROUND_STATE=str(self.out))
        self.start('as2', ['ros2', 'launch', str(ROOT/'ground.launch.py')])
        deadline = time.monotonic()+90
        ego_started = False
        next_status = 0.0
        while True:
            rclpy.spin_once(self, timeout_sec=.05)
            # Drain incoming DDS callbacks without a filesystem write and graph
            # walk after every message. Monitoring remains at 10 Hz; stop gates
            # below still use a newly sampled state.
            if time.monotonic() < next_status:
                continue
            next_status = time.monotonic() + .1
            state = self.snapshot()
            if any(p.poll() is not None for _, p, _ in self.children):
                self.error = 'Child exited; retain remaining control services for inspection/recovery'
            if not ego_started and not self.error:
                if all(k in self.rows and time.monotonic()-self.rows[k][1] < .75
                       for k in ('platform', 'mode', 'odom', 'cloud', 'clock')):
                    if state['armed'] or state['px4_armed'] or self.count_publishers(self.topic_map.get('/ego/goal', '/ego/goal')):
                        self.error = 'Requires disarmed platform and no existing goal publisher'
                    elif state['interfaces_ready']:
                        launch = [str(ROOT/'fault.launch.py')] if self.config.get('fault_case') else ([self.config['ego_launch_file']] if self.config.get('ego_launch_file') else ['uav_ego_planner_adapter','ego_as2.launch.py'])
                        self.start('ego', ['ros2', 'launch', *launch,
                            'resolved_config:='+str(ROOT/'ego.json'), 'use_sim_time:=true',
                            *(['vehicle:='+self.config['vehicle']] if self.config.get('vehicle') else [])])
                        ego_started = True
                if time.monotonic() > deadline:
                    self.error = 'AS2 input readiness timed out; inspect logs'
            self.write(state)
            request = self.out/'stop.request'
            if request.exists() or self.stop_requested:
                self.stop_requested = False
                # If EGO was never started it cannot own an execution goal.
                stop_state = self.snapshot()
                if not ego_started:
                    stop_state.update(ego_state='WAITING', fresh=all(
                        k in self.rows and time.monotonic()-self.rows[k][1] < .75
                        for k in ('platform', 'mode', 'behavior')))
                if not safe_to_stop(stop_state):
                    (self.out/'stop-denied.json').write_text(json.dumps(stop_state))
                    request.unlink(missing_ok=True)
                    continue
                for name, proc, stream in reversed(self.children):
                    if proc.poll() is None:
                        # ros2 launch forwards SIGINT to its owned nodes. Sending
                        # to the whole group gives nodes a second SIGINT as well.
                        os.kill(proc.pid, signal.SIGINT)
                    # Adapter shutdown can still call the execution stop service.
                    # Keep AS2/controller alive until EGO and its descendants exit.
                    try:
                        proc.wait(timeout=12)
                    except subprocess.TimeoutExpired:
                        self.error = 'Child did not stop: '+name
                        break
                    stream.flush()
                    try:
                        os.killpg(proc.pid, 0)
                    except ProcessLookupError:
                        pass
                    else:
                        self.error = 'Descendants remain: '+name
                        break
                else:
                    (self.out/'stopped.json').write_text(json.dumps(dict(safe_state=stop_state,
                        children={n:p.returncode for n,p,_ in self.children})))
                    request.unlink(missing_ok=True)
                    return
                request.unlink(missing_ok=True)


def main():
    rclpy.init()
    node = Runtime(Path(sys.argv[1]))
    def stop(*_):
        node.stop_requested = True
    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    try:
        node.run()
    finally:
        import faulthandler
        faulthandler.dump_traceback_later(10, repeat=False)
        print(json.dumps(dict(lifecycle='destroy_node_begin',wall=time.time(),mono=time.monotonic())),flush=True)
        node.destroy_node()
        print(json.dumps(dict(lifecycle='destroy_node_end',wall=time.time(),mono=time.monotonic())),flush=True)
        rclpy.try_shutdown()
        print(json.dumps(dict(lifecycle='shutdown_end',wall=time.time(),mono=time.monotonic())),flush=True)
        faulthandler.cancel_dump_traceback_later()


if __name__ == '__main__':
    main()
