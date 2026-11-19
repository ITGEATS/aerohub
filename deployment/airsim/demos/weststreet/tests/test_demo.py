"""Mission lifecycle with Tool peers. These tests do not claim ROS/flight evidence."""
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import time
from types import ModuleType, SimpleNamespace as Obj
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
spec = importlib.util.spec_from_file_location('weststreet_demo', ROOT/'demo.py')
demo = importlib.util.module_from_spec(spec)
spec.loader.exec_module(demo)


class Future:
    def __init__(self, value):
        self.value = value
    def done(self):
        return True
    def result(self):
        return self.value


class ResultFuture(Future):
    def done(self):
        return self.value is not None


class Handle:
    accepted = True
    def __init__(self, result=None, stop=True):
        self.result_future = ResultFuture(result)
        self.stop = stop
    def get_result_async(self):
        return self.result_future
    def cancel_goal_async(self):
        self.result_future.value = Obj(result=Obj(stop_confirmed=self.stop))
        return Future(None)


def stamp(value):
    return Obj(sec=int(value), nanosec=int((value-int(value))*1e9))


class Drone:
    ready = True
    stop = True
    lose_target = False
    instance = None
    def __init__(self, *args, **kwargs):
        type(self).instance = self
        self.system = self.navigation = self.target_tracking = self
        self.calls = []
        self.armed = False
        self.done = False
    def load_module(self, name):
        pass
    def get_clock(self):
        return Obj(now=lambda:Obj(nanoseconds=int(time.monotonic()*1e9)))
    def get_state(self):
        return Future(Obj(fresh=True, armed=self.armed, busy=False, offboard=self.armed,
            navigation_pose_fresh=True, navigation_pose=Obj(header=Obj(frame_id='drone0/odom'),
                     pose=Obj(position=Obj(x=0., y=0., z=3. if self.armed else 0.)))))
    def takeoff(self, **kwargs):
        self.calls.append('takeoff')
        self.armed = True
        return Future(Handle(Obj(result=Obj(success=True, code='SUCCEEDED', reason='',
            stop_confirmed=True, landed=False, disarmed=False))))
    def land(self):
        self.calls.append('land')
        self.armed = False
        return Future(Handle(Obj(result=Obj(success=True, code='SUCCEEDED', reason='',
            stop_confirmed=True, landed=True, disarmed=True))))
    def track(self, query, timeout, feedback, *, follow):
        self.calls.append('track')
        assert query == 'person' and follow is False
        now = time.monotonic()
        self.target = Obj(valid=True, status='tracked', reason='', session_id='test', generation=1, capture_id=1,
            observation_stamp=stamp(now), valid_until=stamp(now+1),
            header=Obj(frame_id='drone0/odom', stamp=stamp(now)), position=Obj(x=10.,y=0.,z=1.),
            velocity=Obj(x=0.,y=0.,z=0.))
        feedback(Obj(feedback=Obj(state=self.target)))
        return Future(Handle())
    def to_point(self, *point, **kwargs):
        self.calls.append(('navigate', point))
        if self.lose_target:
            self.target.valid = False
            self.target.status = 'lost'
        return True
    def cancel(self, **kwargs):
        self.calls.append('cancel_navigation')
        if not self.stop:
            raise RuntimeError('Stop unconfirmed')
        return True
    def shutdown(self):
        pass
    def destroy_node(self):
        pass


class DemoTests(unittest.TestCase):
    def run_demo(self, *, stop=True, lose=False):
        Drone.stop, Drone.lose_target = stop, lose
        ros = ModuleType('rclpy')
        ros.init = ros.try_shutdown = lambda:None
        api = ModuleType('as2_python_api.drone_interface_base')
        api.DroneInterfaceBase = Drone
        with tempfile.TemporaryDirectory() as folder:
            output = Path(folder)/'evidence'
            with patch.dict(sys.modules, {'rclpy':ros, 'as2_python_api':ModuleType('as2_python_api'),
                    'as2_python_api.drone_interface_base':api}), patch.object(sys, 'argv',
                    ['demo.py','--output',str(output),'--duration','.15','--update-period','.02',
                     '--observation-grace','.04']):
                try:
                    demo.main()
                except SystemExit as exc:
                    self.assertEqual(exc.code, 1)
            return json.loads((output/'result.json').read_text()), Drone.instance.calls

    def test_navigation_stops_before_landing(self):
        result, calls = self.run_demo()
        self.assertTrue(result['success'])
        self.assertLess(calls.index('cancel_navigation'), calls.index('land'))
        points = [item[1] for item in calls if isinstance(item, tuple)]
        self.assertGreaterEqual(len(points), 2)
        self.assertTrue(all(point == (2., 0., 3.) for point in points))

    def test_lost_target_cancels_and_lands(self):
        result, calls = self.run_demo(lose=True)
        self.assertFalse(result['success'])
        self.assertIn('No fresh observation', result['error'])
        self.assertLess(calls.index('cancel_navigation'), calls.index('land'))

    def test_unconfirmed_navigation_does_not_start_competing_land(self):
        result, calls = self.run_demo(stop=False)
        self.assertFalse(result['success'])
        self.assertNotIn('land', calls)
        self.assertTrue(result['cleanup_errors'])
