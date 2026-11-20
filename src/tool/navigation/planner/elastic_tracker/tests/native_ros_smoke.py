"""Synthetic RPC/controller fixture shared by the actual timed-path/HFV smoke.

The current executable probe runs Goal/modify, PVA, hover and coverage failure
through compiled HFV. The retired high-rate source-reference test is removed.
"""
import copy
import importlib.util
import json
import os
from pathlib import Path
import signal
import socketserver
import subprocess
import tempfile
import threading
import time
import sys
import rclpy
import yaml
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from as2_msgs.msg import ControllerInfo, ControlMode, PlatformInfo, PlatformStatus, TrajectorySetpoints
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool
from std_msgs.msg import String
from uav_planner_interfaces.msg import TimedTrajectorySegment
from uav_elastic_tracker_adapter.native_execution import NativeAdapter

ROOT=Path('/workspace')
spec=importlib.util.spec_from_file_location('existing_smoke',ROOT/'src/behavior/follow_path/tests/ros_smoke.py')
old=importlib.util.module_from_spec(spec);spec.loader.exec_module(old)


def ns(stamp):
    return stamp.sec*1_000_000_000+stamp.nanosec


class RPC(socketserver.ThreadingTCPServer):
    allow_reuse_address=True
    daemon_threads=True
    def __init__(self):
        class Handler(socketserver.StreamRequestHandler):
            def handle(inner):
                request=json.loads(inner.rfile.readline())
                assert request['kind']=='latest'
                self.trigger.set()
                inner.wfile.write((json.dumps(dict(ok=True,trajectory=self.latest))+'\n').encode())
        self.latest=None
        self.trigger=threading.Event()
        super().__init__(('127.0.0.1',28500),Handler)  # unchanged proxy endpoint


class Fixture(old.Test):
    def __init__(self):
        self.mode_value=ControlMode(control_mode=ControlMode.HOVER)
        self.flying=True
        self.sources={}
        self.plans=[]
        self.states=[]
        self.injector=None
        super().__init__()
        self.body=self.create_publisher(Odometry,'/drone0/elastic_tracker/odom',10)
        self.target=self.create_publisher(Odometry,'/elastic_tracker/accepted_target_odom',10)
        self.controller_info=self.create_publisher(ControllerInfo,'/drone0/controller/info',10)
        self.source_sub=self.create_subscription(TrajectorySetpoints,'/drone0/follow_timed_path/source_reference',
            lambda m:self.sources.__setitem__(ns(m.header.stamp),m),10)
        self.plan_sub=self.create_subscription(TimedTrajectorySegment,'/planner/trajectory',self.plans.append,10)
        self.state_sub=self.create_subscription(String,'/drone0/elastic_adapter/state',
            lambda m:self.states.append(json.loads(m.data)),10)
        self.test_executor=SingleThreadedExecutor()
        self.test_executor.add_node(self)
        self.goals=self.modifies=0
        self.ack_elapsed=[]

    def set_mode(self,request,response):
        self.mode_value=copy.deepcopy(request.control_mode)
        response.success=True
        return response

    def pose(self):
        msg=old.PoseStamped()
        msg.header.frame_id='drone0/odom'
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.pose.position.x=float(self.x);msg.pose.orientation.w=1.
        self.pub.publish(msg)
        transform=old.TransformStamped();transform.header.stamp=msg.header.stamp
        transform.header.frame_id='earth';transform.child_frame_id='drone0/base_link'
        transform.transform.translation.x=float(self.x);transform.transform.rotation.w=1.
        self.tf.sendTransform(transform)
        twist=old.TwistStamped();twist.header.stamp=msg.header.stamp;twist.header.frame_id='earth'
        self.twist.publish(twist)
        # Explicit fake platform, as in the existing compiled-Behavior smoke.
        info=PlatformInfo()
        info.status.state=PlatformStatus.FLYING if self.flying else PlatformStatus.LANDED
        self.info.publish(info)
        if not hasattr(self,'body'):
            return
        msg=Odometry()
        msg.header.stamp=self.get_clock().now().to_msg()
        msg.header.frame_id='drone0/odom'
        msg.pose.pose.position.x=float(self.x)
        msg.pose.pose.orientation.w=1.
        self.body.publish(msg)
        msg=copy.deepcopy(msg);msg.pose.pose.position.x=2.  # upstream fake_target fixture
        self.target.publish(msg)
        control=ControllerInfo()
        control.header.stamp=self.get_clock().now().to_msg()
        control.input_control_mode=self.mode_value
        self.controller_info.publish(control)
        if self.injector:
            self.injector()

    def wait_until(self,predicate):
        # Existing ROS smoke wait budget / polling period, not new gates.
        deadline=time.monotonic()+6.
        while not predicate() and time.monotonic()<deadline:
            self.test_executor.spin_once(timeout_sec=.02)
        assert predicate(),'existing 6s observation budget elapsed'

    def spin(self,duration):
        deadline=time.monotonic()+duration
        while time.monotonic()<deadline:
            self.test_executor.spin_once(timeout_sec=.02)

    def wait(self,future,timeout=6):
        self.wait_until(future.done)
        return future.result()

    def enable(self,value):
        return self.wait(self.enable_client.call_async(SetBool.Request(data=value)))

    def spy(self,adapter):
        action=adapter.client.actions['drone0']
        original=action.send_goal_async
        def send(*args,**kwargs):
            self.goals+=1
            started=time.monotonic()
            if getattr(self,'reject_goal',False):
                # Deliberate invalid-version request; the real executor rejects.
                args[0].path.interface_version=0
            future=original(*args,**kwargs)
            future.add_done_callback(lambda f:self.ack_elapsed.append(time.monotonic()-started))
            return future
        action.send_goal_async=send
        modifier=adapter.client.modifiers['drone0']
        original_modify=modifier.call_async
        def modify(*args,**kwargs):
            self.modifies+=1
            return original_modify(*args,**kwargs)
        modifier.call_async=modify

    def curve(self,identifier,position=None,start=None):
        # Coefficient/order/3s coverage: existing test_core.py + ros_smoke goal.
        stamp=self.get_clock().now().nanoseconds if start is None else start
        return dict(traj_id=identifier,hover=False,hover_p=[],order=5,duration=[3.],yaw=.4,
            start_time=list(divmod(stamp,1_000_000_000)),
            generated_at=list(divmod(self.get_clock().now().nanoseconds,1_000_000_000)),
            coef_x=[0.,0.,0.,0.,1. if position is None else 0.,0. if position is None else float(position)],
            # Unit-coefficient polynomial basis also exercises nonzero V/A.
            coef_y=[0.,0.,0.,1.,0.,0.] if position is None else [0.]*6,
            coef_z=[0.,0.,1.,0.,0.,0.] if position is None else [0.]*6)

    def verify_references(self,after=0):
        references=[m for m in self.trajectory_references if ns(m.header.stamp)>=after]
        assert references,'no actual AS2 trajectory reference'
        for actual in references:
            source=self.sources.get(ns(actual.header.stamp))
            assert source is not None,'stamp was altered or source was not observed'
            assert actual.header.frame_id==source.header.frame_id
            a,b=actual.setpoints[0],source.setpoints[0]
            for field in ('position','twist','acceleration'):
                for axis in ('x','y','z'):
                    assert abs(getattr(getattr(a,field),axis)-getattr(getattr(b,field),axis))<=1e-12
            assert abs(a.yaw_angle-b.yaw_angle)<=1e-12
        stamps=[ns(m.header.stamp) for m in references]
        assert all(a<b for a,b in zip(stamps,stamps[1:])), 'duplicate/old reference stamp'
        return len(references)


def launch_entry():
    """Exercise the installed standalone launch, without enabling execution."""
    rclpy.init()
    node=Node('native_launch_probe');states=[]
    node.create_subscription(String,'/drone0/elastic_adapter/state',lambda m:states.append(json.loads(m.data)),10)
    process=subprocess.Popen(['ros2','launch','uav_elastic_tracker_adapter','native_source.launch.py',
        'execution_mode:=timed_path','use_sim_time:=false'],stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,text=True,start_new_session=True)
    try:
        # Deployment discovery budget: deployment/onboard_computer/check_nodes.py:24.
        # Includes launch/Python startup; the Fixture's business waits remain 6s.
        deadline=time.monotonic()+25.
        while not states and time.monotonic()<deadline:
            rclpy.spin_once(node,timeout_sec=.02)
        assert states,'standalone launch did not publish state'
        assert states[-1]['execution_mode']=='timed_path' and not states[-1]['enabled']
        assert states[-1]['revision']==0 and states[-1]['stop_confirmed']
        assert 'planner_framework' not in node.get_node_names()
        print('PASS: installed native launch publishes inactive state; no framework/Goal',flush=True)
    finally:
        os.killpg(process.pid,signal.SIGINT)
        output,_=process.communicate(timeout=10)
        if sys.exc_info()[0] is not None:
            print(output,flush=True)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':
    launch_entry()
