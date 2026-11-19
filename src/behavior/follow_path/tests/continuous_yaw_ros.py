"""Real compiled Behavior/service protocol; fake localization/controller, not flight."""
import math
import os
from pathlib import Path
import signal
import subprocess
from ament_index_python.packages import get_package_share_directory
import rclpy
from uav_planner_interfaces.srv import UpdateTimedYaw
from ros_smoke import Test, stamp

class ContinuousYawTest(Test):
    def run(self):
        assert self.action.wait_for_server(timeout_sec=8)
        client=self.create_client(UpdateTimedYaw,'/drone0/follow_timed_path/update_yaw')
        assert client.wait_for_service(timeout_sec=5)
        self.spin(.5)
        goal=self.goal(final=False,duration=3.,lead=.2)
        p=goal.path;p.valid_fields=3;p.continuous_yaw=True
        p.target_yaw=.8;p.max_yaw_rate=1.5;p.yaw_revision=1
        stamp(self.get_clock().now().nanoseconds/1e9+1.,p.yaw_valid_until)
        handle=self.wait(self.action.send_goal_async(goal));assert handle.accepted
        self.spin(.5)
        first=self.trajectory_references[-1].setpoints[0].yaw_angle
        assert first>.05,first
        def update(owner,seq,yaw,life=1.):
            req=UpdateTimedYaw.Request(task_id=owner,revision=seq,target_yaw=yaw)
            stamp(self.get_clock().now().nanoseconds/1e9+life,req.valid_until)
            return self.wait(client.call_async(req))
        assert not update('wrong',2,-.8).accepted
        assert not update('test',1,-.8).accepted
        assert update('test',2,-.8).accepted
        # No new position plan submitted: yaw still reverses in the same execution.
        # Allow bounded deceleration from positive yaw rate before reversal.
        self.spin(.8)
        assert self.trajectory_references[-1].setpoints[0].yaw_angle<first
        result=handle.get_result_async();self.wait(handle.cancel_goal_async())
        assert self.wait(result).status==5
        assert not update('test',3,0.).accepted
        self.spin(.2)
        goal=self.goal(final=False,duration=2.,lead=.1)
        p=goal.path;p.valid_fields=3;p.continuous_yaw=True;p.target_yaw=.5
        p.max_yaw_rate=1.5;p.yaw_revision=1
        stamp(self.get_clock().now().nanoseconds/1e9+.5,p.yaw_valid_until)
        handle=self.wait(self.action.send_goal_async(goal));assert handle.accepted
        result=self.wait(handle.get_result_async())
        assert not result.result.success and result.result.reason=='YAW_TARGET_EXPIRED',result.result.reason
        assert not update('test',2,0.).accepted
        print('PASS: same executor yaw updates, wrong/stale owner rejected, cancel/expiry stop')

rclpy.init();node=ContinuousYawTest()
log=open('/tmp/continuous-yaw-behavior.log','w')
process=subprocess.Popen(['ros2','run','uav_follow_path_execution','follow_path_extended_node','--ros-args',
 '--params-file',str(Path(get_package_share_directory('as2_behaviors_motion'))/'follow_path_behavior/config/config_default.yaml'),
 '-r','__ns:=/drone0','-p','plugin_name:=follow_path_plugin_position','-p','follow_path_speed:=1.0',
 '-p','follow_path_threshold:=0.2','-p','tf_timeout_threshold:=0.1'],stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
try:node.run()
except BaseException:
    log.flush();print(Path('/tmp/continuous-yaw-behavior.log').read_text());raise
finally:
    os.killpg(process.pid,signal.SIGINT)
    try:process.wait(timeout=8)
    except subprocess.TimeoutExpired:os.killpg(process.pid,signal.SIGKILL);process.wait()
    log.close();node.destroy_node();rclpy.shutdown()
