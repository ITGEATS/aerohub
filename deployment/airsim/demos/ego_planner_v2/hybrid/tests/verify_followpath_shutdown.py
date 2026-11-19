"""Real FollowPath signal shutdown with TF-dependent input; no flight commands."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_prefix

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output',type=Path)
    args=parser.parse_args();args.output.mkdir(parents=True,exist_ok=True)
    prefix=Path(get_package_prefix('uav_follow_path_execution'))
    binary=prefix/'lib/uav_follow_path_execution/follow_path_extended_node'
    rclpy.init();node=rclpy.create_node('followpath_shutdown_check')
    publisher=node.create_publisher(PoseStamped,'/shutdown_probe/self_localization/pose',qos_profile_sensor_data)
    rows=[];proc=None
    try:
        for sig in (signal.SIGINT,signal.SIGTERM):
            for trial in range(3):
                with (args.output/f'{sig.name}-{trial}.log').open('w') as log:
                    proc=subprocess.Popen([str(binary),'--ros-args','-r','__ns:=/shutdown_probe',
                        '--params-file','/root/aerostack2_ws/install/share/as2_behaviors_motion/follow_path_behavior/config/config_default.yaml',
                        '-p','plugin_name:=follow_path_plugin_position','-p','vehicle_id:=shutdown_probe','-p','use_sim_time:=false'],stdout=log,stderr=subprocess.STDOUT)
                    deadline=time.monotonic()+5
                    while publisher.get_subscription_count()==0 and time.monotonic()<deadline and proc.poll() is None:
                        rclpy.spin_once(node,timeout_sec=.02)
                    subscribers=publisher.get_subscription_count()
                    for _ in range(15):
                        msg=PoseStamped();msg.header.frame_id='earth';msg.header.stamp=node.get_clock().now().to_msg();msg.pose.orientation.w=1.
                        publisher.publish(msg);rclpy.spin_once(node,timeout_sec=.01)
                    start=time.monotonic();forced=False
                    if proc.poll() is None:proc.send_signal(sig)
                    try:code=proc.wait(timeout=5)
                    except subprocess.TimeoutExpired:proc.kill();proc.wait();code=proc.returncode;forced=True
                rows.append(dict(signal=sig.name,trial=trial,subscribers=subscribers,returncode=code,forced=forced,elapsed_s=time.monotonic()-start))
                until=time.monotonic()+3
                while publisher.get_subscription_count() and time.monotonic()<until:rclpy.spin_once(node,timeout_sec=.02)
    finally:
        if proc and proc.poll() is None:proc.kill();proc.wait()
        node.destroy_node();rclpy.shutdown()
    result=dict(passed=all(x['subscribers']>0 and x['returncode']==0 and not x['forced'] for x in rows),binary=str(binary),cases=rows,
                scope='Isolated real-node signal checks, not aircraft or long-term shutdown reliability')
    (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result));return 0 if result['passed'] else 1

if __name__=='__main__':raise SystemExit(main())
