"""Real Detect/Tracking services and an unmodified orchestration path in Mission.

Ground fixture only. Scene truth is restricted to setup/independent motion scoring.
"""
import argparse
import importlib.util
import json
import math
from pathlib import Path
import subprocess
import sys
import threading
import time
import rclpy
from as2_msgs.msg import PlatformInfo
from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import VehicleCommand
from rclpy.action.graph import get_action_names_and_types
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from uav_inference.grounding_dino_node import GroundingDinoNode
from uav_object_detection.node import ObjectDetectionTool
from uav_target_tracking.node import TrackingNode
sys.path.insert(0,'/repo/simulator/airsim/sensors')
from uav_as2_sensor_adapter.rpc import Rpc


def main():
    p=argparse.ArgumentParser();p.add_argument('--output',type=Path,required=True)
    p.add_argument('--query',default='car')
    p.add_argument('--estimate-position',action='store_true');a=p.parse_args()
    a.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=['--ros-args','-r','__ns:=/drone0','-p','use_sim_time:=true',
        '--params-file','/repo/deployment/target_tracking/params.yaml','-p','camera_reliable:=true','-p','output_frame:=earth'])
    peer=Node('mountains_observer_guard')
    inference,detector,tracker=GroundingDinoNode(),ObjectDetectionTool(),TrackingNode()
    executor=MultiThreadedExecutor(num_threads=8)
    for node in (peer,inference,detector,tracker):executor.add_node(node)
    platform=[];poses=[];commands=[]
    peer.create_subscription(PlatformInfo,'platform/info',lambda m:platform.append(dict(
        wall=time.time(),armed=m.armed,connected=m.connected,offboard=m.offboard)),qos_profile_sensor_data)
    peer.create_subscription(VehicleCommand,'fmu/in/vehicle_command',lambda m:commands.append(dict(
        wall=time.time(),command=m.command,param1=m.param1,param2=m.param2)),qos_profile_sensor_data)
    peer.create_subscription(PoseStamped,'self_localization/pose',lambda m:poses.append(dict(
        wall=time.time(),frame=m.header.frame_id,stamp=m.header.stamp.sec+m.header.stamp.nanosec*1e-9,
        p=[m.pose.position.x,m.pose.position.y,m.pose.position.z])),qos_profile_sensor_data)
    inference_calls=[];projection_calls=[]
    original_detect=inference.detector.infer
    def record_detect(*args,**kwargs):
        reply=original_detect(*args,**kwargs)
        boxes=reply['boxes'];transform=dict(masks='none',normalized_prompt=reply['normalized_prompt'])
        inference_calls.append(dict(wall=time.time(),boxes=[dict(category=b.get('category'),
            confidence=b.get('confidence'),xyxy=b.get('xyxy')) for b in boxes],
            transform=transform))
        return reply
    inference.detector.infer=record_detect
    original_infer=detector.infer
    def record_projection(*args,**kwargs):
        try:
            frame=original_infer(*args,**kwargs)
            projection_calls.append(dict(wall=time.time(),stamp=frame.header.stamp.sec+frame.header.stamp.nanosec*1e-9,
                frame=frame.header.frame_id,detections=[dict(category=d.detection.category,
                valid=d.position_valid,p=[d.position.x,d.position.y,d.position.z],error=d.position_error)
                for d in frame.detections]))
            return frame
        except Exception as exc:
            projection_calls.append(dict(wall=time.time(),error=repr(exc)));raise
    detector.infer=record_projection
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    rpc=Rpc('127.0.0.1',49471,5)
    convoy=[];mission=None;result=dict(passed=False,scope='moving ground observation only')
    def wait(predicate,timeout=40):
        end=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>end:raise TimeoutError('ground inputs/TF readiness timeout')
            time.sleep(.05)
    def status():
        s=json.loads(rpc.call('simGetConvoyStatus'))
        convoy.append(dict(wall=time.time(),data=s,pickup_pose=rpc.call('simGetObjectPose','01Pickup-Leader')))
        return s
    try:
        detector._ensure_position_inputs()
        wait(lambda: platform and poses and detector.projector.depth)
        assert all(not v['armed'] for v in platform)
        result['initial_convoy']=status()
        result['ground_fixture']=dict(source='dedicated scene spawn; no teleport',
            actual=rpc.call('simGetVehiclePose','PX4'))
        result['camera']=rpc.call('simGetCameraInfo','FrontCamera','PX4',False)
        images=rpc.call('simGetImages',[dict(camera_name='FrontCamera',image_type=0,
            pixels_as_float=False,compress=True),dict(camera_name='FrontCamera',image_type=1,
            pixels_as_float=True,compress=False)],'PX4',False)
        (a.output/'ground-camera.png').write_bytes(bytes(images[0]['image_data_uint8']))
        import numpy as np
        np.save(a.output/'ground-depth.npy',np.array(images[1]['image_data_float'],dtype=np.float32).reshape(images[1]['height'],images[1]['width']))
        result['rgbd_stamps']=[v['time_stamp'] for v in images]
        def ready():
            h=detector.projector.depth[-1][0].header
            return detector.tf_buffer.can_transform('earth',h.frame_id,Time.from_msg(h.stamp))
        wait(ready)
        assert poses[-1]['frame']=='earth'
        result['health']=inference.detector.client.request('/health/ready')
        assert result['health']['model_id']=='grounding-dino-tiny' and result['health']['device'].startswith('cuda')
        result['actions']=get_action_names_and_types(peer)
        result['nodes']=peer.get_node_names_and_namespaces()
        result['start_convoy']=json.loads(rpc.call('simStartConvoy'))
        assert result['start_convoy']['ok']
        command=['python3','/repo/deployment/airsim/demos/target_tracking/mission.py',
            '--query',a.query,'--frame','earth','--rate','10',
            '--duration','12','--initial-wait','10','--output',str(a.output/'mission')]
        result['mission_command']=command
        with (a.output/'mission-console.log').open('w') as out:
            mission=subprocess.Popen(command,stdout=out,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+40
            while mission.poll() is None:
                assert platform and not platform[-1]['armed'], 'armed ground guard'
                status()
                if time.monotonic()>deadline:raise TimeoutError('Mission did not exit')
                time.sleep(.2)
        result['mission_exit']=mission.returncode
        records=[json.loads(line) for line in (a.output/'mission/mission.jsonl').read_text().splitlines()]
        follows=[v for v in records if v['event']=='follow_point']
        tracked={v['observation_stamp']:v for v in follows if v['tracker_status']=='tracked'}
        result['unique_mission_tracked_acquisitions']=len(tracked)
        result['follow_count']=len(follows)
        positions=[v['target'] for v in tracked.values()]
        result['target_displacement_m']=max((math.dist(x,y) for x in positions for y in positions),default=0.)
        result['distinct_capture_ids']=len({v['capture_id'] for v in tracked.values()})
        result['geometry_finite']=bool(follows) and all(math.isfinite(n) for v in follows for n in v['point']+v['own']+v['target']+[v['raw_range']])
        result['altitude_preserved']=bool(follows) and all(abs(v['point'][2]-v['own'][2])<=1e-9 for v in follows)
        result['standoff_geometry']=bool(follows) and all(abs(math.dist(v['point'][:2],v['target'][:2])-
            min(v['distance'],math.dist(v['own'][:2],v['target'][:2])))<=1e-6 for v in follows)
        result['navigation_loaded']=any(v.get('navigation_loaded',False) for v in records if v['event']=='modules_loaded')
        result['summary']=json.loads((a.output/'mission/summary.json').read_text())
        result['business_passed']=(mission.returncode==0 and len(tracked)>=4 and result['target_displacement_m']>1.
            and result['distinct_capture_ids']>=4 and result['standoff_geometry'] and not result['navigation_loaded']
            and result['geometry_finite'] and result['altitude_preserved'] and not result['summary']['motion_requested'])
    except Exception as exc:
        result['error']=repr(exc)
        import traceback
        result['traceback']=traceback.format_exc()
    finally:
        if mission is not None and mission.poll() is None:
            mission.terminate()
            try:mission.wait(timeout=15)
            except subprocess.TimeoutExpired:mission.kill();mission.wait(timeout=3)
        try:
            result['stop_convoy']=json.loads(rpc.call('simStopConvoy'))
            # Verify measured speeds, rather than accepting the stop ACK.
            end=time.monotonic()+10;stable=0
            while time.monotonic()<end:
                s=status();vehicles=s.get('vehicles',[])
                (a.output/'last-convoy.json').write_text(json.dumps(s,indent=2)+'\n')
                speeds=[v.get('speed_mps') for v in vehicles]
                if len(speeds)==3 and all(isinstance(x,(int,float)) and abs(x)<=.1 for x in speeds):stable+=1
                else:stable=0
                if stable>=5:break
                time.sleep(.2)
            result['convoy_stopped']=stable>=5
            xyz=[[v['pickup_pose']['position'][k] for k in ('x_val','y_val','z_val')] for v in convoy]
            result['independent_pickup_displacement_m']=max((math.dist(x,y) for x in xyz for y in xyz),default=0.)
        except Exception as exc:result['convoy_cleanup_error']=repr(exc)
        result['disarmed_throughout']=bool(platform) and all(not v['armed'] for v in platform)
        result['flight_command_count']=sum(v['command'] in (21,22,176,400) for v in commands)
        result['platform_samples']=len(platform)
        tracker._stopping.set();executor.shutdown(timeout_sec=5);thread.join(5)
        for node in (tracker,detector,inference,peer):node.destroy_node()
        rclpy.try_shutdown();rpc.close()
        result['cleanup_complete']=not thread.is_alive()
        result['passed']=bool(result.get('business_passed') and result.get('convoy_stopped') and result['disarmed_throughout'] and result['cleanup_complete'] and result['flight_command_count']==0)
        for name,data in [('result',result),('platform',platform),('poses',poses),('convoy',convoy),('px4-commands',commands),('inference-calls',inference_calls),('projection-calls',projection_calls)]:
            (a.output/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        print(json.dumps(result))
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
