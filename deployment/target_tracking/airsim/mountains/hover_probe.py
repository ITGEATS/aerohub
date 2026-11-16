"""Take off/hover using AS2, confirm parked target, then observe moving convoy.

No navigation client, pursuit, raw motion publisher or AirSim flight API.
"""
import argparse
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time
import traceback

import numpy as np
import rclpy
from rclpy.action.graph import get_action_names_and_types
from rclpy.executors import MultiThreadedExecutor
from rclpy.time import Time
from uav_inference.grounding_dino_node import GroundingDinoNode
from uav_object_detection.node import ObjectDetectionTool
from uav_target_tracking.node import TrackingNode
from flight import Flight
sys.path.insert(0,'/repo/simulator/airsim/sensors')
from uav_as2_sensor_adapter.rpc import Rpc


def read_records(path):
    if not path.exists():return []
    # Ignore only an unfinished last write while tailing the live producer.
    data=path.read_text();lines=data.splitlines()
    if not data.endswith('\n'):lines=lines[:-1]
    return [json.loads(line) for line in lines]


class IndependentFlightComplete(Exception):
    pass


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--flight-only',action='store_true')
    parser.add_argument('--query',default='car')
    a=parser.parse_args();a.output.mkdir(parents=True,exist_ok=True)
    rclpy.init(args=['--ros-args','-r','__ns:=/drone0','-p','use_sim_time:=true',
        '--params-file','/repo/deployment/target_tracking/params.yaml',
        '-p','camera_reliable:=true','-p','output_frame:=earth'])
    flight=Flight(a.output)
    inference,detector,tracker=GroundingDinoNode(),ObjectDetectionTool(),TrackingNode()
    executor=MultiThreadedExecutor(num_threads=8)
    for node in (flight,inference,detector,tracker):executor.add_node(node)
    inference_calls=[];projection_calls=[];predicate_samples=[]
    detect=inference.detector.infer
    def record_detect(*args,**kwargs):
        dest=a.output/'cuda-inputs';dest.mkdir(exist_ok=True)
        index=len(inference_calls)
        np.save(dest/(str(index)+'.npy'),args[0])
        import cv2
        cv2.imwrite(str(dest/(str(index)+'.png')),args[0])
        flight.event('cuda_input',path='cuda-inputs/'+str(index)+'.npy',
            shape=list(args[0].shape),dtype=str(args[0].dtype),encoding='bgr8',metadata=list(args[1:]))
        reply=detect(*args,**kwargs)
        boxes=reply['boxes'];transform=dict(masks='none',normalized_prompt=reply['normalized_prompt'])
        inference_calls.append(dict(wall=time.time(),boxes=[dict(category=b.get('category'),confidence=b.get('confidence'),xyxy=b.get('xyxy')) for b in boxes],transform=transform))
        return reply
    inference.detector.infer=record_detect
    infer=detector.infer
    def record_projection(*args,**kwargs):
        try:
            f=infer(*args,**kwargs)
            projection_calls.append(dict(wall=time.time(),stamp=f.header.stamp.sec+f.header.stamp.nanosec*1e-9,
                frame=f.header.frame_id,detections=[dict(category=d.detection.category,valid=d.position_valid,
                    p=[d.position.x,d.position.y,d.position.z],error=d.position_error) for d in f.detections]))
            return f
        except Exception as exc:
            projection_calls.append(dict(wall=time.time(),monotonic=time.monotonic(),ros=detector.now(),
                error=repr(exc),traceback=traceback.format_exc(),last_call=detector.last_call,
                image_header=str(args[0].image.header),depth_stamps=[str(x[0].header.stamp) for x in detector.projector.depth]));raise
    detector.infer=record_projection
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    rpc=Rpc('127.0.0.1',49471,3)
    truth_samples=[];truth_stop=threading.Event()
    def observe_physics():
        observer=Rpc('127.0.0.1',49471,3)
        try:
            while not truth_stop.is_set():
                try:truth_samples.append(dict(wall=time.time(),phase=flight.phase,pose=observer.call('simGetVehiclePose','PX4'),collision=observer.call('simGetCollisionInfo','PX4'),own=flight.snapshot()))
                except Exception as exc:truth_samples.append(dict(error=repr(exc)))
                truth_stop.wait(.2)
        finally:observer.close()
    truth_thread=threading.Thread(target=observe_physics,daemon=True);truth_thread.start()
    mission=None;mission_log=None;convoy=[]
    result=dict(passed=False,scope=('independent native takeoff/hover/land; no observation' if a.flight_only else 'simulated AS2 takeoff/hover, parked then moving RGB-D observation; no pursuit'),
                convoy_started=False,moving_complete=False,cleanup_errors=[])
    stream=a.output/'mission/mission.jsonl'
    stop_file=a.output/'stop-mission'

    def status():
        s=json.loads(rpc.call('simGetConvoyStatus'))
        convoy.append(dict(wall=time.time(),monotonic=time.monotonic(),status=s,
            pickup_pose=rpc.call('simGetObjectPose','01Pickup-Leader')))
        return s
    def stopped(s):
        v=s.get('vehicles',[])
        return len(v)==3 and all(x['available'] and math.isfinite(x['speed_mps']) and abs(x['speed_mps'])<=.1 for x in v)
    def capture(name):
        try:
            frames=rpc.call('simGetImages',[dict(camera_name='FrontCamera',image_type=0,pixels_as_float=False,compress=True),
                dict(camera_name='FrontCamera',image_type=1,pixels_as_float=True,compress=False)],'PX4',False)
            if len(frames)!=2 or any(x['width']<=0 or x['height']<=0 for x in frames):
                raise ValueError('empty/incomplete RGBD response: '+repr([{k:v for k,v in x.items() if not k.startswith('image_data')} for x in frames]))
            (a.output/(name+'.png')).write_bytes(bytes(frames[0]['image_data_uint8']))
            np.save(a.output/(name+'-depth.npy'),np.array(frames[1]['image_data_float'],np.float32).reshape(frames[1]['height'],frames[1]['width']))
            (a.output/(name+'-geometry.json')).write_text(json.dumps(dict(
                camera=rpc.call('simGetCameraInfo','FrontCamera','PX4',False),
                acquisition_stamps=[x['time_stamp'] for x in frames],own=flight.snapshot(),
                truth_scoring_only=rpc.call('simGetVehiclePose','PX4')),indent=2)+'\n')
        except Exception as exc:
            detail=dict(name=name,error=repr(exc),traceback=traceback.format_exc(),flight=flight.snapshot())
            (a.output/(name+'-image-failure.json')).write_text(json.dumps(detail,indent=2)+'\n')
            result['cleanup_errors'].append('camera '+name+': '+repr(exc))
    def ensure_hover():
        s=flight.snapshot();p=s.get('pose',{}).get('p');v=s.get('twist',{}).get('v')
        predicate_samples.append(dict(wall=time.time(),monotonic=time.monotonic(),ros=s['ros_time'],
            phase=flight.phase,snapshot=s,ground=flight.ground,
            height_error=abs(p[2]-flight.ground[2]-3.) if p and flight.ground else None,
            xy_drift=math.dist(p[:2],flight.ground[:2]) if p and flight.ground else None,
            speed=math.sqrt(sum(x*x for x in v)) if v else None,
            healthy=flight.healthy(airborne=True),hover=flight.hover_check()))
        if not flight.hover_check():
            flight.event('hover_guard_failed',snapshot=flight.snapshot())
            raise RuntimeError('hover height/speed/drift/state/freshness guard failed')
    def follow_records():return [v for v in read_records(stream) if v['event']=='follow_point']
    def static_confirmed():
        follows=follow_records();unique={v['observation_stamp']:v for v in follows if v['tracker_status']=='tracked'}
        positions=[v['target'] for v in unique.values()]
        spread=max((math.dist(x,y) for x in positions for y in positions),default=float('inf'))
        ids=len({v['capture_id'] for v in unique.values()})
        predicate_samples.append(dict(wall=time.time(),monotonic=time.monotonic(),
            ros=flight.snapshot()['ros_time'],phase='static_gate',unique_stamps=list(unique),
            capture_ids=[v['capture_id'] for v in unique.values()],spread_m=spread if math.isfinite(spread) else None,
            stamps_pass=len(unique)>=4,captures_pass=ids>=4,spread_pass=spread<=1.))
        return len(unique)>=4 and ids>=4 and spread<=1.
    def drain_mission():
        if mission is None:return
        if mission.poll() is None:
            stop_file.touch()
            try:mission.wait(timeout=12)
            except subprocess.TimeoutExpired:
                mission.send_signal(signal.SIGINT)
                try:mission.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    mission.kill();mission.wait(timeout=3)
                    result['cleanup_errors'].append('Mission required kill; tracking drain unconfirmed')
        result['mission_exit']=mission.returncode
        summary=a.output/'mission/summary.json'
        if summary.exists():result['mission_summary']=json.loads(summary.read_text())

    try:
        detector._ensure_position_inputs()
        flight.preflight()
        result['preflight_passed']=True
        initial=status()
        assert stopped(initial) and not initial.get('running'), 'convoy must be parked before flight'
        result['initial_convoy']=initial
        result['actions']=get_action_names_and_types(flight)
        result['health']=inference.detector.client.request('/health/ready')
        assert result['health']['model_id']=='grounding-dino-tiny' and result['health']['device'].startswith('cuda')
        capture('ground')
        flight.takeoff()
        result['takeoff_hover_passed']=True
        capture('hover-parked')
        if a.flight_only:
            deadline=time.monotonic()+3.
            while time.monotonic()<deadline:
                ensure_hover();time.sleep(.1)
            result['independent_hover_complete']=True
            # Exit observation work through a local success marker; finally performs Land.
            raise IndependentFlightComplete()
        def camera_tf_ready():
            if not detector.projector.depth:return False
            header=detector.projector.depth[-1][0].header
            return detector.tf_buffer.can_transform('earth',header.frame_id,Time.from_msg(header.stamp))
        flight.wait(camera_tf_ready,5,'capture-time camera TF')
        header=detector.projector.depth[-1][0].header
        camera_tf={}
        for parent in ('drone0/base_link','earth'):
            tf=detector.tf_buffer.lookup_transform(parent,header.frame_id,Time.from_msg(header.stamp))
            t=tf.transform.translation;q=tf.transform.rotation
            camera_tf[parent]=dict(parent=tf.header.frame_id,child=tf.child_frame_id,
                stamp=tf.header.stamp.sec+tf.header.stamp.nanosec*1e-9,
                translation=[t.x,t.y,t.z],quaternion=[q.x,q.y,q.z,q.w])
        (a.output/'camera-tf.json').write_text(json.dumps(dict(wall=time.time(),
            depth_stamp=header.stamp.sec+header.stamp.nanosec*1e-9,transforms=camera_tf),indent=2)+'\n')
        mission_log=(a.output/'mission-console.log').open('w')
        command=['python3','/repo/deployment/airsim/demos/target_tracking/mission.py',
            '--query',a.query,'--frame','earth','--rate','10','--duration','32',
            '--initial-wait','10','--stop-file',str(stop_file),'--output',str(a.output/'mission')]
        result['mission_command']=command
        mission=subprocess.Popen(command,stdout=mission_log,stderr=subprocess.STDOUT)
        flight.phase='parked_observation'
        deadline=time.monotonic()+12
        while not static_confirmed():
            ensure_hover()
            assert stopped(status()), 'vehicle moved before valid static observation'
            if mission.poll() is not None:raise RuntimeError('Mission exited during parked observation')
            if time.monotonic()>deadline:raise TimeoutError('four unique stable parked target observations unavailable')
            time.sleep(.1)
        result['parked_tracked_count']=len({v['observation_stamp'] for v in follow_records() if v['tracker_status']=='tracked'})
        result['parked_target_confirmed']=True
        capture('hover-static-confirmed')
        ensure_hover()
        flight.phase='moving_observation'
        start=time.time();moving_start=time.monotonic()
        result['moving_start_wall']=start
        result['convoy_start']=json.loads(rpc.call('simStartConvoy'))
        assert result['convoy_start']['ok'] and result['convoy_start']['running']
        result['convoy_started']=True
        deadline=moving_start+12
        while time.monotonic()<deadline:
            ensure_hover();status()
            if mission.poll() is not None:raise RuntimeError('Mission exited during moving observation')
            time.sleep(.1)
        result['moving_duration_s']=time.monotonic()-moving_start
        result['moving_complete']=True
        capture('hover-moving-end')
    except IndependentFlightComplete:
        pass
    except (Exception,KeyboardInterrupt) as exc:
        result['error']=repr(exc);result['traceback']=traceback.format_exc()
        result['failure_flight_snapshot']=flight.snapshot()
        result['failure_mission_tail']=read_records(stream)[-12:]
        capture('failure')
    finally:
        # Stop the scene vehicles, drain the observation producer, then native Land.
        # There is no separate hold publisher to compete with landing.
        try:
            result['convoy_stop']=json.loads(rpc.call('simStopConvoy'))
            end=time.monotonic()+10;stable=0
            while time.monotonic()<end:
                stable=stable+1 if stopped(status()) else 0
                if stable>=5:break
                time.sleep(.2)
            result['convoy_stopped']=stable>=5
        except Exception as exc:result['cleanup_errors'].append('convoy stop: '+repr(exc))
        drain_mission()
        try:flight.recover()
        except Exception as exc:
            result['cleanup_errors'].append('native landing recovery: '+repr(exc))
            flight.save()
        result['flight']=flight.result
        result['final_flight_snapshot']=flight.snapshot()
        if flight.result.get('disarmed_ground'):
            (a.output/'SAFE_TO_CLEANUP.json').write_text(json.dumps(dict(disarmed_ground=True,wall=time.time()))+'\n')
        result['mission_records']=len(read_records(stream))
        moving=[v for v in follow_records() if v['wall_time']>=result.get('moving_start_wall',float('inf'))]
        unique={v['observation_stamp']:v for v in moving if v['tracker_status']=='tracked'}
        positions=[v['target'] for v in unique.values()]
        result['moving_unique_tracked']=len(unique)
        result['moving_capture_ids']=len({v['capture_id'] for v in unique.values()})
        result['moving_target_displacement_m']=max((math.dist(x,y) for x in positions for y in positions),default=0.)
        result['geometry_finite']=bool(moving) and all(math.isfinite(x) for v in moving for x in v['point']+v['own']+v['target']+[v['raw_range']])
        result['altitude_preserved']=bool(moving) and all(abs(v['point'][2]-v['own'][2])<=1e-9 for v in moving)
        truth=[[v['pickup_pose']['position'][k] for k in ('x_val','y_val','z_val')] for v in convoy]
        result['independent_pickup_displacement_m']=max((math.dist(x,y) for x in truth for y in truth),default=0.)
        result['passed']=bool(not result.get('error') and result.get('takeoff_hover_passed') and result.get('parked_target_confirmed')
            and result['moving_complete'] and len(unique)>=4 and result['moving_capture_ids']>=4
            and result['moving_target_displacement_m']>1. and result['geometry_finite'] and result['altitude_preserved']
            and result.get('convoy_stopped') and result.get('mission_exit')==0 and flight.result['land_success']
            and flight.result['disarmed_ground'] and not result['cleanup_errors'])
        if a.flight_only:
            result['passed']=bool(result.get('independent_hover_complete') and flight.result['land_success'] and flight.result['disarmed_ground'] and result.get('convoy_stopped') and not result['cleanup_errors'])
        truth_stop.set();truth_thread.join(4)
        (a.output/'physics-samples.json').write_text(json.dumps(truth_samples,indent=2)+'\n')
        def save_evidence():
            for name,data in [('result',result),('convoy',convoy),('inference-calls',inference_calls),('projection-calls',projection_calls),('predicate-samples',predicate_samples)]:
                (a.output/(name+'.json')).write_text(json.dumps(data,indent=2)+'\n')
        # Persist the outcome before teardown: a destructor exception must not erase it.
        save_evidence()
        tracker._stopping.set()
        operations=[('executor',lambda:executor.shutdown(timeout_sec=5)),
                    ('executor thread',lambda:thread.join(5))]
        operations += [(node.get_name(),node.destroy_node) for node in (tracker,detector,inference,flight)]
        operations += [('rclpy',rclpy.try_shutdown),('RPC',rpc.close)]
        if mission_log is not None:operations.append(('Mission log',mission_log.close))
        for label,operation in operations:
            try:operation()
            except Exception as exc:
                result['passed']=False
                result['cleanup_errors'].append(label+': '+repr(exc))
        if thread.is_alive():result['passed']=False;result['cleanup_errors'].append('executor thread still alive')
        save_evidence()
        print(json.dumps(result))
    return 0 if result['passed'] else 1
if __name__=='__main__':raise SystemExit(main())
