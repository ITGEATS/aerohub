"""Ground camera acceptance: real AirSim ROS RGB-D -> public Module -> CUDA.

No AirSim motion calls, synthetic sensor publisher, arming or takeoff operation.
"""
import argparse
from collections import deque
import json
from pathlib import Path
import threading
import time

import cv2
import numpy as np
import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase
from as2_msgs.msg import PlatformInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from uav_inference.yolo_node import YoloNode
from uav_object_detection.node import ObjectDetectionTool


def ns(header):return header.stamp.sec*10**9+header.stamp.nanosec


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--estimate-position',action='store_true')
    parser.add_argument('--near-target',action='store_true')
    parser.add_argument('--samples',type=int,default=10)
    parser.add_argument('--collect-failures',action='store_true')
    args=parser.parse_args();args.output.mkdir(exist_ok=True)
    rclpy.init(args=['--ros-args','-r','__ns:=/drone0','-p','use_sim_time:=true',
                       '--params-file','/repo/deployment/object_detection/params.example.yaml',
                       '-p','camera_reliable:=true'])
    observer=Node('camera_acceptance')
    images,depths=deque(maxlen=100),deque(maxlen=100)
    info={};platform=[];received=[]
    state_trace=[]
    def trace(kind,header):
        state_trace.append(dict(kind=kind,wall=time.monotonic(),stamp_ns=ns(header),
                                clock_ns=observer.get_clock().now().nanoseconds))
    def tf_trace(message):
        for transform in message.transforms:
            if transform.child_frame_id=='drone0/base_link':trace('tf',transform.header)
    observer.create_subscription(TFMessage,'/tf',tf_trace,qos_profile_sensor_data)
    observer.create_subscription(Odometry,'sensor_measurements/odom',
                                 lambda m:trace('odom',m.header),qos_profile_sensor_data)
    camera_qos=QoSProfile(depth=5,reliability=ReliabilityPolicy.RELIABLE)
    base='sensor_measurements/FrontCamera/'
    def rgb(m):images.append(m);received.append((time.monotonic(),ns(m.header),observer.get_clock().now().nanoseconds-ns(m.header)))
    observer.create_subscription(Image,base+'rgb/image',rgb,camera_qos)
    if args.estimate_position:
        observer.create_subscription(Image,base+'depth_planar/image',depths.append,camera_qos)
        for kind in ('rgb','depth_planar'):
            observer.create_subscription(CameraInfo,base+kind+'/camera_info',lambda m,k=kind:info.update({k:m}),camera_qos)
    observer.create_subscription(PlatformInfo,'platform/info',lambda m:platform.append(m),qos_profile_sensor_data)
    tool=adapter=drone=None
    executor=MultiThreadedExecutor(num_threads=8);executor.add_node(observer)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    result=dict(passed=False,position_requested=args.estimate_position,scope='ground real AirSim / real CUDA; optional RGB-D and PX4-AS2 TF; no flight or position-accuracy claim')
    def wait(predicate,timeout=40):
        deadline=time.monotonic()+timeout
        while not predicate():
            if time.monotonic()>deadline:raise TimeoutError('camera/graph readiness timeout')
            time.sleep(.02)
    try:
        wait(lambda:len(images)>=10 and platform and
             (not args.estimate_position or (len(depths)>=10 and len(info)==2)))
        assert not platform[-1].armed,'Requires a disarmed platform'
        for m in ([images[-1],depths[-1]] if args.estimate_position else [images[-1]]):
            assert (m.width,m.height)==(640,480)
            assert m.header.frame_id=='drone0/FrontCamera_optical'
            assert ns(m.header)>0
        assert images[-1].encoding=='rgb8' and images[-1].step==640*3
        if args.estimate_position:
            assert depths[-1].encoding=='32FC1' and depths[-1].step==640*4
            assert list(info['rgb'].k)==list(info['depth_planar'].k)
            assert not any(info['rgb'].d) and not any(info['depth_planar'].d)
            sample=list(images)[-10:]
            # Freeze the RGB sample before allowing bounded depth transport arrival;
            # continually choosing the newest RGB would race its paired depth.
            pair_deadline=time.monotonic()+.3
            while time.monotonic()<pair_deadline:
                skews=[min(abs(ns(r.header)-ns(d.header))/1e9 for d in depths) for r in sample]
                if max(skews)<=.05:break
                time.sleep(.01)
            result.update(rgb_sample_stamps=[ns(r.header) for r in sample],
                          depth_received_stamps=[ns(d.header) for d in depths],pair_skews_s=skews)
            assert max(skews)<=.05,skews
        raw=images[-1];pixels=np.frombuffer(bytes(raw.data),np.uint8).reshape(raw.height,raw.step)[:,:raw.width*3].reshape(raw.height,raw.width,3)
        cv2.imwrite(str(args.output/'rgb.png'),cv2.cvtColor(pixels,cv2.COLOR_RGB2BGR))
        result.update(rgb_encoding=raw.encoding,frame=raw.header.frame_id,
                      dimensions=[raw.width,raw.height],unique_rgb_stamps=len({r[1] for r in received}),
                      rgb_receive_hz=(len(received)-1)/(received[-1][0]-received[0][0]),
                      rgb_receive_age_s=[r[2]/1e9 for r in received])
        if args.estimate_position:
            dep=depths[-1];arr=np.frombuffer(bytes(dep.data),'<f4').reshape(dep.height,dep.width)
            np.save(args.output/'depth-m.npy',arr)
            result.update(depth_encoding=dep.encoding,camera_k=list(info['rgb'].k),
                          max_pair_skew_s=max(skews),depth_finite_fraction=float(np.isfinite(arr).mean()))
        adapter=YoloNode();tool=ObjectDetectionTool()
        executor.add_node(adapter);executor.add_node(tool)
        drone=DroneInterfaceBase('drone0',use_sim_time=True);drone.load_module('object_detection')
        from rclpy.time import Time
        if args.estimate_position:
            tool._ensure_position_inputs()
        def measurement_ready():
            if not drone.object_detection.ready:
                return False
            if not args.estimate_position:
                return True
            if not tool.projector.depth:
                return False
            header=tool.projector.depth[-1][0].header
            try:
                with tool.projector.lock:
                    for kind in ('rgb', 'depth'):
                        tool.projector._info_at(kind, ns(header))
            except ValueError:
                return False
            return tool.tf_buffer.can_transform('drone0/odom',header.frame_id,Time.from_msg(header.stamp))
        wait(measurement_ready)
        health=adapter.detector.client.request('/health/ready');assert health['device'].startswith('cuda')
        outcomes=[]
        failures=[]
        result['diagnostic_failures']=failures
        result['observations']=outcomes
        for index,category in enumerate(['car']*args.samples if args.near_target else ['person','car','bus']):
            before={ns(m.header) for m in images}
            started=time.monotonic();response=drone.object_detection.detect(query=category,timeout=2.,estimate_position=args.estimate_position)
            result['last_response']=dict(category=category,code=response.code,reason=response.reason,
                start_wall=started,end_wall=time.monotonic(),rgb_stamp_ns=ns(response.result.image_header),
                depth_stamp_ns=ns(response.result.depth_header),
                detections=[dict(category=d.detection.category,confidence=d.detection.confidence,
                    position_valid=d.position_valid,position_error=d.position_error) for d in response.result.detections])
            if args.collect_failures and (not response.success or
                    not any(d.position_valid and 2 <= d.depth_m <= 8 for d in response.result.detections)):
                failures.append(dict(result['last_response']))
                continue
            assert response.success,response.code+': '+response.reason
            assert ns(response.result.image_header) not in before,'Reused cached frame'
            assert any(ns(m.header)==ns(response.result.image_header) for m in images),'Returned RGB stamp not from camera bridge'
            assert response.result.position_requested==args.estimate_position
            skew=None
            if args.estimate_position:
                assert response.result.header.frame_id=='drone0/odom'
                skew=abs(ns(response.result.image_header)-ns(response.result.depth_header))/1e9
                assert ns(response.result.depth_header)>0 and skew<=.05
                # Verify historical TF exists at the actual measurement timestamp even
                # when this scene contains none of the requested category.
                from rclpy.time import Time
                tool.tf_buffer.lookup_transform('drone0/odom','drone0/FrontCamera_optical',Time.from_msg(response.result.depth_header.stamp))
            else:
                assert tool.projector is None and tool.tf_buffer is None
                assert response.code=='SUCCEEDED'
                assert response.result.header==response.result.image_header
                assert ns(response.result.depth_header)==0
                assert all(not d.position_valid and not d.position_error for d in response.result.detections)
            scoring = None
            if args.near_target:
                from uav_object_detection.projection import rotation
                # Other distant cars may remain PARTIAL; the declared near-target
                # check requires a valid near object, not positions for every car.
                valid=[d for d in response.result.detections if d.position_valid and 2 <= d.depth_m <= 8]
                assert valid,'No valid car position'
                measured=max(valid,key=lambda d:d.detection.confidence)
                assert 2 <= measured.depth_m <= 8,measured.depth_m
                stamp=ns(response.result.depth_header)
                with tool.projector.lock:
                    paired=next((pair for pair in tool.projector.depth if ns(pair[0].header)==stamp),None)
                    assert paired is not None,'Paired depth no longer available'
                    depth=paired[1].copy()
                tf=tool.tf_buffer.lookup_transform('drone0/odom','drone0/FrontCamera_optical',Time.from_msg(response.result.depth_header.stamp))
                r=rotation(tf.transform.rotation)
                p=tf.transform.translation
                translation=np.array([p.x,p.y,p.z])
                world=np.array([measured.position.x,measured.position.y,measured.position.z])
                optical=r.T@(world-translation)
                assert abs(optical[2]-measured.depth_m)<=.05
                prefix=args.output/('near-%02d'%index)
                source=next(m for m in images if ns(m.header)==ns(response.result.image_header))
                rgb_pixels=np.frombuffer(bytes(source.data),np.uint8).reshape(480,640,3)
                cv2.imwrite(str(prefix)+'.png',cv2.cvtColor(rgb_pixels,cv2.COLOR_RGB2BGR))
                np.save(str(prefix)+'-depth.npy',depth)
                mask=np.frombuffer(bytes(measured.detection.mask.data),np.uint8).reshape(480,640)
                cv2.imwrite(str(prefix)+'-mask.png',mask*255)
                scoring=dict(optical=optical.tolist(),position=world.tolist(),rotation=r.tolist(),
                             translation=translation.tolist(),box=list(measured.detection.xyxy),
                             depth_m=measured.depth_m,valid_depth_pixels=measured.valid_depth_pixels)
            outcomes.append(dict(category=category,code=response.code,reason=response.reason,scoring=scoring,
                objects=[dict(category=d.detection.category,position_valid=d.position_valid,
                    position_error=d.position_error,depth_m=d.depth_m,valid_depth_pixels=d.valid_depth_pixels)
                    for d in response.result.detections],detections=len(response.result.detections),
                valid_positions=sum(d.position_valid for d in response.result.detections),rgb_stamp_ns=ns(response.result.image_header),
                depth_stamp_ns=ns(response.result.depth_header),pair_skew_s=skew,wall_s=time.monotonic()-started))
        assert not platform[-1].armed
        assert not failures, f'{len(failures)} diagnostic calls failed; see diagnostic_failures'
        result.update(passed=True,health=health,observations=outcomes,disarmed=True)
    except Exception as error:
        result['error']=str(error)
        raise
    finally:
        (args.output/'state-trace.json').write_text(json.dumps(state_trace,indent=2)+'\n')
        (args.output/'result.json').write_text(json.dumps(result,indent=2)+'\n')
        print(json.dumps(result),flush=True)
        if tool is not None:tool.stop_event.set();tool.worker.join(timeout=2)
        if drone is not None:drone.shutdown();drone.destroy_node()
        executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        for node in (tool,adapter,observer):
            if node is not None:node.destroy_node()
        rclpy.try_shutdown()


if __name__=='__main__':main()
