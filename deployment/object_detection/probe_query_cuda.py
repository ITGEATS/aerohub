"""Saved real RGB/depth + recorded TF replay through Module/Tool/Engine CUDA.

Repetition uses synthetic replay stamps and a held recorded camera transform.
This validates transport/measurement contracts, not live sensing or 3D accuracy.
No simulator, physical device, motion publisher or navigation client is used.
"""
import argparse
from array import array
import hashlib
import json
from pathlib import Path
import threading
import time
import cv2
import numpy as np
import rclpy
from as2_python_api.drone_interface_base import DroneInterfaceBase
from geometry_msgs.msg import TransformStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import TransformBroadcaster
from uav_inference.grounding_dino_node import GroundingDinoNode
from uav_inference_interfaces.srv import InferGroundingDino
from uav_object_detection.node import ObjectDetectionTool
from uav_target_tracking.node import TrackingNode


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--input-dir',type=Path,required=True)
    p.add_argument('--output',type=Path,required=True)
    a=p.parse_args()
    files={key:a.input_dir/name for key,name in dict(rgb='hover-parked.png',
        depth='hover-parked-depth.npy',tf='camera-tf.json',geometry='hover-parked-geometry.json').items()}
    image=cv2.imread(str(files['rgb']))
    if image is None:raise ValueError('saved image missing')
    depth_image=np.load(files['depth'],allow_pickle=False).astype(np.float32)
    h,w=image.shape[:2]
    assert depth_image.shape==(h,w)
    saved_tf=json.loads(files['tf'].read_text())['transforms']['earth']
    geometry=json.loads(files['geometry'].read_text())
    # Saved render projection matrix fixes fx=fy=320 for this 640x480 image.
    matrix=geometry['camera']['proj_mat']['matrix']
    fx,fy=abs(matrix[0][1])*w/2,abs(matrix[1][2])*h/2
    rclpy.init(args=['--ros-args','-r','__ns:=/query_replay','-p','detector_backend:=grounding_dino',
        '-p','query:=car','-p','timeout_s:=10.0','-p','registered_depth:=true','-p','output_frame:=earth'])
    adapter,tool,fixture=GroundingDinoNode(),ObjectDetectionTool(),Node('saved_pickup_replay')
    tracking=TrackingNode()
    tool._ensure_position_inputs()
    base='sensor_measurements/FrontCamera/'
    rgb=fixture.create_publisher(Image,base+'rgb/image',qos_profile_sensor_data)
    depth=fixture.create_publisher(Image,base+'depth_planar/image',qos_profile_sensor_data)
    ri=fixture.create_publisher(CameraInfo,base+'rgb/camera_info',qos_profile_sensor_data)
    di=fixture.create_publisher(CameraInfo,base+'depth_planar/camera_info',qos_profile_sensor_data)
    broadcaster=TransformBroadcaster(fixture)
    tf=TransformStamped();tf.header.frame_id='earth';tf.child_frame_id=saved_tf['child']
    for key,value in zip(('x','y','z'),saved_tf['translation']):setattr(tf.transform.translation,key,value)
    for key,value in zip(('x','y','z','w'),saved_tf['quaternion']):setattr(tf.transform.rotation,key,value)
    published={}
    def publish():
        frame=Image(width=w,height=h,encoding='bgr8',step=w*3,data=array('B',image.tobytes()))
        frame.header.frame_id=saved_tf['child'];frame.header.stamp=fixture.get_clock().now().to_msg()
        info=CameraInfo(header=frame.header,width=w,height=h,k=[fx,0.,w/2,0.,fy,h/2,0.,0.,1.],d=[0.]*5)
        ri.publish(info);di.publish(info)
        tf.header.stamp=frame.header.stamp;broadcaster.sendTransform(tf)
        depth.publish(Image(header=frame.header,width=w,height=h,encoding='32FC1',step=w*4,
            data=array('B',depth_image.tobytes())))
        rgb.publish(frame)
        published[(frame.header.stamp.sec,frame.header.stamp.nanosec)]=frame
    fixture.create_timer(.5,publish)
    executor=MultiThreadedExecutor(num_threads=8)
    for node in (adapter,tool,fixture,tracking):executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    drone=DroneInterfaceBase('query_replay',spin_rate=100.);drone.load_module('object_detection');drone.load_module('target_tracking')
    result=dict(passed=False,scope='saved real RGB-D / held recorded TF / synthetic replay stamps; no live sensor or 3D accuracy proof',
        hashes={key:hashlib.sha256(path.read_bytes()).hexdigest() for key,path in files.items()},
        recorded_tf=saved_tf,intrinsics=[fx,fy,w/2,h/2],calls=[])
    try:
        deadline=time.monotonic()+20
        while not (drone.object_detection.ready and tool.client.service_is_ready() and tool.projector.depth):
            if time.monotonic()>deadline:raise TimeoutError('graph readiness')
            time.sleep(.02)
        health=adapter.detector.client.request('/health/ready')
        assert health['device'].startswith('cuda'),health
        result['health']=health
        # Capture exact Engine phrase/boxes for the same full image and normalized query.
        client=fixture.create_client(InferGroundingDino,'inference/grounding_dino/infer')
        source=list(published.values())[-1]
        reference=InferGroundingDino.Request(request_id='query-reference',generation=91,image=source,
            prompt='car.',box_threshold=.30,text_threshold=.25)
        future=client.call_async(reference)
        deadline=time.monotonic()+20
        while not future.done():
            if time.monotonic()>deadline:raise TimeoutError('Engine reference')
            time.sleep(.01)
        expected=future.result();assert expected.success,expected.error
        result['engine_reference']=[dict(category=d.category,xyxy=list(d.xyxy),confidence=d.confidence) for d in expected.result.detections]
        for query,position in [('car',False),(' Car ',True),('car',True)]:
            accepted=tool.get_clock().now().nanoseconds
            started=time.monotonic()
            response=drone.object_detection.detect(query=query,timeout=15.,estimate_position=position)
            assert response.success,response.reason
            observed=response.result
            assert observed.original_query==query and observed.normalized_query=='car.'
            stamp=observed.image_header.stamp
            stamp_ns=stamp.sec*10**9+stamp.nanosec
            assert stamp_ns>=accepted
            assert (stamp.sec,stamp.nanosec) in published
            assert observed.model_version==expected.result.model_version
            assert observed.worker_generation==expected.result.worker_generation
            assert len(observed.detections)==len(expected.result.detections)>0
            for actual,ref in zip(observed.detections,expected.result.detections):
                assert actual.detection.category==ref.category
                assert not actual.detection.mask.data and actual.detection.mask.width==0
                np.testing.assert_allclose(actual.detection.xyxy,ref.xyxy,atol=.02,rtol=1e-5)
                if actual.position_valid:
                    assert position and actual.position_method=='box_foreground_surface_v1'
                    assert observed.header.frame_id=='earth' and observed.header.stamp==observed.depth_header.stamp
                elif position:assert actual.position_error
            result['calls'].append(dict(query=query,position_requested=position,code=response.code,reason=response.reason,
                stamp_ns=stamp_ns,model_version=observed.model_version,worker_generation=observed.worker_generation,
                request_id=observed.request_id,tool_generation=observed.tool_generation,roundtrip_ms=(time.monotonic()-started)*1000,
                image_frame=observed.image_header.frame_id,output_frame=observed.header.frame_id,
                detections=[dict(category=d.detection.category,xyxy=list(d.detection.xyxy),confidence=d.detection.confidence,
                    position_valid=d.position_valid,position_method=d.position_method,position_error=d.position_error,
                    position=[d.position.x,d.position.y,d.position.z],depth_m=d.depth_m,valid_depth_pixels=d.valid_depth_pixels)
                    for d in observed.detections]))
        assert len({c['stamp_ns'] for c in result['calls']})==3
        feedback=[]
        submission=drone.target_tracking.track(query='car',timeout=2.,
            feedback_callback=lambda msg:feedback.append(msg.feedback.state))
        deadline=time.monotonic()+5
        while not submission.done() and time.monotonic()<deadline:time.sleep(.01)
        assert submission.done() and submission.result().accepted
        completed=submission.result().get_result_async()
        while not completed.done() and time.monotonic()<deadline:time.sleep(.01)
        assert completed.done() and completed.result().result.code=='TIMEOUT'
        assert feedback and all(not s.valid for s in feedback)
        result['tracking']=dict(query='car',code=completed.result().result.code,
            feedback_count=len(feedback),valid_count=sum(s.valid for s in feedback),
            reasons=sorted({s.reason for s in feedback}),
            scope='actual Tracking consumes positioned query; unreliable replay depth stays unavailable')
        # Drain a possibly outstanding observation before shutting down its executor.
        deadline=time.monotonic()+12
        while tool.snapshot.gate.locked() and time.monotonic()<deadline:time.sleep(.02)
        assert not tool.snapshot.gate.locked()
        result['passed']=True
    finally:
        a.output.write_text(json.dumps(result,indent=2)+'\n')
        tool.stop_event.set();tool.worker.join(2)
        drone.shutdown();drone.destroy_node()
        executor.shutdown(timeout_sec=5);thread.join(5)
        for node in (adapter,tool,fixture,tracking):node.destroy_node()
        rclpy.try_shutdown()
    print(json.dumps(result))


if __name__=='__main__':main()
