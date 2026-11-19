"""Actual Module -> Service -> RGB-D adapter -> ROS inference -> CUDA worker.

Public real-image input, synthetic depth/calibration/TF. No live accuracy claim.
Worker must already be running on its configured endpoint.
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
from tf2_ros import StaticTransformBroadcaster
from uav_inference.yolo_node import YoloNode
from uav_object_detection.node import ObjectDetectionTool


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image',type=Path,required=True)
    parser.add_argument('--output',type=Path,required=True)
    args=parser.parse_args()
    image=cv2.imread(str(args.image))
    if image is None:raise ValueError('real input image unavailable')
    h,w=image.shape[:2]
    rclpy.init(args=['--ros-args','-r','__ns:=/probe'])
    adapter,tool=YoloNode(),ObjectDetectionTool()
    tool.position_config['registered']=True
    tool._ensure_position_inputs()
    fixture=Node('real_image_fixture')
    rgb=fixture.create_publisher(Image,'sensor_measurements/FrontCamera/rgb/image',qos_profile_sensor_data)
    depth=fixture.create_publisher(Image,'sensor_measurements/FrontCamera/depth_planar/image',qos_profile_sensor_data)
    ri=fixture.create_publisher(CameraInfo,'sensor_measurements/FrontCamera/rgb/camera_info',qos_profile_sensor_data)
    di=fixture.create_publisher(CameraInfo,'sensor_measurements/FrontCamera/depth_planar/camera_info',qos_profile_sensor_data)
    broadcaster=StaticTransformBroadcaster(fixture)
    tf=TransformStamped();tf.header.frame_id='probe/odom';tf.child_frame_id='camera_optical';tf.transform.rotation.w=1.
    broadcaster.sendTransform(tf)
    def publish():
        frame=Image(width=w,height=h,encoding='bgr8',step=w*3,data=array('B',image.tobytes()))
        frame.header.frame_id='camera_optical';frame.header.stamp=fixture.get_clock().now().to_msg()
        info=CameraInfo(header=frame.header,width=w,height=h,k=[w/2,0.,(w-1)/2,0.,w/2,(h-1)/2,0.,0.,1.],d=[0.]*5)
        ri.publish(info);di.publish(info)
        depth.publish(Image(header=frame.header,width=w,height=h,encoding='32FC1',step=w*4,
                            data=array('B',np.full((h,w),2.,np.float32).tobytes())))
        rgb.publish(frame)
    fixture.create_timer(.1,publish)
    executor=MultiThreadedExecutor(num_threads=8)
    for node in (adapter,tool,fixture):executor.add_node(node)
    thread=threading.Thread(target=executor.spin,daemon=True);thread.start()
    drone=DroneInterfaceBase('probe');drone.load_module('object_detection')
    try:
        deadline=time.monotonic()+5
        while not (drone.object_detection.ready and tool.projector.depth):
            if time.monotonic()>deadline:raise TimeoutError('public graph not ready')
            time.sleep(.01)
        health=adapter.detector.client.request('/health/ready')
        assert health['device'].startswith('cuda'),health
        response=drone.object_detection.detect(query='person',timeout=2.,estimate_position=True)
        assert response.success,response.reason
        assert response.result.detections,'Expected people in real bus image'
        assert all(d.detection.category=='person' and d.position_valid for d in response.result.detections),str(response)
        assert response.result.header.frame_id=='probe/odom'
        assert all(abs(d.position.z-2.)<1e-6 for d in response.result.detections)
        report=dict(passed=True,scope='real RGB and CUDA; synthetic depth/calibration/TF, not live accuracy or flight',
                    image_sha256=hashlib.sha256(args.image.read_bytes()).hexdigest(),health=health,
                    code=response.code,detections=len(response.result.detections),all_positions_valid=True)
        args.output.write_text(json.dumps(report,indent=2)+'\n');print(json.dumps(report))
    finally:
        tool.stop_event.set();tool.worker.join(timeout=2)
        drone.shutdown();drone.destroy_node();executor.shutdown(timeout_sec=3);thread.join(timeout=3)
        for node in (adapter,tool,fixture):node.destroy_node()
        rclpy.shutdown()


if __name__=='__main__':main()
