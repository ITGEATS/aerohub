"""Cross-check physical AirSim camera geometry and actual ROS static optical TF."""
import argparse
import json
import math
from pathlib import Path
import sys

ROOT=Path(__file__).resolve().parents[4]
sys.path.insert(0,str(ROOT/'simulator/airsim/sensors'))
from uav_as2_sensor_adapter.conversion import multiply, extrinsic


def quaternion(p):return tuple(p[k] for k in ('x_val','y_val','z_val','w_val'))
def position(p):return tuple(p[k] for k in ('x_val','y_val','z_val'))
def angle(a,b):
    dot=abs(sum(x*y for x,y in zip(a,b)))/math.sqrt(sum(x*x for x in a)*sum(x*x for x in b))
    return math.degrees(2*math.acos(min(1.,dot)))


def evaluate(run):
    run=Path(run);scene=json.loads((run/'scene-reference.json').read_text())
    config=json.loads((run/'settings.json').read_text())['Vehicles']['PX4']['Cameras']['FrontCamera']
    body=scene['vehicle_pose'];cam=scene['camera'];q=quaternion(body['orientation'])
    offset=tuple(config.get(k,0) for k in ('X','Y','Z'))
    rotated=multiply(multiply(q,(*offset,0)),(-q[0],-q[1],-q[2],q[3]))[:3]
    expected=tuple(x+y for x,y in zip(position(body['position']),rotated))
    r,p,y=(math.radians(config.get(k,0))/2 for k in ('Roll','Pitch','Yaw'))
    cq=multiply(multiply((0,0,math.sin(y),math.cos(y)),(0,math.sin(p),0,math.cos(p))),(math.sin(r),0,0,math.cos(r)))
    actual_pos_error=math.dist(expected,position(cam['pose']['position']))
    actual_angle=angle(multiply(q,cq),quaternion(cam['pose']['orientation']))
    tf=json.loads((run/'probe/camera-tf.json').read_text())
    observed=tf['transforms']['drone0/base_link'];translation,rotation=extrinsic(config,True)
    report=dict(scope='stationary physical geometry + actual sample-time ROS static optical TF, not absolute earth localization accuracy',
        tolerances=dict(translation_m=.02,rotation_deg=.2,focal_px=.01),
        physical_translation_error_m=actual_pos_error,physical_rotation_error_deg=actual_angle,
        tf_translation_error_m=math.dist(translation,observed['translation']),
        tf_rotation_error_deg=angle(rotation,observed['quaternion']),
        capture_tf=tf,expected_body_optical=dict(translation=translation,quaternion=rotation),
        reported_camera_fov_deg=cam['fov'])
    capture=config['CaptureSettings'][0];width=capture['Width'];height=capture['Height']
    focal=width/(2*math.tan(math.radians(capture['FOV_Degrees'])/2))
    matrix=cam['proj_mat']['matrix'];physical_fx=abs(matrix[0][1])*width/2;physical_fy=abs(matrix[1][2])*height/2
    report.update(config_focal_px=focal,physical_projection_focal_px=[physical_fx,physical_fy],
        reported_fov_note='RPC scalar differs slightly; actual rendering projection matrix agrees with configured CameraInfo geometry')
    checks=dict(physical_offset=actual_pos_error<=.02,physical_rotation=actual_angle<=.2,
        ros_offset=report['tf_translation_error_m']<=.02,ros_rotation=report['tf_rotation_error_deg']<=.2,
        intrinsics=max(abs(focal-x) for x in (physical_fx,physical_fy))<=.01)
    report.update(checks=checks,passed=all(checks.values()))
    return report

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('run',type=Path);a=p.parse_args()
    report=evaluate(a.run);(a.run/'camera-evaluation.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps(report,indent=2));raise SystemExit(0 if report['passed'] else 1)
