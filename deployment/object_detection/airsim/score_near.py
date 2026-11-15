#!/usr/bin/env python3
"""Test-only independent collision-geometry scoring; never an algorithm input.

AirSim visibility ray casts use scene collision geometry, not the RGB-D buffer.
The test is static and disarmed. Origin is the vehicle home position; camera
position is relative to the stationary vehicle centre. Preserve all raw poses.
"""
import argparse
import json
from pathlib import Path
import sys
import numpy as np

ROOT=Path(__file__).resolve().parents[3]
sys.path[:0]=[str(ROOT/'deployment/robopi/vendor'),str(ROOT/'simulator/airsim/sensors')]
from uav_as2_sensor_adapter.rpc import Rpc


def vector(value):return np.array([value[k+'_val'] for k in ('x','y','z')],dtype=float)


def rotation(value):
    x,y,z,w=[value[k+'_val'] for k in ('x','y','z','w')]
    q=np.array([x,y,z,w]);q/=np.linalg.norm(q);x,y,z,w=q
    return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                     [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                     [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])


def geopoint(ned,home):
    # WGS84 local NED -> ECEF -> geodetic (same convention as AirSim).
    a,e2=6378137.,6.69437999014e-3
    lat,lon=np.deg2rad([home['latitude'],home['longitude']]);h=home['altitude']
    sl,cl,so,co=np.sin(lat),np.cos(lat),np.sin(lon),np.cos(lon)
    radius=a/np.sqrt(1-e2*sl*sl)
    origin=np.array([(radius+h)*cl*co,(radius+h)*cl*so,(radius*(1-e2)+h)*sl])
    basis=np.array([[-sl*co,-sl*so,cl],[-so,co,0],[-cl*co,-cl*so,-sl]])
    x,y,z=origin+basis.T@ned
    longitude=np.arctan2(y,x);p=np.hypot(x,y);latitude=np.arctan2(z,p*(1-e2))
    for _ in range(10):
        radius=a/np.sqrt(1-e2*np.sin(latitude)**2)
        altitude=p/np.cos(latitude)-radius
        latitude=np.arctan2(z,p*(1-e2*radius/(radius+altitude)))
    radius=a/np.sqrt(1-e2*np.sin(latitude)**2)
    altitude=p/np.cos(latitude)-radius
    return dict(latitude=float(np.rad2deg(latitude)),longitude=float(np.rad2deg(longitude)),altitude=float(altitude))


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--samples',type=int,default=10)
    args=parser.parse_args();output=args.output
    result=json.loads((output/'probe/result.json').read_text())
    report=dict(passed=False,reference='AirSim ECC_Visibility scene collision ray; separate from depth rendering',samples=[])
    rpc=Rpc('127.0.0.1',49471,timeout=5.)
    try:
        home=rpc.call('getHomeGeoPoint','PX4')
        camera=rpc.call('simGetCameraInfo','FrontCamera','PX4',False)
        vehicle=rpc.call('simGetVehiclePose','PX4')
        report.update(home=home,camera=camera,vehicle=vehicle)
        # No movement during these ground checks. home is initial vehicle centre.
        camera_local=vector(camera['pose']['position'])-vector(vehicle['position'])
        r=rotation(camera['pose']['orientation'])
        origin=geopoint(camera_local,home)
        assert len(result['observations'])==args.samples
        for sample in result['observations']:
            s=sample['scoring'];optical=np.array(s['optical'])
            ray=r@np.array([1.,optical[0]/optical[2],optical[1]/optical[2]])
            def clear(z):
                return rpc.call('simTestLineOfSightBetweenPoints',origin,geopoint(camera_local+ray*z,home))
            assert clear(.5),'Ray already blocked near camera; invalid scoring setup'
            assert not clear(12.),'No collision surface within scoring ray; invalid setup'
            lo,hi=.5,12.
            for _ in range(12):
                middle=(lo+hi)/2
                if clear(middle):lo=middle
                else:hi=middle
            reference=(lo+hi)/2
            error=abs(s['depth_m']-reference)
            camera_point=np.array(s['optical']);position=np.array(s['position'])
            transformed=np.array(s['rotation'])@camera_point+np.array(s['translation'])
            residual=float(np.linalg.norm(transformed-position))
            record=dict(depth_m=s['depth_m'],collision_depth_m=reference,depth_error_m=error,
                        transform_residual_m=residual,rgb_stamp_ns=sample['rgb_stamp_ns'])
            report['samples'].append(record)
            assert error<=.30,record
            assert residual<=.05,record
        report['passed']=True
    except Exception as exc:
        report['error']=str(exc)
        raise
    finally:
        rpc.close();(output/'near-score.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps(report),flush=True)


if __name__=='__main__':main()
