"""Reuse the accepted PC crossing geometry with scoped mixed compute owners."""
import json
from pathlib import Path
import sys
sys.path.insert(0,'/input')
from crossing_flight_mission import Crossing,MINCOTraj,OffboardControlMode,qos_profile_sensor_data
from flight_client import MixedFleet,rclpy

class MixedCrossing(Crossing,MixedFleet):
    def __init__(self):
        # MRO: Crossing methods -> MixedFleet scoped services -> original Fleet.
        MixedFleet.__init__(self,'short')
        self.minimum_initial_separation=3.5
        self.check_independent_route_tube=False
        self.report['stage']='crossing'
        self.report['gates'].update(goal_distance_m='equal-length crossing diagonals',
            goal_timeout_wall_s=150,max_horizontal_excursion_m=9.,route_tube_radius_m=None,
            straight_nominal_min_separation_max_m=.2,initial_height_difference_max_m=.2,
            swarm_clearance_per_vehicle_m=1.,minimum_lateral_detour_m=.5,
            native_virtual_height_bounds_m=[1.2,2.6],actual_relative_height_upper_m=3.2,
            measured_peer_body_mask_radius_m=.75,settling_stable_seconds=2.,
            settling_speed_m_s=.15,settling_height_error_m=.15,
            observed_route_clearance_m=.6,minimum_simultaneous_motion_s=2.)
        (self.out/'criteria.json').write_text(json.dumps(self.report['gates'],indent=2))
        for n in self.nodes:n.report['gates']['max_horizontal_excursion_m']=9.
        self.swarm=[];self.px4_modes={}
        self.swarm_sub=self.nodes[0].create_subscription(MINCOTraj,'/ego_swarm/trajectory',self.minco,qos_profile_sensor_data)
        self.mode_subs=[self.nodes[0].create_subscription(OffboardControlMode,
            f'/drone{i}/fmu/in/offboard_control_mode',lambda m,k=i:self.px4_modes.update({k:m}),qos_profile_sensor_data)
            for i in (0,1)]

if __name__=='__main__':
    rclpy.init();mission=MixedCrossing()
    try:mission.run()
    except BaseException as exc:
        mission.report.update(passed=False,error=repr(exc))
        try:mission.recover()
        except BaseException as recovery:mission.report['recovery_error']=repr(recovery)
    finally:mission.finish();rclpy.try_shutdown()
    raise SystemExit(0 if mission.report['passed'] else 1)
