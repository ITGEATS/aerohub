import copy
import os
import json
from pathlib import Path
import sys
import unittest
from hybrid_plan import resolve

class PlanTests(unittest.TestCase):
    def setUp(self):
        self.config=json.loads(Path('/session/swarm-config.json').read_text())
        self.session_name=json.loads(Path('/session/config.json').read_text())['run_id']
        self.snapshot=json.loads(Path('/snapshot.json').read_text())
    def test_owns_one_board_and_preserves_actual_spawn_and_ids(self):
        result=resolve(self.config,self.snapshot,'/input')
        self.assertEqual(result['topology']['board_vehicle']['name'],'drone0')
        self.assertEqual([v['name'] for v in result['topology']['pc_vehicles']],['drone1'])
        self.assertEqual(result['ego']['deployment']['drone1']['px4_origin_in_fleet_ned'],[0,-4,0])
        self.assertEqual(result['settings']['Vehicles'],self.snapshot['Vehicles'])
        self.assertEqual(result['settings']['ExternalCameras']['swarm_overhead']['Pitch'],-90.)
    def test_rejects_id_port_and_vehicle_mismatch(self):
        for key in ('VehicleSysID','TcpPort','ControlPortLocal','ControlPortRemote'):
            invalid=copy.deepcopy(self.snapshot);invalid['Vehicles']['Drone1'][key]+=1
            with self.assertRaises(ValueError):resolve(self.config,invalid,'/input')
        invalid=copy.deepcopy(self.config);invalid['fleet']['board_vehicle_index']=1
        with self.assertRaises(ValueError):resolve(invalid,self.snapshot,'/input')
    def test_single_runtime_names_and_prefix_contract(self):
        sys.path.insert(0,'/single')
        from interfaces import topic_map
        self.assertEqual(topic_map({'vehicle':'drone1'})['/ego/status'],'/drone1/ego/status')

    def test_real_ros_runtime_constructs_for_hyphenated_session(self):
        import tempfile
        os.environ['EGO_SESSION_ROOT']='/session/drone1'
        sys.path.insert(0,'/single')
        import rclpy
        from runtime import Runtime
        rclpy.init()
        try:
            with tempfile.TemporaryDirectory() as tmp:
                node=Runtime(Path(tmp))
                try:
                    self.assertEqual(node.get_namespace(),'/drone1')
                    self.assertEqual(node.probe_pub.topic_name,'/acceptance/run_'+self.session_name.replace('-', '_')+'/drone1/minco')
                finally:node.destroy_node()
        finally:rclpy.shutdown()

if __name__=='__main__':unittest.main()
