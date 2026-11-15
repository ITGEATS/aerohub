"""Offline configuration and generated ownership checks; no device operations."""
import importlib.util
import json
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

HERE=Path(__file__).resolve().parents[1]
spec=importlib.util.spec_from_file_location('hybrid_entry_test',HERE/'run.py')
entry=importlib.util.module_from_spec(spec);spec.loader.exec_module(entry)
from hybrid_plan import resolve


class EntryTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.root=Path(self.temp.name);self.work=self.root/'work';self.work.mkdir()
        (self.work/'component').write_text('ego-planner-v2\n')
        manifest={p.relative_to(entry.PROJECT).as_posix():entry.hashlib.sha256(p.read_bytes()).hexdigest()
                  for rel in ['src/planner/ego_planner_v2/ros2_adapter/native',
                              'src/planner/ego_planner_v2/ros2_adapter/bridge',
                              'src/planner/interfaces','src/planner/framework','src/behavior/follow_path']
                  for p in (entry.PROJECT/rel).rglob('*') if p.is_file() and '__pycache__' not in p.parts}
        (self.work/'source-manifest.json').write_text(json.dumps(manifest))
        self.config=json.loads((HERE/'config.example.json').read_text())
        self.config['pc_workspace']=str(self.work)

    def tearDown(self):self.temp.cleanup()

    def test_explicit_workspace_and_no_implicit_clock_sync(self):
        value=entry.normalize_config(self.config)
        self.assertEqual(value['pc_workspace'],str(self.work.resolve()))
        self.assertEqual(value['aerohub_root'],str(entry.PROJECT))
        self.assertFalse(value['allow_clock_sync'])

    def test_mixed_entry_refuses_legacy_global_scope(self):
        for vehicle in ('', None, 'drone1'):
            with self.subTest(vehicle=vehicle),self.assertRaisesRegex(ValueError,'per-vehicle'):
                entry.normalize_config(dict(self.config,vehicle=vehicle))

    def test_rejects_unbuilt_or_stale_workspace(self):
        (self.work/'source-manifest.json').write_text('{}')
        with self.assertRaisesRegex(ValueError,'Rebuild'):entry.normalize_config(self.config)

    def test_rejects_other_component_workspace(self):
        (self.work/'component').write_text('another-component')
        with self.assertRaisesRegex(ValueError,'another component'):entry.normalize_config(self.config)

    def test_participant_cannot_overlap_compute_pool(self):
        for value in (6,16,17,32,True):
            with self.subTest(value=value),self.assertRaises(ValueError):
                entry.normalize_config(dict(self.config,mission_participant=value))

    def test_fault_scope_and_stage_are_explicit(self):
        for extra in [{'mission_stage':'unknown'},{'mission_stage':'fault'},
                      {'fault_case':'cloud'},{'mission_stage':'fault','fault_case':'cloud','fault_vehicle':'drone2'}]:
            with self.subTest(extra=extra),self.assertRaises(ValueError):entry.normalize_config(dict(self.config,**extra))

    def test_prepare_preserves_actual_topology_and_single_fault_owner(self):
        def local_plan(command,**kwargs):
            # Same shared resolver as the offline container, without requiring Docker-in-Docker.
            schema=json.loads((out/'swarm-config.json').read_text())
            plan=resolve(schema,json.loads((entry.DEMO/'pc/settings.snapshot.json').read_text()),entry.PROJECT)
            (out/'plan.json').write_text(json.dumps(plan))
            return ''
        for owner in ('drone0','drone1'):
            out=self.root/('case-'+owner)
            with patch.object(entry.single,'execute',side_effect=local_plan) as execute:
                entry.prepare(out,dict(self.config,mission_stage='fault',fault_case='cloud',fault_vehicle=owner))
                self.assertEqual(execute.call_count,1)
            plan=json.loads((out/'plan.json').read_text())
            self.assertEqual(plan['topology']['board_vehicle']['name'],'drone0')
            self.assertEqual([x['name'] for x in plan['topology']['pc_vehicles']],['drone1'])
            for vehicle,directory in [('drone0','bundle'),('drone1','drone1')]:
                cfg=json.loads((out/directory/'config.json').read_text())
                self.assertEqual(cfg['vehicle'],vehicle)
                self.assertEqual('fault_case' in cfg,vehicle==owner)
                self.assertEqual((out/directory/'fault.launch.py').exists(),vehicle==owner)
            command=entry.compute_command(out,json.loads((out/'config.json').read_text()))
            self.assertIn(str(self.work.resolve())+':/work:ro',command)
            self.assertIn(str(entry.SINGLE)+':/single:ro',command)

    def test_real_fault_launch_has_one_unwrapped_injector(self):
        # Build real launch actions but never execute them or start ROS nodes.
        from launch import LaunchContext
        from launch.actions import ExecuteProcess
        from launch_ros.actions import Node
        context=LaunchContext()
        module_spec=importlib.util.spec_from_file_location('fault_launch_test',HERE/'fault_launch.py')
        module=importlib.util.module_from_spec(module_spec);module_spec.loader.exec_module(module)
        with patch.object(module,'ROOT',self.root):
            cfg=dict(self.config,fault_vehicle='drone1',vehicle='drone1',
                     ego_launch_file=str(entry.PROJECT/'src/planner/ego_planner_v2/ros2_adapter/bridge/launch/ego_as2.launch.py'))
            (self.root/'config.json').write_text(json.dumps(cfg))
            context.launch_configurations.update(vehicle='drone1',resolved_config=str(entry.DEMO/'pc/ego_dual.yaml'),use_sim_time='true')
            actions=module.nodes(context)
            injectors=[a for a in actions if isinstance(a,ExecuteProcess) and not isinstance(a,Node)]
            self.assertEqual(len(injectors),1)
            from launch.utilities import perform_substitutions
            args=[perform_substitutions(context,x) for x in injectors[0].cmd]
            self.assertEqual(args,['python3',str(self.root/'fault_injector.py')])


if __name__=='__main__':unittest.main()
