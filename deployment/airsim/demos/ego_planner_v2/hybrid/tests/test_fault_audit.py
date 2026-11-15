"""Frozen real mixed cancellation and evidence mutations; no live ROS or flight."""
import json
from pathlib import Path
import tempfile
import unittest
import audit_fault

class FaultAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.session=Path(self.temp.name)
        for source in Path('/frozen').rglob('*'):
            if 'source-snapshot' in source.parts:continue
            target=self.session/source.relative_to('/frozen')
            if source.is_dir():target.mkdir(exist_ok=True)
            elif source.name!='stage-audit.json' or source.parent.name=='mission-basic':target.symlink_to(source)
        self.root=self.session/'mission-fault';self.report=json.loads((self.root/'result.json').read_text())
        self.i=int(self.report['fault_vehicle'][-1]);self.peer=1-self.i
    def tearDown(self):self.temp.cleanup()
    def replace(self,path,data,lines=False):
        target=self.root/path;target.unlink();target.write_text('\n'.join(json.dumps(r) for r in data)+'\n' if lines else json.dumps(data))
    def rows(self,i):return [json.loads(l) for l in (self.root/f'drone{i}/flight.jsonl').read_text().splitlines()]
    def test_real_frozen_case_passes(self):self.assertTrue(audit_fault.audit(self.root)['passed'])
    def test_early_peer_injection_rejected(self):
        rows=self.rows(self.peer)
        pre=[r for r in rows if r.get('topic')=='odom' and r['phase']=='ego' and r['wall']<self.report['injection_wall']]
        summary=json.loads((self.root/f'drone{self.peer}/result.json').read_text())
        pre[-1]['position']=summary['ego_start_position']
        self.replace(f'drone{self.peer}/flight.jsonl',rows,True)
        self.assertFalse(audit_fault.audit(self.root)['checks']['peer_moving_at_injection'])
    def test_early_owner_injection_rejected(self):
        rows=self.rows(self.i)
        pre=[r for r in rows if r.get('topic')=='odom' and r['phase']=='ego' and r['wall']<self.report['injection_wall']]
        summary=json.loads((self.root/f'drone{self.i}/result.json').read_text())
        pre[-1]['position']=summary['ego_start_position']
        self.replace(f'drone{self.i}/flight.jsonl',rows,True)
        self.assertFalse(audit_fault.audit(self.root)['checks']['moving_at_injection'])
    def test_peer_failure_rejected(self):
        rows=self.rows(self.peer)
        row=next(r for r in rows if 'ego_state' in r and r['wall']>=self.report['injection_wall'])
        row['ego_state']['state']='FAILED';self.replace(f'drone{self.peer}/flight.jsonl',rows,True)
        self.assertFalse(audit_fault.audit(self.root)['checks']['peer_never_failed'])
    def test_old_task_resumption_rejected(self):
        rows=self.rows(self.i);states=[r for r in rows if 'ego_state' in r and r['wall']>=self.report['injection_wall']]
        states[-1]['ego_state']['state']='RUNNING';self.replace(f'drone{self.i}/flight.jsonl',rows,True)
        self.assertFalse(audit_fault.audit(self.root)['checks']['closed_without_resume'])
    def test_missing_execution_segments_rejected(self):
        self.replace(f'drone{self.peer}/segments.json',[])
        self.assertFalse(audit_fault.audit(self.root)['checks'][f'drone{self.peer}_native_pva'])
    def test_wrong_owner_gate_rejected(self):
        v=self.report['fault_vehicle'];self.report['input_graph'][f'/acceptance/{v}/odom']['publishers'][0]['namespace']='/other'
        self.replace('result.json',self.report)
        self.assertFalse(audit_fault.audit(self.root)['checks']['scoped_gate_graph'])
    def test_missing_live_navigation_interval_rejected(self):
        rows=self.rows(self.i);start=self.report['injection_wall']+.2
        rows=[r for r in rows if not (r.get('topic')=='odom' and start<=r['wall']<start+1.2)]
        self.replace(f'drone{self.i}/flight.jsonl',rows,True)
        self.assertFalse(audit_fault.audit(self.root)['checks'][f'drone{self.i}_live_navigation'])

if __name__=='__main__':unittest.main()
