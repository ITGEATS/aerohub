"""Frozen real flight plus invalid-hold mutations; networking is unnecessary."""
import copy
import json
from pathlib import Path
import tempfile
import unittest
from audit_flight import audit

class HoldEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.root=Path(self.tmp.name)
        for source in Path('/frozen').rglob('*'):
            target=self.root/source.relative_to('/frozen')
            if source.is_dir():target.mkdir()
            elif source.name!='stage-audit.json':target.symlink_to(source)
        self.file=self.root/'drone1/segments.json'
        self.segments=json.loads(self.file.read_text())
    def tearDown(self):self.tmp.cleanup()
    def changed(self,change):
        change(self.segments);self.file.unlink();self.file.write_text(json.dumps(self.segments))
        self.assertFalse(audit(self.root)['checks']['drone1_native_pva'])
    def test_real_emitted_hold_passes(self):self.assertTrue(audit(self.root)['passed'])
    def test_no_segment_evidence_rejected(self):self.changed(lambda s:s.clear())
    def test_non_final_rejected(self):
        self.changed(lambda s:[row.update(is_final=False) for row in s])
    def test_hold_over_100ms_rejected(self):
        self.changed(lambda s:[v['points'][-1].update(t_ns=v['points'][-1]['t_ns']+100000001) for row in s if row['is_final'] for v in row['vehicles']])
    def test_wrong_terminal_position_rejected(self):
        self.changed(lambda s:[v['points'][-1].update(p=[99.,99.,99.]) for row in s if row['is_final'] for v in row['vehicles']])
    def test_nonzero_terminal_velocity_rejected(self):
        self.changed(lambda s:[v['points'][-1].update(v=[.01,0.,0.]) for row in s if row['is_final'] for v in row['vehicles']])
    def test_missing_native_endpoint_knot_rejected(self):
        self.changed(lambda s:[v.update(points=[v['points'][0],v['points'][-1]]) for row in s if row['is_final'] for v in row['vehicles']])

if __name__=='__main__':unittest.main()
