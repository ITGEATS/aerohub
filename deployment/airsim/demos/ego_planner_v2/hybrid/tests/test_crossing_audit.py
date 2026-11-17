"""Frozen encounter: JSON export, conflict geometry, native parameters, coverage."""
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import audit_crossing

class CrossingAuditTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();self.root=Path(self.temp.name)
        for source in Path('/frozen').rglob('*'):
            target=self.root/source.relative_to('/frozen')
            if source.is_dir():target.mkdir()
            elif source.name not in ('stage-audit.json','crossing-tracks.png'):target.symlink_to(source)
        common=json.loads((self.root/'common-audit-before-serialization-fix.json').read_text())
        self.mock=patch.object(audit_crossing,'common_audit',return_value=common);self.mock.start()
    def tearDown(self):self.mock.stop();self.temp.cleanup()
    def replace(self,name,value):
        p=self.root/name;p.unlink();p.write_text(json.dumps(value))
    def test_frozen_crossing_exports_valid_json(self):
        result=audit_crossing.audit(self.root)
        self.assertTrue(result['passed'])
        self.assertEqual(json.loads((self.root/'stage-audit.json').read_text()),result)
    def test_separated_nominal_routes_rejected(self):
        report=json.loads((self.root/'result.json').read_text())
        report['routes']['goals'][0]['world'][0]+=3
        self.replace('result.json',report)
        self.assertFalse(audit_crossing.audit(self.root)['checks']['nominal_collision_course'])
    def test_wrong_native_clearance_rejected(self):
        report=json.loads((self.root/'result.json').read_text());report['native_parameters']['0'][0]=.15
        self.replace('result.json',report)
        self.assertFalse(audit_crossing.audit(self.root)['checks']['drone0_native_parameters'])
    def test_missing_encounter_samples_rejected(self):
        p=self.root/'drone0/flight.jsonl';rows=[json.loads(l) for l in p.read_text().splitlines()]
        times=[r['sim'] for r in rows if r.get('topic')=='odom' and r['phase']=='ego'];center=(min(times)+max(times))/2
        kept=[r for r in rows if not (r.get('topic')=='odom' and center<=r['sim']<center+.3)]
        p.unlink();p.write_text('\n'.join(json.dumps(r) for r in kept)+'\n')
        self.assertFalse(audit_crossing.audit(self.root)['checks']['odom_encounter_complete_coverage'])

if __name__=='__main__':unittest.main()
