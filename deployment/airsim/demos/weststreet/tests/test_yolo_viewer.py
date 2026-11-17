import importlib.util
import json
from pathlib import Path
import tempfile
import unittest

spec = importlib.util.spec_from_file_location('viewer', Path(__file__).resolve().parents[1]/'yolo_viewer.py')
viewer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(viewer)

class SameFrameTest(unittest.TestCase):
    def test_timestamp_identity_stride_and_no_source_mutation(self):
        with tempfile.TemporaryDirectory() as folder:
            folder = Path(folder)
            (folder/'123.json').write_text(json.dumps(dict(width=2,height=1,step=8,encoding='rgb8')))
            (folder/'123.rgb').write_bytes(bytes([255,0,0,0,255,0,0,0]))
            image = viewer.native_frame(folder,dict(stamp_ns=123,detections=[dict(xyxy=[0,0,2,1])]))
            self.assertEqual(image['data'],bytes([0,0,255,0,255,0]))
            self.assertEqual(image['annotations'][0]['bbox2d']['center'],dict(x=1,y=.5))
            self.assertEqual((folder/'123.rgb').read_bytes(),bytes([255,0,0,0,255,0,0,0]))
            with self.assertRaises(FileNotFoundError):
                viewer.native_frame(folder,dict(stamp_ns=124))

if __name__ == '__main__':
    unittest.main()
