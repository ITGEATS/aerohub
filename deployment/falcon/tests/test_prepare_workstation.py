import os
import tempfile
import unittest
from pathlib import Path

from prepare_workstation import INTERFACES, prepare


class WorkstationInterfaceTests(unittest.TestCase):
    def test_copies_locked_interface_files_and_records_hashes(self):
        configured = os.environ.get("AS2_MSGS_SOURCE")
        source = (Path(configured) if configured else
                  Path(__file__).resolve().parents[3] / "as2/aerostack2/as2_msgs")
        self.assertTrue(source.is_dir(), f"AS2 message source is missing: {source}")
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "workspace"
            manifest = prepare(source, output)
            for relative in INTERFACES:
                self.assertEqual(
                    (output / "src/as2_msgs" / relative).read_bytes(),
                    (source / relative).read_bytes())
                self.assertEqual(len(manifest["interfaces"][relative]), 64)


if __name__ == "__main__":
    unittest.main()
