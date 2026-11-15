import json
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

import run


class Clock:
    def __init__(self):
        self.value = 100.0

    def monotonic(self):
        return self.value

    def sleep(self, duration):
        self.value += duration


class PreflightTests(unittest.TestCase):
    def test_resolves_packaged_scene_wrapper_to_managed_elf(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            wrapper = root / "AirSimNH.sh"
            executable = root / "AirSimNH/Binaries/Linux/AirSimNH"
            executable.parent.mkdir(parents=True)
            wrapper.touch()
            executable.touch(mode=0o755)
            self.assertEqual(run.resolve_airsim_binary(wrapper), executable.resolve())

    def test_requires_stable_target_firmware_status(self):
        clock = Clock()
        statuses = iter([
            "pre_flight_checks_pass: True\n",
            "pre_flight_checks_pass: False\n",
            *(["pre_flight_checks_pass: True\n"] * 9),
        ])

        def output(command, **_kwargs):
            if command[1] == "logs":
                return "Ready for takeoff!\n"
            return next(statuses)

        with tempfile.TemporaryDirectory() as directory, patch.object(
                run.subprocess, "check_output", side_effect=output), patch.object(
                run.time, "monotonic", side_effect=clock.monotonic), patch.object(
                run.time, "sleep", side_effect=clock.sleep):
            evidence = Path(directory)
            run.wait_px4_preflight("vehicle", evidence)
            record = json.loads((evidence / "preflight.json").read_text())

        self.assertTrue(record["ready"])
        self.assertGreaterEqual(record["stable_seconds"], 2.0)
        self.assertEqual(record["samples"], 11)
        self.assertEqual(record["source"], "target firmware uORB vehicle_status")


if __name__ == "__main__":
    unittest.main()
