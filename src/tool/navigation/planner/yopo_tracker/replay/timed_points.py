"""Access the shared timed trajectory encoder from the replay entry points."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from ros2_adapter.bridge.timed_points import timed_goal, timed_points  # noqa: F401
