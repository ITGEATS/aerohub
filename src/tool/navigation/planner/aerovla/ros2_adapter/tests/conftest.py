import sys
from pathlib import Path

# Host pytest without colcon: make the package importable from the source tree.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
