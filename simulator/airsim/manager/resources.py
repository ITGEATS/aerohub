"""Compatibility import for the original AirSim resource module."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[3]))
from simulator.manager.resources import resolve_resource

__all__ = ["resolve_resource"]
