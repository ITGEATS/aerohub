#!/usr/bin/env python3
"""Reuse the owned, disarmed RGB-D ground harness with a tracking consumer."""
from pathlib import Path
import sys
ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT))
from deployment.object_detection.airsim.run import main
if __name__ == '__main__':
    main(probe_script='deployment/target_tracking/airsim/probe.py',
         detector_backend='grounding_dino')
