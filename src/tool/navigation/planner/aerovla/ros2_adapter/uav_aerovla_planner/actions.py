"""Validate model output before applying the upstream action interpretation."""
import re

from . import policy


def parse_action(text):
    """Return None for LAND, otherwise (forward, down, yaw); reject missing bins.

    Keep upstream last-three-bin/dequantization and near-zero semantics. Unlike
    the upstream default-zero fallback, incomplete output is an input error.
    Only inspect the action suffix, so a target description cannot request LAND.
    """
    if not isinstance(text, str) or not text.strip():
        raise ValueError('empty action output')
    output = text.split('Action:')[-1]
    if re.search(r'(?<!\w)LAND(?!\w)', output):
        return None
    if len(re.findall(r'\d+', output)) < 3:
        raise ValueError('action output requires three bins or LAND')
    action = policy.dequantize_bins(output)
    return None if policy.is_landing('', *action) else action
