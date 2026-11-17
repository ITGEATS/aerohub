"""System Tool admission policy, independent of ROS transport."""
import math


def validate(command, height, speed, timeout):
    if command not in (1, 2):
        return 'command must be TAKEOFF or LAND'
    if not all(math.isfinite(x) for x in (height, speed, timeout)):
        return 'parameters must be finite'
    if not .1 <= speed <= 2. or not 5. <= timeout <= 300.:
        return 'speed must be 0.1..2m/s and timeout 5..300s'
    if command == 1 and not .5 <= height <= 20.:
        return 'takeoff height must be 0.5..20m'
    return ''
