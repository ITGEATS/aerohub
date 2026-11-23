"""ROS ENU/FLU observations to the upstream AeroVLA NED/FRD convention."""

import math

import numpy as np
from scipy.spatial.transform import Rotation

_WORLD_ENU_TO_NED = np.array([[0., 1., 0.], [1., 0., 0.], [0., 0., -1.]])
_BODY_FRD_TO_FLU = np.diag([1., -1., -1.])


def enu_ned_vector(vector):
    """Convert a world vector; this map is its own inverse."""
    return [float(vector[1]), float(vector[0]), -float(vector[2])]


def enu_to_ned_quat(orientation):
    """Convert ENU<-FLU attitude to NED<-FRD, as an xyzw quaternion.

    The left transform changes the world frame; the right transform changes
    the body frame. Using the world transform on both sides swaps the body
    forward axis and incorrectly maps an east-facing vehicle to north.
    """
    if isinstance(orientation, dict):
        quat = [orientation[key] for key in ('x', 'y', 'z', 'w')]
    else:
        quat = list(orientation)
    matrix = Rotation.from_quat(quat).as_matrix()
    return Rotation.from_matrix(
        _WORLD_ENU_TO_NED @ matrix @ _BODY_FRD_TO_FLU
    ).as_quat().tolist()


def enu_to_ned_yaw(yaw):
    """Convert planar heading with the same convention as the framework."""
    return (math.pi / 2 - yaw + math.pi) % (2 * math.pi) - math.pi
