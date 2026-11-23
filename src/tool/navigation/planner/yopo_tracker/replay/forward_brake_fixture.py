"""Clearance and braking calculations for the target-independent SITL fixture."""
import math
from threading import Event, Thread

import numpy as np
from scipy.spatial.transform import Rotation
from uav_as2_sensor_adapter.rpc import Rpc


FIXTURE_DISTANCE_M = 1.8  # Existing fixed trajectory.
CORRIDOR_HALF_WIDTH_M = .5  # Half width of the box used to check the simulated vehicle's route.
CORRIDOR_LENGTH_M = 2 * FIXTURE_DISTANCE_M  # Also covers the measured 2.60 m prior excursion.
CONTROL_PERIOD_NS = 20_000_000  # Existing 50 Hz trajectory sample interval.
SECOND = 1_000_000_000
HANDOFF_LEAD_NS = SECOND // 5  # Existing ordinary YOPO lead.


class NoBrakingPath(RuntimeError):
    pass


def check_front_clearance(rpc, direction):
    # The dedicated clearance camera sits ahead of the simulated propeller
    # envelope. The normal co-located camera can return the airframe itself.
    name = 'CorridorCamera'
    request = dict(camera_name=name, image_type=1, pixels_as_float=True, compress=False)
    (frame,) = rpc.call('simGetImages', [request], 'PX4', False)
    width, height = frame['width'], frame['height']
    depth = np.asarray(frame['image_data_float']).reshape(height, width)
    if not np.all(np.isfinite(depth) & (depth > 0.)):
        raise RuntimeError('Invalid forward corridor depth image')
    info = rpc.call('simGetCameraInfo', name, 'PX4', False)
    fx = width / (2 * math.tan(math.radians(info['fov']) / 2))
    v, u = np.indices(depth.shape)
    xyz_camera = np.stack((depth, (u - (width - 1) / 2) * depth / fx,
                           (v - (height - 1) / 2) * depth / fx), axis=-1)
    q = frame['camera_orientation']
    attitude = Rotation.from_quat([q['x_val'], q['y_val'], q['z_val'], q['w_val']])
    ned = attitude.apply(xyz_camera.reshape(-1, 3))
    vehicle = rpc.call('simGetVehiclePose', 'PX4')['position']
    camera = frame['camera_position']
    offset = [camera[key] - vehicle[key] for key in ('x_val', 'y_val', 'z_val')]
    ned += offset
    enu = ned[:, [1, 0, 2]].copy()
    enu[:, 2] *= -1
    longitudinal = enu @ direction
    lateral = enu @ np.array([-direction[1], direction[0], 0.])
    corridor = (longitudinal > 0.) & (np.abs(lateral) <= CORRIDOR_HALF_WIDTH_M) & (
        np.abs(enu[:, 2]) <= CORRIDOR_HALF_WIDTH_M)
    if not np.any(corridor):
        raise RuntimeError('No valid surface samples in the forward corridor')
    clearance = float(np.min(longitudinal[corridor]))
    return dict(measured_clearance_m=clearance, required_clearance_m=CORRIDOR_LENGTH_M,
                corridor_half_width_m=CORRIDOR_HALF_WIDTH_M, sample_count=int(np.count_nonzero(corridor)),
                depth_stamp_ns=frame['time_stamp'], camera_offset_ned_m=offset)


def forecast_stop(position, velocity, acceleration, lead, deceleration):
    future_speed = max(0., velocity + lead * acceleration)
    future_position = position + lead * (velocity + future_speed) / 2
    return future_speed, future_position + future_speed**2 / (2 * deceleration)


def select_measured_brake(position, velocity, acceleration, end, direction, nominal_duration_ns,
                          measured_position, measured_velocity, measured_acceleration, lead,
                          calibration):
    """Choose an admissible cubic-velocity brake near the measured stopping prediction."""
    along = float(velocity @ direction)
    acc = float(acceleration @ direction)
    predicted_speed, predicted_stop = forecast_stop(
        measured_position, measured_velocity, measured_acceleration, lead, calibration['deceleration_m_s2'])
    best = None
    for duration_ns in range(CONTROL_PERIOD_NS, nominal_duration_ns + 1, CONTROL_PERIOD_NS):
        duration = duration_ns / SECOND
        terminal = position + duration * velocity / 2 + duration**2 * acceleration / 12
        if (terminal - end) @ direction > 0. or along < 0. or 3 * along + acc * duration < 0.:
            continue
        # Analytic extrema of the acceleration quadratic and its linear derivative.
        linear = -6 * along / duration - 4 * acc
        quadratic = 6 * along / duration + 3 * acc
        extreme = min(1., max(0., -linear / (2 * quadratic))) if quadratic else 0.
        peak_acc = max(abs(acc), abs(acc + extreme * (linear + extreme * quadratic)))
        peak_jerk = max(abs(linear), abs(linear + 2 * quadratic)) / duration
        if (peak_acc > calibration['reference_acceleration_m_s2'] or
                peak_jerk > calibration['reference_jerk_m_s3']):
            continue
        score = abs(float(terminal @ direction) - predicted_stop)
        if best is None or score < best[0]:
            best = (score, duration_ns, terminal, peak_acc, peak_jerk)
    if best is None:
        raise NoBrakingPath('No continuous braking curve fits the measured test corridor')
    _, duration_ns, terminal, peak_acc, peak_jerk = best
    details = dict(predicted_handoff_speed_m_s=predicted_speed, predicted_stop_m=predicted_stop,
                   peak_reference_acceleration_m_s2=peak_acc, peak_reference_jerk_m_s3=peak_jerk)
    return duration_ns, terminal, details


class CollisionWatch:
    """Poll the simulator's persistent contact stamp during the motion case."""
    def __init__(self, host, port, initial_stamp):
        self.rpc = Rpc(host, port, 3)
        self.last_stamp = initial_stamp
        self.contacts = []
        self.error = None
        self.stop = Event()
        self.thread = Thread(target=self.poll, daemon=True)
        self.thread.start()

    def poll(self):
        try:
            while not self.stop.is_set():
                contact = self.rpc.call('simGetCollisionInfo', 'PX4')
                if contact['time_stamp'] != self.last_stamp:
                    self.contacts.append(contact)
                    self.last_stamp = contact['time_stamp']
                self.stop.wait(1 / 15)
        except Exception as error:
            self.error = str(error)
        finally:
            self.rpc.close()

    def close(self):
        self.stop.set()
        self.thread.join(3.2)
