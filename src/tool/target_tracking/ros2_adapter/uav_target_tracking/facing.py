"""Turn-only tracking intent and lifecycle over the public Navigation Tool API."""
import math
import time


class FacingError(RuntimeError):
    def __init__(self, code, reason):
        super().__init__(reason)
        self.code = code


def facing_yaw(own, target):
    if not all(math.isfinite(x) for x in (*own, *target)):
        raise FacingError('INVALID_TARGET', 'non-finite own or target position')
    dx, dy = target[0]-own[0], target[1]-own[1]
    if math.hypot(dx, dy) < .25:
        raise FacingError('INVALID_TARGET', 'target too close horizontally to define facing')
    return math.atan2(dy, dx)


class FacingSession:
    """One owned HOLD Navigation task, with serial yaw updates and bounded cleanup.

    The transport constructs ROS Tool messages. No Behavior or setpoint access.
    Initial xyz is frozen; follow never requests translation toward the target.
    """
    def __init__(self, transport, response_timeout=1., cleanup_timeout=10., clock=time.monotonic):
        self.transport = transport
        self.response_timeout = response_timeout
        self.cleanup_timeout = cleanup_timeout
        self.clock = clock
        self.submission = self.handle = self.result = self.update = None
        self.sent = self.updated = 0.
        self.revision = 1
        self.point = self.frame = None

    @property
    def started(self):
        return self.submission is not None

    def step(self, own, frame, yaw, remaining):
        wall = self.clock()
        if self.submission is None:
            if not self.transport.ready():
                raise FacingError('NAVIGATION_NOT_READY', 'Navigation Tool unavailable for follow=true')
            self.point, self.frame = tuple(own), frame
            self.submission = self.transport.start(self.point, frame, yaw, remaining, 1.)
            self.sent = self.updated = wall
            return
        if frame != self.frame:
            raise FacingError('FRAME_CHANGED', 'facing frame changed during session')
        if self.handle is None:
            if not self.submission.done():
                if wall-self.sent > self.response_timeout:
                    raise FacingError('NAVIGATION_TIMEOUT', 'navigation submission outcome unknown')
                return
            self.handle = self.submission.result()
            if not self.handle.accepted:
                raise FacingError('NAVIGATION_REJECTED', 'Navigation Tool rejected turn-only task')
            self.result = self.handle.get_result_async()
        if self.result.done():
            result = self.result.result().result
            raise FacingError('NAVIGATION_'+result.code, result.reason or 'navigation ended before tracking')
        if self.update is not None:
            if not self.update.done():
                if wall-self.sent > self.response_timeout:
                    raise FacingError('NAVIGATION_TIMEOUT', 'navigation update outcome unknown')
                return
            response = self.update.result()
            if not response.accepted or response.target_revision != self.revision:
                raise FacingError('NAVIGATION_UPDATE_FAILED', response.reason or 'target revision mismatch')
            self.update = None
        if wall-self.updated >= .2:
            self.revision += 1
            self.update = self.transport.update(self.handle, self.revision, self.point, frame, yaw)
            self.sent = self.updated = wall

    def _wait(self, future, deadline):
        while not future.done():
            if self.clock() >= deadline:
                raise TimeoutError('navigation stop deadline exceeded')
            time.sleep(.01)
        return future.result()

    def close(self):
        if self.submission is None:
            return True
        deadline = self.clock()+self.cleanup_timeout
        try:
            if self.handle is None:
                try:
                    self.handle = self._wait(self.submission, deadline)
                except TimeoutError:
                    def cancel_late(future):
                        handle = future.result()
                        if handle.accepted:
                            handle.cancel_goal_async()
                    self.submission.add_done_callback(cancel_late)
                    return False
            if not self.handle.accepted:
                return True
            if self.result is None:
                self.result = self.handle.get_result_async()
            if not self.result.done():
                self._wait(self.handle.cancel_goal_async(), deadline)
            wrapped = self._wait(self.result, deadline)
            return bool(wrapped.result.stop_confirmed)
        except Exception:
            return False
