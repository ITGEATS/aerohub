"""Bounded Mission recovery; never turns expired Tracking output into a valid pose."""
import math


class ObservationRecovery:
    def __init__(self, frame, generation, initial_observation, grace_seconds=2.):
        if not math.isfinite(grace_seconds) or not 0<grace_seconds<=5.:
            raise ValueError('Observation grace must be within (0,5] seconds')
        self.frame=frame
        self.generation=generation
        self.last_observation=initial_observation
        self.grace_seconds=grace_seconds
        self.started=None
        self.last_ros=None
        self.last_wall=None

    def update(self, state, now, wall):
        stamp=lambda t:t.sec+t.nanosec/1e9
        observed=stamp(state.observation_stamp)
        if (not all(math.isfinite(x)for x in (now,wall,observed)) or
            (self.last_ros is not None and now<self.last_ros) or
            (self.last_wall is not None and wall<self.last_wall)):
            raise ValueError('Observation clock changed')
        self.last_ros,self.last_wall=now,wall
        if state.header.frame_id!=self.frame or state.generation!=self.generation:
            raise ValueError('Observation frame or generation changed')
        if observed<=0 or observed>now:
            raise ValueError('Observation timestamp invalid')
        if self.started is not None and wall-self.started>=self.grace_seconds:
            raise TimeoutError(f'No fresh observation within {self.grace_seconds:g}s Mission recovery')
        available=(state.valid and state.status in ('tracked','predicted') and
                   now<=stamp(state.valid_until))
        if available:
            if not all(math.isfinite(x)for x in (state.position.x,state.position.y,state.position.z,
                                                 state.velocity.x,state.velocity.y,state.velocity.z)):
                raise ValueError('Observation geometry invalid')
            fresh_recovery=(observed>self.last_observation and now-observed<=.4)
            if self.started is None or fresh_recovery:
                elapsed=wall-self.started if self.started is not None else 0.
                self.started=None
                self.last_observation=max(self.last_observation,observed)
                return False,elapsed
        if self.started is None:self.started=wall
        elapsed=wall-self.started
        if elapsed>=self.grace_seconds:
            raise TimeoutError(f'No fresh observation within {self.grace_seconds:g}s Mission recovery')
        return True,elapsed
