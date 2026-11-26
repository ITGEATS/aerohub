"""Acknowledged planning timeline, derived from the YOPO reference planner (MIT)."""
import math


class Timeline:
    """A rejected or unacknowledged update never changes accepted references."""
    def __init__(self):
        self.plans = {}
        self.pending = None
        self.revision = 0
        self.effective = 0

    @property
    def latest(self):
        return self.plans.get(self.revision)

    def propose(self, curve):
        if self.pending is not None:
            raise ValueError("execution request already pending")
        revision = self.revision + 1
        self.pending = revision, curve
        return revision

    def acknowledge(self, accepted):
        if self.pending is None:
            raise ValueError("no pending submission")
        revision, curve = self.pending
        self.pending = None
        if accepted:
            self.plans[revision] = curve
            self.revision = revision
        return revision

    def feedback(self, revision, elapsed, now, waiting):
        curve = self.plans.get(revision)
        if curve is None or revision < self.effective:
            return False
        # A feedback sample can arrive after transport delay, never before its epoch.
        epoch = curve.start + elapsed
        if not math.isfinite(elapsed) or elapsed < 0 or epoch > now + .02 or now-epoch > .5:
            return False
        if not waiting and elapsed > .12 and now > curve.start + .12:
            self.effective = revision
            self.plans = {r: p for r, p in self.plans.items() if r >= revision}
        return True

    def reference(self, epoch):
        if self.latest is None or not self.latest.start < epoch < self.latest.end:
            raise ValueError("handoff outside accepted coverage")
        return self.latest.execution_sample(epoch)
