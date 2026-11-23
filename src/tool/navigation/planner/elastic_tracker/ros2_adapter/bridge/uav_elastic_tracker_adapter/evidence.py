"""Task-scoped evidence accounting; acceptance must come from the framework."""
from pathlib import PurePosixPath
import re


def uuid_values(value):
    """ROS UUID may expose numpy.uint8; audit JSON requires Python integers."""
    return [int(item) for item in value]


def goal_rejection_from_log(log, task_id):
    active = None
    observed_task = False
    for line in log.splitlines():
        response = re.search(r'ACTION_GOAL_RESPONSE task_id=(\S+) accepted=(true|false)', line)
        if response and response.group(1) == task_id and response.group(2) == 'false':
            return True
        match = re.search(r'resolved_config:=([^\s]+\.yaml)', line)
        if 'Starting runtime:' in line and match:
            active = PurePosixPath(match.group(1).replace('\\', '/')).stem
            observed_task |= active == task_id
        if active == task_id and 'FOLLOW_PATH_REJECTED:' in line:
            return True
    # Absence of a task's log is unknown, not a negative observation.
    return False if observed_task else None


def message_key(message):
    stamp = lambda value: value.sec*1_000_000_000+value.nanosec
    return (message.task_id, stamp(message.generated_at), stamp(message.valid_from))


class TrajectoryEvidence:
    def __init__(self, task_id, start_ns, end_ns=None):
        self.task_id, self.start_ns, self.end_ns = task_id, start_ns, end_ns
        self.raw = {}
        self.accepted = {}
        self.seen_revisions = set()
        self.raw_received = 0

    def _window(self, generated, valid, accepted_at):
        return (self.start_ns <= generated <= valid and self.start_ns <= accepted_at
                and (self.end_ns is None or max(generated, accepted_at) <= self.end_ns))

    def receive(self, message):
        self.raw_received += 1
        key = message_key(message)
        if key[0] == self.task_id:
            self.raw[key] = message

    def accept(self, event):
        if event['task_id'] != self.task_id or event.get('kind') != 'framework_accepted':
            return
        revision = event['revision']
        if revision <= 0 or revision in self.seen_revisions:
            return
        if self.seen_revisions and revision <= max(self.seen_revisions):
            return
        generated, valid = event['generated_ns'], event['valid_from_ns']
        if not self._window(generated, valid, event['accepted_at_ns']):
            return
        key = (self.task_id, generated, valid)
        self.accepted[key] = event
        self.seen_revisions.add(revision)

    @property
    def samples(self):
        return [(self.raw[key], event) for key, event in self.accepted.items() if key in self.raw]
