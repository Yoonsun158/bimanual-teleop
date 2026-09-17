"""Hand following and lifecycle tests; no SDK manager or hardware connections."""


from bimanual_teleop.devices.wuji.adapter import JOINT_NAMES
from bimanual_teleop.types import HandSkeleton, Health, JointState, JointTarget, Sample, SampleHeader, SampleRef, Submission


class Sink:
    def __init__(self): self.events, self.samples = [], []
    def try_event(self, event): self.events.append(event); return True
    def try_publish(self, sample): self.samples.append(sample); return True


class Device:
    def __init__(self, side, clock, *, glove=False):
        self.side, self.clock, self.glove = side, clock, glove
        self.device_id = f"wuji_{side}_{'glove' if glove else 'hand'}"
        self.metadata, self.statistics = {}, {}
        self.fault, self.latest, self.sink = None, None, None
        self.sequence, self.q = 0, .1
        self.ready, self.closed = True, False
        self.enabled, self.profile, self.last_target = False, None, None
        self.commands, self.calls = [], []
        self.accept, self.engage_hook, self.disable_hook, self.close_error = True, None, None, None

    def start(self, sink=None): self.sink = sink; self.calls.append("start"); self.emit()

    def emit(self, *, valid=True, q=None):
        if q is not None: self.q = q
        self.sequence += 1
        payload = HandSkeleton(f"{self.side}_wrist", tuple(f"p{i}" for i in range(21)),
            ((self.q, 0., 0.),)*21, (1.,)*21, ()) if self.glove else JointState(JOINT_NAMES, (self.q,)*20)
        self.latest = Sample(SampleHeader(SampleRef(self.device_id, "test", self.sequence),
                                         self.clock(), valid), payload)
        if not valid: self.fault = "invalid sample"
        if self.sink: self.sink.try_publish(self.latest)
        return self.latest

    def get_latest(self): return self.latest

    def health(self, *, check_latch=False):
        ready = self.ready and self.latest is not None and self.latest.header.valid
        if check_latch and self.fault: ready = False
        return Health(ready, self.clock(), None if ready else self.fault or "unavailable")

    def clear_fault(self):
        if not self.health().ready: raise RuntimeError("not currently healthy")
        self.fault = None

    def configure(self, profile): self.profile = profile; self.calls.append("configure")

    def engage(self, *, cancelled=None):
        self.calls.append("engage")
        if self.engage_hook: self.engage_hook()
        if cancelled and cancelled(): raise RuntimeError("cancelled")
        self.last_target = JointTarget(JOINT_NAMES, self.latest.payload.position_rad)
        self.enabled = True

    def submit(self, command):
        if not self.accept: return Submission(command.command_id, False, f"{self.side} send failed")
        if self.clock() >= command.expires_monotonic_ns:
            return Submission(command.command_id, False, "expired")
        self.commands.append(command)
        self.last_target = command.payload
        return Submission(command.command_id, True)

    def request_hold(self, reason): self.calls.append("hold")

    def disable(self, reason="requested"):
        self.calls.append("disable")
        self.enabled = False
        if self.disable_hook: self.disable_hook()

    def close(self):
        self.calls.append("close")
        self.enabled, self.closed = False, True
        if self.close_error: raise self.close_error


class Mapper:
    def __init__(self): self.calls, self.resets = 0, 0; self.fail = False; self.hook = None
    def reset(self): self.resets += 1
    def solve(self, sample):
        self.calls += 1
        if self.hook: self.hook()
        if self.fail: raise ValueError("invalid retarget output")
        return JointTarget(JOINT_NAMES, (sample.payload.positions_m[0][0],)*20)
