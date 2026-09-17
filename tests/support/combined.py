"""Observable arm/hand lifecycle fixture for coordination and cancellation."""

from bimanual_teleop.system import SystemState
from bimanual_teleop.types import Health

class Runtime:
    def __init__(self, name, calls):
        self.name, self.calls = name, calls
        self.state = SystemState.DISCONNECTED
        self.last_error = None
        self.holding = False
        self.start_hook = self.engage_hook = self.tick_hook = None
        self.prepare_hook = self.pause_hook = self.close_hook = None

    def start(self):
        self.calls.append(f"{self.name}.start")
        if self.start_hook:
            self.start_hook()
        self.state = SystemState.READY

    def prepare_engage(self):
        self.calls.append(f"{self.name}.prepare")
        self.holding = True
        if self.prepare_hook:
            self.prepare_hook()

    def begin_follow(self):
        self.calls.append(f"{self.name}.follow")
        self.holding = False
        if self.state != SystemState.CLOSED:
            self.state = SystemState.ENGAGED

    def engage(self, profile=None):
        self.calls.append(f"{self.name}.engage")
        if self.engage_hook:
            self.engage_hook()
        if self.state != SystemState.CLOSED:
            self.state = SystemState.ENGAGED

    def pause(self, reason):
        self.calls.append(f"{self.name}.pause")
        self.state, self.last_error = SystemState.PAUSED, reason
        self.holding = True
        if self.pause_hook:
            self.pause_hook()

    def tick(self, now=None):
        self.calls.append(f"{self.name}.tick")
        if self.tick_hook:
            self.tick_hook()
        return "arm target" if self.state == SystemState.ENGAGED else None

    def health(self):
        return Health(self.state not in (SystemState.DISCONNECTED, SystemState.CLOSED), 0)

    def status(self, *, include_target=True):
        return {"state": self.state.value, "last_error": self.last_error}

    def close(self):
        self.calls.append(f"{self.name}.close")
        self.state = SystemState.CLOSED
        if self.close_hook:
            self.close_hook()
