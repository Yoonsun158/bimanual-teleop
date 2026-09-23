"""Deterministic non-physical backend. Never imports a control SDK."""
from copy import deepcopy
from dataclasses import asdict
import time

from bimanual_teleop.types import Pose


class FakeBackend:
    def __init__(self, sides=("left", "right"), clock=time.monotonic_ns):
        self.sides, self.clock = tuple(sides), clock
        self.poses = {s: Pose(f"tianji_{s}_base", f"tianji_{s}_flange",
                             (.4, 0., .2), (0., 0., 0., 1.)) for s in self.sides}
        self.joints = {s: (0.,) * 20 for s in self.sides}
        self.currents = {s: (0.,) * 20 for s in self.sides}
        self.arm_enabled = False
        self.hand_enabled = set()
        self.problems = []
        self.calls = []
        self.closed = False
        self.freeze_arms = set()
        self.freeze_hands = set()

    def open(self):
        self.calls.append("open")

    def snapshot(self):
        return deepcopy({"simulated": True, "monotonic_ns": self.clock(),
            "healthy": not self.problems and not self.closed, "problems": list(self.problems),
            "arms": {s: {"pose": asdict(p), "joints_rad": [0.] * 7,
                "velocity_rad_s": [0.] * 7, "state": 3 if self.arm_enabled else 0,
                "error": 0, "age_s": 0.} for s, p in self.poses.items()},
            "hands": {s: {"position_rad": self.joints[s], "velocity_rad_s": [0.] * 20,
                "current_a": self.currents[s], "enabled": s in self.hand_enabled,
                "age_s": 0.} for s in self.sides}})

    def configure(self):
        if not self.snapshot()["healthy"]:
            raise RuntimeError("fake hardware unhealthy")
        self.calls.append("configure")

    def engage_hand(self, side):
        self.hand_enabled.add(side)
        self.calls.append("engage_hand:" + side)
        return self.joints[side]

    def engage_arms(self):
        self.arm_enabled = True
        self.calls.append("engage_arms")
        return dict(self.poses)

    def send_arms(self, poses, now_ns):
        if not self.arm_enabled or not self.snapshot()["healthy"]:
            raise RuntimeError("fake arms disabled/unhealthy")
        if set(poses) != set(self.sides):
            raise ValueError("arm target group mismatch")
        for side in self.sides:
            if side not in self.freeze_arms:
                self.poses[side] = poses[side]

    def send_hand(self, side, joints, now_ns):
        if side not in self.hand_enabled or not self.snapshot()["healthy"]:
            raise RuntimeError("fake hand disabled/unhealthy")
        if side not in self.freeze_hands:
            self.joints[side] = tuple(joints)

    def fault_stop(self, reason):
        self.calls.append("fault_stop:" + reason)
        self.arm_enabled = False
        self.hand_enabled.clear()

    def close(self):
        self.calls.append("close")
        self.arm_enabled = False
        self.hand_enabled.clear()
        self.closed = True
