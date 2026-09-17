"""Driver boundary tests with synthetic packets; never connect to a robot."""

from collections import deque
from copy import deepcopy
import ctypes as ct
import time

from bimanual_teleop.devices.tianji.driver import TianjiDriver, TianjiJointCommand, FeedbackSnapshot
from bimanual_teleop.types import ControlProfile, DeviceCommand, Event


class Sink:
    def __init__(self):
        self.samples, self.events = [], []
        self.accept = True

    def try_publish(self, sample):
        if self.accept:
            self.samples.append(sample)
        return self.accept

    def try_event(self, event):
        self.events.append(event)
        return True


def packet(index=0, sequences=None, stamp=None):
    value = FeedbackSnapshot()
    value.packet_index = index
    value.received_ns = time.monotonic_ns() if stamp is None else stamp
    value.sequence[:] = sequences or (index, index)
    value.q[:] = [12, -22, 31, -48, 17, 15, -12] * 2
    return value


class FakeSDK:
    """Lifecycle fixture at the Python SDK boundary; no transport receipts."""
    def __init__(self, driver=None):
        self.driver = driver
        self.feedback = deque()
        self.calls = []
        self.version = (100343014, 100343015)
        self.echo = True
        self.hold_failures = 0
        self.stop_nack = False
        self.fail_open = False
        self.servo_errors = {}
        self.reset_rejected = False
        self.reset_without_state_change = False
        self.complete_joint_move = True
        self.send_joint_move = True
        self.send_submit = True
        self.send_configure = True
        self.disable_send = True
        self.disable_feedback = True

    def __getattr__(self, name):
        if name in {"open", "close", "configure", "engage", "submit", "move_joints", "hold", "disable", "clear_errors"}:
            return lambda *args: self.call(name, *args)
        raise AttributeError(name)

    def call(self, name, *args):
        self.calls.append((name, args))
        submitted = time.monotonic_ns()
        if name == "open" and self.fail_open:
            raise RuntimeError("already owned")
        if name == "hold":
            if self.hold_failures:
                self.hold_failures -= 1
                raise RuntimeError("SDK command pending")
            if self.stop_nack:
                raise RuntimeError("RSTA0 returned 1; inspect feedback")
        if name == "clear_errors" and self.reset_rejected:
            raise RuntimeError("RESET returned 1")
        if name == "configure" and not self.send_configure:
            raise RuntimeError("SDK configure rejected")
        if name == "disable" and not self.disable_send:
            raise RuntimeError("SDK disable rejected")
        joint_submit = name == "submit" and self.driver._moving_side is not None
        if (name == "move_joints" or joint_submit) and not self.send_joint_move:
            raise RuntimeError("SDK joint move rejected")
        if name == "submit" and not joint_submit and not self.send_submit:
            raise RuntimeError("SDK submit rejected")
        if name in ("open", "close", "hold"):
            return submitted
        p = deepcopy(self.driver._packet)
        changed = False
        if name == "disable" and self.disable_feedback:
            for arm in range(2):
                if args[0] & (1 << arm):
                    p.sequence[arm] += 1
                    p.state[arm] = p.error[arm] = 0
                    p.commanded_state[arm] = -1
                    p.low_speed[arm] = 1
                    p.dq[arm*7:arm*7+7] = [0]*7
            changed = True
        if name == "clear_errors" and not self.reset_without_state_change:
            p.state[args[0]], p.error[args[0]] = 0, 0
            for joint in range(7):
                self.servo_errors[f"SERVO{args[0]}ERR{joint}"] = 0
            p.sequence[0] += 1
            p.sequence[1] += 1
            changed = True
        if name == "move_joints" and self.complete_joint_move:
            index = args[0]
            p.sequence[index] += 1
            target = p.q[index*7:index*7+7] if p.state[index] != 1 else args[1]
            p.state[index], p.low_speed[index] = 1, 1
            p.target[index*7:index*7+7] = [ct.c_float(q).value for q in target]
            changed = True
        if joint_submit and self.complete_joint_move:
            for index in range(2):
                if args[0] & (1 << index):
                    p.sequence[index] += 1
                    p.low_speed[index] = 1
                    p.target[index*7:index*7+7] = [ct.c_float(q).value for q in args[1][index*7:index*7+7]]
                    p.q[index*7:index*7+7] = p.target[index*7:index*7+7]
            changed = True
        if self.echo and name in ("configure", "engage"):
            for i in range(2):
                p.sequence[i] += 1
                if not args[0] & (1 << i):
                    continue
                if name == "configure":
                    profile = args[1][i]
                    p.cart_k[i*7:i*7+7] = [ct.c_float(v).value for v in (*profile.stiffness, profile.nullspace_stiffness)]
                    p.cart_d[i*7:i*7+7] = [ct.c_float(v).value for v in (*profile.damping, profile.nullspace_damping)]
                    p.tool_dynamics[i*10:i*10+10] = [ct.c_float(v).value for v in profile.tool_dyn10]
                    p.velocity_ratio[i], p.acceleration_ratio[i] = profile.velocity_ratio, profile.acceleration_ratio
                    p.force_type[i] = 1
                else:
                    p.state[i], p.impedance_type[i] = 3, 2
            changed = True
        if changed:
            p.packet_index += 1
            p.received_ns = time.monotonic_ns()
            self.driver._on_feedback(p)
        return submitted

    def poll_feedback(self):
        return self.feedback.popleft() if self.feedback else None

    def versions(self):
        return self.version

    def get_int(self, name):
        if name.startswith("SERVO"):
            return self.servo_errors.get(name, 0)
        return 1017 if name.endswith("Type") else 7


class TianjiFixture:
    """Synthetic SDK interface with production driver lifecycle."""

    def __init__(self):
        self.sink = Sink()
        self.driver = TianjiDriver("192.0.2.1", watchdog_s=0.5)
        self.native = FakeSDK(self.driver)
        self.driver._sdk, self.driver._sink = self.native, self.sink
        self.driver._on_feedback(packet())

    @staticmethod
    def make_profile(sides=("left",)):
        arm = {"stiffness": [1]*6, "damping": [0.5]*6, "nullspace_stiffness": 1,
               "nullspace_damping": 0.5, "tool_dyn10": [1, 0, 0, 0, .01, 0, 0, .01, 0, .01],
               "velocity_ratio": 100, "acceleration_ratio": 100}
        return ControlProfile("test", "cartesian_impedance", {
            "active_arms": list(sides), "arms": {side: deepcopy(arm) for side in sides}})

    def configure(self, sides=("left",)):
        self.driver.configure(self.make_profile(sides))

    def position_feedback(self):
        p = deepcopy(self.driver._packet)
        p.state[:] = [1, 1]
        p.sequence[:] = [q+1 for q in p.sequence]
        p.received_ns = time.monotonic_ns()
        self.driver._on_feedback(p)

    def engage(self, sides=("left",)):
        self.configure(sides)
        seed = self.driver.get_latest()
        self.driver.engage()
        return seed

    def command(self, command_id="move", *, expires=None, targets=None):
        now = time.monotonic_ns()
        targets = targets or {side: self.driver.get_latest().payload.arms[side].joints.position_rad
                              for side in self.driver.profile.active_arms}
        return DeviceCommand("tianji", command_id, TianjiJointCommand(targets, {}), (), now,
                             now + 100_000_000 if expires is None else expires, "test")

    def events(self, kind):
        return [e for e in self.sink.events if isinstance(e, Event) and e.kind == kind]

    def set_emergency_feedback(self, errors=(13, 13)):
        p = packet(self.driver._packet.packet_index+1)
        p.sequence[:] = [s+1 for s in self.driver._packet.sequence]
        p.error[:] = errors
        p.state[:] = [100 if e else 0 for e in errors]
        p.low_speed[:] = [1, 1]
        self.driver._on_feedback(p)
