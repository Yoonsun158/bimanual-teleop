"""Synthetic acquisition and the real Cartesian executor; no device access."""

import math
from types import SimpleNamespace
from unittest.mock import patch

from bimanual_teleop.devices.quest.adapter import QuestFrame, QuestPose
from bimanual_teleop.devices.tianji.driver import TianjiFrame, TianjiArmState, classify_feedback
from bimanual_teleop.devices.tianji.model import M6Model
from bimanual_teleop.control.arm.quest import QuestTianjiTeleop
from bimanual_teleop.control.arm.cartesian import TianjiCartesianExecutor
from bimanual_teleop.types import ControlProfile, Health, JointState, Pose, Sample, SampleHeader, SampleRef, Submission

from tests.support.clock import Clock

SIDES = ("left", "right")


class Sink:
    def __init__(self): self.samples, self.events, self.ready = [], [], True
    def try_publish(self, sample): self.samples.append(sample); return self.ready
    def try_event(self, event): self.events.append(event); return self.ready
    def health(self): return Health(self.ready, 0, None if self.ready else "disk failed")


class Quest:
    def __init__(self, clock):
        self.clock = clock
        self.sink = None
        self.sequence = -1
        self.positions = {"head": (0., 0., 1.5), "left": (0., .2, 1.), "right": (0., -.2, 1.)}
        self.origin, self.session = 0, "test"
        self.metadata = {"session": self.session}
        self.latest = None
        self.ready = True
        self.closed = False

    def start(self, sink): self.sink = sink; self.emit()
    def close(self): self.closed = True
    def health(self, *, sides=SIDES, require_head=True):
        return Health(self.ready, self.clock(), None if self.ready else "Quest unavailable")

    def emit(self, *, positions=None, rotations=None, invalid=None, origin=None, session=None, query_ns=None, state=5,
             refresh_hz=90.):
        self.positions.update(positions or {})
        self.origin = self.origin if origin is None else origin
        self.session = self.session if session is None else session
        self.sequence += 1
        query = self.clock()-500_000_000 if query_ns is None else query_ns
        parent = f"quest_local_flu/{self.session}/{self.origin}"
        rotations = {"head": (0., 0., 0., 1.), "left": (-.5, -.5, .5, .5),
                     "right": (.5, -.5, -.5, .5), **(rotations or {})}
        poses = {s: QuestPose(parent, f"quest_{s}_grip_flu", p, rotations[s], 0 if s == invalid else 15,
                               None if s == "head" else True) for s, p in self.positions.items()}
        frame = QuestFrame(self.session, self.sequence, self.origin, query, query+1, query+1, state,
                           refresh_hz, poses["head"], poses["left"], poses["right"])
        self.latest = Sample(SampleHeader(SampleRef("quest.poses", f"{self.session}/origin-{self.origin}",
            self.sequence), self.clock(), invalid is None), frame)
        self.sink.try_publish(self.latest)
        return self.latest


class Kinematics:
    def __init__(self): self.fail_side = None; self.after_ik = None

    def fk(self, side, q):
        return Pose(f"tianji_{side}_base", f"tianji_{side}_flange", tuple(q[:3]),
                    (0., 0., math.sin(q[3]/2), math.cos(q[3]/2)))

    def ik(self, side, pose, reference):
        if self.after_ik: self.after_ik()
        if side == self.fail_side: raise ValueError(f"{side}: unreachable")
        angle = 2*math.atan2(pose.orientation_xyzw[2], pose.orientation_xyzw[3])
        return (*pose.position_m, angle, 0., 0., 0.)

    def jacobian(self, side, reference):
        if self.after_ik: self.after_ik()
        if side == self.fail_side: raise ValueError(f"{side}: invalid Jacobian")
        return ((1., 0., 0., 0., 0., 0., 0.), (0., 1., 0., 0., 0., 0., 0.),
                (0., 0., 1., 0., 0., 0., 0.), (0.,) * 7, (0.,) * 7,
                (0., 0., 0., 1., 0., 0., 0.))


class Driver:
    model_path = None
    controller_ip = "192.168.1.190"

    def __init__(self, clock, parsed):
        self.clock, self.parsed = clock, parsed
        self.profile = self.engagement_sample = None
        self.metadata = {}
        self.engaged, self.ready = False, True
        self.commands, self.holds, self.calls = [], [], []
        self.q = {"left": (.3, .2, .5, 0., 0., 0., 0.), "right": (.3, -.2, .5, 0., 0., 0., 0.)}
        self.index = 0
        self.accept = True
        self.state = 0
        self.after_engage = None
        self._motion_fault = None

    def start(self, sink, *, record_reported_config=True):
        self.sink = sink
        self.record_reported_config = record_reported_config
        self.calls.append("start")
        self.emit()
    def health(self, *, sides=None):
        selected = (self.profile.active_arms if self.profile else SIDES) if sides is None else sides
        error = (self._motion_fault if self.engaged else None) or self.assessment.control_problem_for(selected)
        return Health(self.ready and error is None, self.clock(), error or (None if self.ready else "Tianji stale"))
    def get_latest(self): return self.latest

    def receive(self, sample):
        """The synthetic driver owns the same hardware fault boundary as Tianji."""
        self.latest = sample
        self.assessment = classify_feedback(sample)
        if self.engaged:
            selected = self.profile.active_arms
            self._motion_fault = self._motion_fault or self.assessment.control_problem_for(selected)
            if self._motion_fault is None and any(sample.payload.arms[s].state != 3 or
                                                 sample.payload.arms[s].impedance_type != 2 for s in selected):
                self._motion_fault = "Tianji Cartesian impedance mode changed"
        self.sink.try_publish(sample)

    def emit(self, *, error_side=None, wrong_mode=None):
        self.index += 1
        arms = {}
        for side in SIDES:
            joints = JointState(tuple(f"j{i}" for i in range(7)), self.q[side], (0.,)*7)
            arms[side] = TianjiArmState(joints, self.index, self.index, self.state,
                self.state, 1 if error_side == side else 0, 1 if wrong_mode == side else 2, 1, self.q[side], (0.,)*7)
        self.receive(Sample(SampleHeader(SampleRef("tianji.feedback", "test", self.index), self.clock(), True),
                            TianjiFrame(self.index, arms)))

    def configure(self, profile): self.calls.append("configure"); self.profile = self.parsed
    def engage(self):
        self.calls.append("engage")
        self.engagement_sample = self.latest
        self._motion_fault = None
        self.engaged = True
        self.state = 3
        self.emit()
        if self.after_engage: self.after_engage()

    def submit(self, command):
        self.commands.append(command)
        if self.accept:
            self.q.update(command.payload.targets)
            self.emit()
        return Submission(command.command_id, self.accept, None if self.accept else "send rejected")

    def request_hold(self, reason): self.holds.append(reason); self.engaged = False
    def close(self): self.calls.append("close"); self.engaged = False


class RuntimeFixture:
    """Reusable by the thin CLI integration test."""

    def __init__(self, *, side="both", coordinate_frame="headset", external_sink=True):
        self.clock, self.sink, self.kine = Clock(), Sink() if external_sink else None, Kinematics()
        selected = SIDES if side == "both" else (side,)
        self.parsed = SimpleNamespace(profile_id="test", active_arms=selected,
            model=M6Model.from_file(), arms={s: SimpleNamespace(velocity_ratio=100, acceleration_ratio=100)
                                            for s in selected})
        self.profile = ControlProfile("test", "cartesian_impedance", {})
        self.quest, self.driver = Quest(self.clock), Driver(self.clock, self.parsed)
        self.executor = TianjiCartesianExecutor(self.driver, self.kine, clock_ns=self.clock)
        with patch("bimanual_teleop.control.arm.quest.MotionProfile.from_control_profile", return_value=self.parsed):
            self.runtime = QuestTianjiTeleop(self.quest, self.driver, self.executor,
                profile=self.profile, sink=self.sink, clock_ns=self.clock,
                side=side, coordinate_frame=coordinate_frame)
        self.runtime.start()

    def advance(self, ns=5_000_000, *, positions=None, quest=True):
        self.clock.advance(ns)
        self.driver.emit()
        if quest: self.quest.emit(positions=positions)
