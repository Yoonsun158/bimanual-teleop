"""Crossed Quest following in world or leveled-headset axes for Tianji arms.

The caller ticks at 200 Hz. Device receivers retain their own rates; live
callbacks inspect samples and status without commanding motion.
"""

from __future__ import annotations

from collections import deque
from dataclasses import asdict, replace
import math
import threading
import time
import uuid

from bimanual_teleop.devices.quest.adapter import QuestFrame, QuestSource
from bimanual_teleop.devices.tianji.driver import TianjiDriver, TianjiFrame
from bimanual_teleop.devices.tianji.model import MotionProfile, TianjiKinematics
from bimanual_teleop.devices.interfaces import RealtimeObserver
from bimanual_teleop.system import SystemState
from bimanual_teleop.control.arm.mapping import (
    CONTROLLER_FOR_ARM, MAPPING_ID, PoseGoalFilter, PoseGoalInterpolator, QuestTianjiMapper,
    _from_matrix, matmul, rotate_vector, rotation_matrix,
)
from bimanual_teleop.control.arm.cartesian import TianjiCartesianExecutor
from bimanual_teleop.types import (
    ControlProfile, Event, Health, OperatorInput, Pose, RobotState,
    Sample, SampleHeader, SampleRef, TrackedPose,
)


SIDES = ("left", "right")
INPUT_TIMEOUT_NS = 100_000_000


def _headset_yaw(orientation):
    rotation = rotation_matrix(orientation)
    x, y = rotation[0][0], rotation[1][0]
    if math.hypot(x, y) < 1e-6:
        raise ValueError("Quest head yaw is undefined near vertical")
    return math.atan2(y, x)


def operator_input(sample: Sample[QuestFrame], *, sides=SIDES, coordinate_frame="headset") -> OperatorInput:
    """Physical controller names and source timing, in the selected FLU frame."""
    if coordinate_frame not in ("headset", "world"):
        raise ValueError("coordinate_frame must be headset or world")
    sides = tuple(sides)
    frame = sample.payload
    required = ("head", *sides) if coordinate_frame == "headset" else sides
    header = replace(sample.header, valid=all(getattr(frame, side).valid for side in required))
    if coordinate_frame == "headset":
        head = frame.head
        if not head.valid or not head.tracked:
            raise ValueError("Quest head tracking unavailable")
        yaw = _headset_yaw(head.orientation_xyzw)
        c, s = math.cos(yaw), math.sin(yaw)
        headset_from_world = ((c, s, 0.), (-s, c, 0.), (0., 0., 1.))
    wrists = {}
    for side in sides:
        tracked = getattr(frame, side)
        pose = None if not tracked.valid else Pose(
            tracked.parent_frame, tracked.child_frame, tracked.position_m, tracked.orientation_xyzw)
        if pose is not None and coordinate_frame == "headset":
            pose = replace(pose, parent_frame=f"quest_headset_flu/{frame.session}/{frame.origin}",
                position_m=rotate_vector(headset_from_world, tuple(p-h for p, h in zip(
                    pose.position_m, head.position_m))),
                orientation_xyzw=_from_matrix(matmul(headset_from_world, rotation_matrix(pose.orientation_xyzw))))
        wrists[side] = Sample(header, TrackedPose(
            pose, tracked.position_valid, tracked.orientation_valid,
            tracked.position_tracked, tracked.orientation_tracked))
    return OperatorInput(wrists, {})


class QuestInputMonitor:
    """Latch even an invalid→valid burst; estimate additional queue age only.

    min(receive - query) is an offset-plus-minimum-transport baseline, not clock
    synchronization. It resets only on a new source session, never on re-engage.
    """

    def __init__(self, sink: RealtimeObserver | None = None, *, timeout_ns=INPUT_TIMEOUT_NS,
                 sides=SIDES, require_head=True):
        if timeout_ns <= 0:
            raise ValueError("input timeout must be positive")
        self.sink, self.timeout_ns = sink, timeout_ns
        self.sides = tuple(sides)
        self.require_head = require_head
        if not self.sides or len(set(self.sides)) != len(self.sides) or any(s not in SIDES for s in self.sides):
            raise ValueError("Select left, right, or both Quest controllers")
        self._lock = threading.Lock()
        self.latest = None
        self.fault = None
        self.generation = 0
        self._baseline_ns = None
        self._history = deque(maxlen=16)
        self._observer_error = None

    def _observer_failure(self, error=None):
        with self._lock:
            if self._observer_error is not None:
                return
            self._observer_error = (f"Quest input observer failed: {error}" if error else
                                    "Quest input observer rejected live data")
            self._latch(self._observer_error)

    def _latch(self, reason):
        self.fault = self.fault or reason
        self.generation += 1

    def _tracking_problem(self, sample):
        frame = sample.payload
        if frame.session_state != 5:
            return "Quest XR session not focused"
        for side in (("head", *self.sides) if self.require_head else self.sides):
            pose = getattr(frame, side)
            if not pose.valid or not pose.tracked or pose.active is False:
                return f"Quest {side} tracking unavailable"
        if self.require_head:
            try:
                _headset_yaw(frame.head.orientation_xyzw)
            except ValueError as error:
                return str(error)
        return None

    def try_publish(self, sample):
        with self._lock:
            previous = self.latest
            frame = sample.payload
            if previous is not None:
                old = previous.payload
                if frame.session != old.session:
                    self._baseline_ns = None
                if (frame.session, frame.origin) != (old.session, old.origin):
                    self._latch("Quest reference changed; press Enter to re-engage")
                elif frame.sequence <= old.sequence or frame.query_monotonic_ns <= old.query_monotonic_ns:
                    self._latch("Quest sequence or source query time did not advance")
            offset = sample.header.received_monotonic_ns - frame.query_monotonic_ns
            self._baseline_ns = offset if self._baseline_ns is None else min(offset, self._baseline_ns)
            self.latest = sample
            self._history.append(sample)
            problem = self._tracking_problem(sample)
            if offset - self._baseline_ns >= self.timeout_ns:
                problem = problem or "Quest additional input backlog exceeded 100 ms"
            if problem:
                self._latch(problem)
        if self.sink is None:
            return True
        if self._observer_error is not None:
            return False
        try:
            accepted = self.sink.try_publish(sample)
        except Exception as error:
            self._observer_failure(error)
            return False
        if not accepted:
            self._observer_failure()
        return bool(accepted)

    def try_event(self, event):
        if isinstance(event, Event):
            kind = event.kind
            details = event.details.get("details", event.details)
            with self._lock:
                if kind in ("quest.reference_space_change", "quest.origin_changed"):
                    self._latch("Quest reference changed; press Enter to re-engage")
                elif kind == "quest.session_state" and details.get("state") != 5:
                    self._latch("Quest session lost focus")
                elif kind in ("quest.disconnected", "quest.error", "quest.malformed", "quest.out_of_order",
                              "quest.reference_metadata_gap", "quest.reference_time_mismatch"):
                    self._latch(kind)
        if self.sink is None:
            return True
        if self._observer_error is not None:
            return False
        try:
            accepted = self.sink.try_event(event)
        except Exception as error:
            self._observer_failure(error)
            return False
        if not accepted:
            self._observer_failure()
        return bool(accepted)

    def current(self, now_ns, *, acknowledge=False, check_latch=False):
        with self._lock:
            sample = self.latest
            if self._observer_error is not None:
                raise RuntimeError(self._observer_error)
            if sample is None:
                raise RuntimeError("Waiting for Quest frames")
            problem = self._tracking_problem(sample)
            deadline = min(sample.header.received_monotonic_ns + self.timeout_ns,
                           sample.payload.query_monotonic_ns + self._baseline_ns + self.timeout_ns)
            if now_ns >= deadline:
                problem = problem or "Quest input is silent or additionally queued beyond 100 ms"
            if problem:
                if check_latch:
                    self._latch(problem)
                raise RuntimeError(problem)
            if acknowledge:
                self.fault = None
            elif check_latch and self.fault:
                raise RuntimeError(self.fault)
            return sample, deadline, self.generation

    def since(self, previous_ref, through_ref):
        """Snapshot each new source frame, including frames received in a burst."""
        with self._lock:
            return [(sample, min(sample.header.received_monotonic_ns + self.timeout_ns,
                                 sample.payload.query_monotonic_ns + self._baseline_ns + self.timeout_ns))
                    for sample in self._history
                    if sample.header.ref.epoch == through_ref.epoch
                    and previous_ref.sequence < sample.header.ref.sequence <= through_ref.sequence]

    def health(self):
        now = time.monotonic_ns()
        try:
            self.current(now, check_latch=True)
            return Health(True, now)
        except RuntimeError as error:
            return Health(False, now, str(error))


class QuestTianjiTeleop:
    """One owner of the selected arms; preview never configures or submits."""

    def __init__(self, quest: QuestSource, driver: TianjiDriver,
                 executor: TianjiCartesianExecutor, kinematics: TianjiKinematics, *,
                 profile: ControlProfile, sink: RealtimeObserver | None = None,
                 enable_motion: bool = False, clock_ns=time.monotonic_ns, side="both",
                 coordinate_frame="headset"):
        self.quest, self.driver, self.executor, self.kinematics = quest, driver, executor, kinematics
        if side not in ("left", "right", "both"):
            raise ValueError("side must be left, right, or both")
        if coordinate_frame not in ("headset", "world"):
            raise ValueError("coordinate_frame must be headset or world")
        self.side = side
        self.sides = SIDES if side == "both" else (side,)
        self.controller_sides = tuple(CONTROLLER_FOR_ARM[s] for s in self.sides)
        self.coordinate_frame = coordinate_frame
        self.profile = profile
        self.motion_profile = MotionProfile.from_control_profile(profile, driver.model_path)
        if set(self.motion_profile.active_arms) != set(self.sides):
            raise ValueError("Quest teleoperation profile must configure exactly the selected arms")
        self.sink, self.enable_motion, self.clock_ns = sink, enable_motion, clock_ns
        self.input = QuestInputMonitor(sink, sides=self.controller_sides,
                                       require_head=coordinate_frame == "headset")
        self._state = SystemState.DISCONNECTED
        self.mode = None
        self.last_error = None
        self._epoch = uuid.uuid4().hex
        self._sequence = 0
        self._mapper = self._interpolator = self._filter = None
        self._last_input_ref = None
        self._source_time_offset_ns = None
        self._last_target = None
        self._preview_q = {}
        self._robot_fault = None
        self._observer_error = None
        self._owns_motion = False
        self.cycles = 0
        self.last_compute_ns = 0

    @property
    def state(self):
        return self._state

    def _event(self, kind, details):
        if self.sink is None or self._observer_error is not None:
            return
        try:
            if self.sink.try_event(Event(f"teleop.{kind}", self.clock_ns(), "quest_tianji", details)):
                return
            self._observer_failure()
        except Exception as error:
            self._observer_failure(error)

    def _observer_failure(self, error=None):
        if self._observer_error is not None:
            return
        self._observer_error = (f"Realtime observer failed: {error}" if error else
                                "Realtime observer rejected live data")
        self._robot_fault = self._observer_error
        if self.state == SystemState.ENGAGED:
            self.pause(self._observer_error)

    def _publish_target(self, stream, target):
        if self.sink is None:
            return
        if self._observer_error is not None:
            raise RuntimeError(self._observer_error)
        self._sequence += 1
        try:
            accepted = self.sink.try_publish(Sample(SampleHeader(
                SampleRef(f"teleop.{stream}", self._epoch, self._sequence),
                target.created_monotonic_ns, True), target))
        except Exception as error:
            self._observer_failure(error)
            raise RuntimeError(self._observer_error) from error
        if not accepted:
            self._observer_failure()
            raise RuntimeError(self._observer_error)

    def try_publish(self, sample):
        # Called by Tianji's receiver, including every feedback packet in a burst.
        if self._owns_motion and isinstance(sample.payload, TianjiFrame):
            if ((self.side == "both" and not sample.header.valid) or
                    any(sample.payload.arms[s].error or any(q is None for q in
                        (*sample.payload.arms[s].joints.position_rad,
                         *sample.payload.arms[s].joints.velocity_rad_s)) for s in self.sides)):
                self._robot_fault = "Invalid Tianji feedback or controller error"
            elif self.state == SystemState.ENGAGED and any(
                    sample.payload.arms[s].state != 3 or sample.payload.arms[s].impedance_type != 2 for s in self.sides):
                self._robot_fault = "Tianji Cartesian impedance mode changed"
        if self.sink is None:
            return True
        if self._observer_error is not None:
            return False
        try:
            accepted = self.sink.try_publish(sample)
        except Exception as error:
            self._observer_failure(error)
            return False
        if not accepted:
            self._observer_failure()
        return bool(accepted)

    def try_event(self, event):
        if self.sink is None:
            return True
        if self._observer_error is not None:
            return False
        try:
            accepted = self.sink.try_event(event)
        except Exception as error:
            self._observer_failure(error)
            return False
        if not accepted:
            self._observer_failure()
        return bool(accepted)

    def start(self):
        if self.state != SystemState.DISCONNECTED:
            raise RuntimeError("Create a new runtime after start/close")
        try:
            self.driver.start(sink=self)
            self.quest.start(sink=self.input)
            self._state = SystemState.READY
            self._event("started", {"motion_enabled": self.enable_motion, "profile": asdict(self.profile),
                                   "coordinate_frame": self.coordinate_frame,
                                   "quest": dict(self.quest.metadata), "tianji": dict(self.driver.metadata)})
            if self._observer_error:
                raise RuntimeError(self._observer_error)
        except BaseException:
            self.close()
            raise

    def _robot_state(self):
        status = self.driver.health(sides=self.sides)
        if not status.ready:
            raise RuntimeError(status.detail)
        sample = self.driver.get_latest()
        if sample is None:
            raise RuntimeError("Waiting for Tianji feedback")
        joints, poses = {}, {}
        header = sample.header if self.side == "both" else replace(sample.header, valid=True)
        for side in self.sides:
            joint = sample.payload.arms[side].joints
            joints[f"{side}_arm"] = Sample(header, joint)
            poses[side] = Sample(header, self.kinematics.fk(side, joint.position_rad))
        return RobotState(joints, poses)

    def _quest(self, *, acknowledge=False, check_latch=False):
        status = self.quest.health(sides=self.controller_sides, require_head=self.coordinate_frame == "headset")
        if not status.ready:
            raise RuntimeError(status.detail)
        return self.input.current(self.clock_ns(), acknowledge=acknowledge, check_latch=check_latch)

    def _operator_input(self, sample):
        return operator_input(sample, sides=self.controller_sides, coordinate_frame=self.coordinate_frame)

    def _activate_robot(self):
        # Configuration is explicitly deferred until a live keyboard action.
        if self.driver.profile is None:
            self.executor.configure(self.profile)
        self._owns_motion = True
        self._robot_fault = None
        self.executor.engage()
        if self._robot_fault:
            raise RuntimeError(self._robot_fault)
        return RobotState({}, {s: Sample(SampleHeader(self.executor.engagement_ref,
            self.driver.engagement_sample.header.received_monotonic_ns, True), pose)
            for s, pose in self.executor.engagement_poses.items()})

    def engage(self, profile: ControlProfile | None = None):
        if profile is not None and profile != self.profile:
            raise ValueError("Restart with the intended profile; profiles cannot change while running")
        if self.state not in (SystemState.READY, SystemState.PAUSED):
            raise RuntimeError("Pause before engaging again")
        _, _, generation = self._quest(acknowledge=True)
        robot = self._robot_state()
        try:
            if self.enable_motion:
                robot = self._activate_robot()
            else:
                self._preview_q = {s: tuple(robot.joints[f"{s}_arm"].payload.position_rad) for s in self.sides}
                for side in self.sides:
                    self.kinematics.ik(side, robot.tool_poses[side].payload, self._preview_q[side])
            sample, deadline, after = self._quest(check_latch=True)
            if generation != after:
                raise RuntimeError("Quest became invalid during engagement")
            self._mapper = QuestTianjiMapper(self.profile.profile_id, sides=self.sides)
            operator = self._operator_input(sample)
            self._mapper.reset_reference(operator, robot)
            poses = {s: robot.tool_poses[s].payload for s in self.sides}
            self._interpolator = PoseGoalInterpolator(poses, nominal_period_s=1/sample.payload.refresh_hz)
            self._filter = PoseGoalFilter(poses)
            now = self.clock_ns()
            # Fixed timeline alignment only; this is not clock synchronization
            # or an estimate of one-way transport latency.
            self._source_time_offset_ns = sample.header.received_monotonic_ns - sample.payload.query_monotonic_ns
            anchor = self._mapper.compute(operator, robot, now_monotonic_ns=now)
            anchor = replace(anchor, expires_monotonic_ns=min(anchor.expires_monotonic_ns, deadline))
            self._interpolator.set_goal(self._filter.update(anchor), sample_time_ns=sample.header.received_monotonic_ns)
            self._last_target = None
            self._last_input_ref = sample.header.ref
            self._state = SystemState.ENGAGED
            self.mode = "follow" if self.enable_motion else "preview"
            self.last_error = None
            self._event("engaged", {"mode": self.mode, "side": self.side, "quest_anchor": asdict(sample),
                                    "robot_anchors": asdict(robot), "mapping_id": MAPPING_ID,
                                    "coordinate_frame": self.coordinate_frame,
                                    "controller_for_arm": {s: CONTROLLER_FOR_ARM[s] for s in self.sides},
                                    "interpolation_delay_ns": self._interpolator.delay_ns,
                                    "base_from_quest": self._mapper.base_from_quest})
            if self._observer_error:
                raise RuntimeError(self._observer_error)
        except BaseException as error:
            self.pause(str(error))
            raise

    def resume(self):
        self.engage()

    def pause(self, reason):
        if self.state == SystemState.CLOSED:
            return
        owned = self._owns_motion
        self._owns_motion = False
        self._state, self.mode, self.last_error = SystemState.PAUSED, None, reason
        self._mapper = self._interpolator = self._filter = None
        self._last_input_ref = None
        self._last_target = None
        try:
            if owned:
                self.executor.request_hold(reason)
        finally:
            self._event("paused", {"reason": reason, "stop_requested": owned,
                                   "physical_stop_confirmed": False})

    def tick(self, now_monotonic_ns=None):
        """One bounded command step; the caller skips missed scheduling deadlines."""
        if self.state != SystemState.ENGAGED:
            return None
        now = self.clock_ns() if now_monotonic_ns is None else now_monotonic_ns
        try:
            if self._robot_fault:
                raise RuntimeError(self._robot_fault)
            if self._owns_motion and not self.driver.engaged:
                raise RuntimeError("Tianji driver stopped accepting motion")
            robot = self._robot_state()
            latest, _, _ = self._quest(check_latch=True)
            for sample, deadline in self.input.since(self._last_input_ref, latest.header.ref):
                goal = self._mapper.compute(self._operator_input(sample), robot,
                                            now_monotonic_ns=self.clock_ns())
                goal = replace(goal, expires_monotonic_ns=min(goal.expires_monotonic_ns, deadline))
                self._interpolator.set_goal(self._filter.update(goal),
                    sample_time_ns=sample.payload.query_monotonic_ns + self._source_time_offset_ns)
                self._publish_target("goals", goal)
                self._last_input_ref = sample.header.ref
            step_ns = self.clock_ns()
            target = self._interpolator.sample(step_ns)
            # A receiver may have reported a transient fault while FK/mapping ran.
            self._quest(check_latch=True)
            if self._owns_motion:
                result = self.executor.submit(target, before_submit=lambda: self._quest(check_latch=True))
                if not result.accepted:
                    raise RuntimeError(result.reason)
            else:
                q = {s: self.kinematics.ik(s, target.tool_poses[s], self._preview_q[s]) for s in self.sides}
                if self.clock_ns() >= target.expires_monotonic_ns:
                    raise RuntimeError("Preview target expired during IK")
                self._quest(check_latch=True)
                self._preview_q = q
            self._interpolator.accept(target)
            self._last_target = target
            self._publish_target("commands" if self._owns_motion else "preview", target)
            self.cycles += 1
            self.last_compute_ns = self.clock_ns() - now
            return target
        except (RuntimeError, ValueError) as error:
            self.pause(str(error))
            return None

    def health(self):
        now = self.clock_ns()
        try:
            if self.state in (SystemState.DISCONNECTED, SystemState.CLOSED):
                raise RuntimeError(self.state.value)
            if self._robot_fault:
                raise RuntimeError(self._robot_fault)
            status = self.driver.health(sides=self.sides)
            if not status.ready:
                raise RuntimeError(status.detail)
            self._quest(check_latch=self.state == SystemState.ENGAGED)
            return Health(True, now, "following" if self.state == SystemState.ENGAGED
                          else "ready; motion requires explicit engagement")
        except (ValueError, RuntimeError) as error:
            return Health(False, now, str(error))

    def status(self, *, include_target=True):
        return {"state": self.state.value, "motion_enabled": self.enable_motion, "mode": self.mode,
                "side": self.side,
                "coordinate_frame": self.coordinate_frame,
                "controller_for_arm": {s: CONTROLLER_FOR_ARM[s] for s in self.sides},
                "mapping_id": MAPPING_ID, "reference_ready": self._mapper is not None,
                "last_error": self.last_error, "health": asdict(self.health()),
                "cycles": self.cycles, "last_compute_ns": self.last_compute_ns,
                "pose_filter_retention": .8,
                "interpolation_delay_ns": self._interpolator.delay_ns if self._interpolator else None,
                "last_target": asdict(self._last_target) if include_target and self._last_target else None}

    def close(self):
        if self.state == SystemState.CLOSED:
            return
        try:
            self.pause("Teleoperation closed")
        finally:
            try:
                self.driver.close()
            finally:
                try:
                    self.quest.close()
                finally:
                    self._state = SystemState.CLOSED
                    self._event("closed", {"physical_stop_confirmed": False})
