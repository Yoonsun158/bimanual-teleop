"""Relative Quest goals, OPEN TEACH filtering and continuous interpolation.

DROID's rotation increment is left-multiplied onto the robot reference.
OPEN TEACH's bimanual position/rotation filter is applied to new goals.
Filtered poses are resampled on their source query timeline with linear
translation and shortest-path quaternion interpolation, one input frame later.
The bundled upstream license is in licenses/Open-Teach-MIT.txt.
"""

from __future__ import annotations

from collections import deque
from dataclasses import replace
import math
from typing import Mapping, Sequence
import uuid

from bimanual_teleop.types import OperatorInput, Pose, Quaternion, RobotState, RobotTarget, Side

Matrix3 = tuple[tuple[float, float, float], tuple[float, float, float], tuple[float, float, float]]
SIDES: tuple[Side, Side] = ("left", "right")
CONTROLLER_FOR_ARM = {"left": "right", "right": "left"}
# Forward/left/up -> each target arm's native base, independent of grip orientation.
BASE_FROM_FLU: dict[Side, Matrix3] = {
    "left": ((1., 0., 0.), (0., 0., -1.), (0., 1., 0.)),
    "right": ((1., 0., 0.), (0., 0., 1.), (0., -1., 0.)),
}
MAPPING_ID = "quest-crossed-flu-to-tianji-base-20260914.v2"


def _vector(values: Sequence[float], count: int) -> tuple[float, ...]:
    try:
        valid = len(values) == count and all(math.isfinite(value) for value in values)
    except TypeError:
        valid = False
    if not valid:
        raise ValueError(f"Expected {count} finite components")
    return tuple(float(value) for value in values)


def _quaternion(values: Sequence[float]) -> Quaternion:
    q = _vector(values, 4)
    norm = math.sqrt(sum(value * value for value in q))
    if not math.isclose(norm, 1.0, abs_tol=0.001):
        raise ValueError("Expected a unit quaternion in xyzw order")
    return tuple(value / norm for value in q)


def transpose(matrix: Matrix3) -> Matrix3:
    return tuple(zip(*matrix))


def rotate_vector(matrix: Matrix3, vector: Sequence[float]) -> tuple[float, float, float]:
    return tuple(sum(a * b for a, b in zip(row, vector)) for row in matrix)


def matmul(a: Matrix3, b: Matrix3) -> Matrix3:
    return tuple(tuple(sum(x * y for x, y in zip(row, column)) for column in zip(*b)) for row in a)


def _pose(pose: Pose) -> Pose:
    if not pose.parent_frame or not pose.child_frame:
        raise ValueError("Pose parent and child frames must be named")
    return replace(pose, position_m=_vector(pose.position_m, 3),
                   orientation_xyzw=_quaternion(pose.orientation_xyzw))


def rotation_matrix(quaternion: Quaternion) -> Matrix3:
    x, y, z, w = _quaternion(quaternion)
    return ((1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)),
            (2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)),
            (2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)))


def _from_matrix(m: Matrix3) -> Quaternion:
    trace = sum(m[i][i] for i in range(3))
    if trace > 0:
        scale = math.sqrt(trace + 1) * 2
        q = ((m[2][1]-m[1][2])/scale, (m[0][2]-m[2][0])/scale,
             (m[1][0]-m[0][1])/scale, scale/4)
    else:
        i = max(range(3), key=lambda k: m[k][k])
        j, k = (i+1) % 3, (i+2) % 3
        scale = math.sqrt(1 + m[i][i] - m[j][j] - m[k][k]) * 2
        q = [0.0] * 4
        q[i], q[j], q[k] = scale/4, (m[j][i]+m[i][j])/scale, (m[k][i]+m[i][k])/scale
        q[3] = (m[k][j]-m[j][k])/scale
    return _quaternion(q)


def slerp(a: Quaternion, b: Quaternion, fraction: float) -> Quaternion:
    a, b = _quaternion(a), _quaternion(b)
    if not math.isfinite(fraction) or not 0 <= fraction <= 1:
        raise ValueError("Interpolation fraction must be between zero and one")
    dot = sum(x*y for x, y in zip(a, b))
    if dot < 0:
        b, dot = tuple(-x for x in b), -dot
    if dot > 0.9995:
        q = tuple(x + fraction*(y-x) for x, y in zip(a, b))
    else:
        angle = math.acos(min(1.0, dot))
        left, right = math.sin((1-fraction)*angle), math.sin(fraction*angle)
        q = tuple((left*x + right*y) / math.sin(angle) for x, y in zip(a, b))
    norm = math.sqrt(sum(x*x for x in q))
    return tuple(x / norm for x in q)


def _arms(values: Mapping[Side, object], label: str, sides: tuple[Side, ...]) -> None:
    if set(values) != set(sides):
        raise ValueError(f"{label} must contain exactly {', '.join(sides)}")


def _selected_sides(sides: Sequence[Side]) -> tuple[Side, ...]:
    selected = tuple(sides)
    if not selected or len(set(selected)) != len(selected) or any(side not in SIDES for side in selected):
        raise ValueError("Select left, right, or both arms without duplicates")
    return selected


def _operator_poses(operator: OperatorInput, sides: tuple[Side, ...]) -> dict[Side, Pose]:
    _arms(operator.wrists, "Operator wrists", sides)
    poses = {}
    for side, sample in operator.wrists.items():
        tracked = sample.payload
        if not (sample.header.valid and tracked.pose is not None and tracked.position_valid and
                tracked.orientation_valid and tracked.position_tracked and tracked.orientation_tracked):
            raise ValueError(f"{side} controller pose is invalid or untracked")
        poses[side] = _pose(tracked.pose)
    if len(sides) == 2 and poses["left"].parent_frame != poses["right"].parent_frame:
        raise ValueError("Both controller poses must share the same Quest origin")
    return poses


def _robot_poses(robot: RobotState, sides: tuple[Side, ...]) -> dict[Side, Pose]:
    _arms(robot.tool_poses, "Robot tools", sides)
    if any(not sample.header.valid for sample in robot.tool_poses.values()):
        raise ValueError("Robot reference poses must be valid")
    return {side: _pose(sample.payload) for side, sample in robot.tool_poses.items()}


class QuestTianjiMapper:
    """Crossed controllers, fixed FLU axes, and independent relative arm anchors.

    The caller supplies world or leveled-headset poses with physical controller
    names. Translation is 1:1; rotation increments left-multiply robot anchors.
    """

    def __init__(self, profile_id: str, input_timeout_ns: int = 100_000_000,
                 *, sides: Sequence[Side] = SIDES):
        if not profile_id or type(input_timeout_ns) is not int or input_timeout_ns <= 0:
            raise ValueError("A profile ID and positive input timeout are required")
        self.sides = _selected_sides(sides)
        self.controller_sides = tuple(CONTROLLER_FOR_ARM[side] for side in self.sides)
        self.base_from_quest = {side: BASE_FROM_FLU[side] for side in self.sides}
        self.profile_id, self.input_timeout_ns = profile_id, input_timeout_ns
        self._operator_reference: dict[Side, Pose] = {}
        self._robot_reference: dict[Side, Pose] = {}
        self._operator_refs = {}
        self._reference_refs = ()
        self._command_prefix, self._sequence = uuid.uuid4().hex, 0

    def reset_reference(self, operator: OperatorInput, robot: RobotState) -> None:
        operator_poses = _operator_poses(operator, self.controller_sides)
        robot_poses = _robot_poses(robot, self.sides)
        self._operator_reference, self._robot_reference = operator_poses, robot_poses
        self._operator_refs = {side: operator.wrists[side].header.ref for side in self.controller_sides}
        self._reference_refs = tuple(dict.fromkeys(
            [*self._operator_refs.values(), *(robot.tool_poses[side].header.ref for side in self.sides)]))

    def compute(self, operator: OperatorInput, *, now_monotonic_ns: int) -> RobotTarget:
        if not self._operator_reference:
            raise ValueError("Set valid operator and robot references before mapping")
        poses = _operator_poses(operator, self.controller_sides)
        targets, refs = {}, []
        deadline = min(sample.header.received_monotonic_ns + self.input_timeout_ns
                       for sample in operator.wrists.values())
        if (now_monotonic_ns >= deadline or
                any(sample.header.received_monotonic_ns > now_monotonic_ns for sample in operator.wrists.values())):
            raise ValueError("Controller input expired or has a future host arrival time")
        for side in self.sides:
            controller = CONTROLLER_FOR_ARM[side]
            pose, q0, b0 = poses[controller], self._operator_reference[controller], self._robot_reference[side]
            ref, reference_ref = operator.wrists[controller].header.ref, self._operator_refs[controller]
            if ((pose.parent_frame, pose.child_frame) != (q0.parent_frame, q0.child_frame) or
                    (ref.stream, ref.epoch) != (reference_ref.stream, reference_ref.epoch) or
                    ref.sequence < reference_ref.sequence):
                raise ValueError("Controller stream/origin changed; explicit re-engagement is required")
            c = self.base_from_quest[side]
            delta = rotate_vector(c, tuple(p-q for p, q in zip(pose.position_m, q0.position_m)))
            rotation_delta = matmul(rotation_matrix(pose.orientation_xyzw), transpose(rotation_matrix(q0.orientation_xyzw)))
            desired = matmul(matmul(matmul(c, rotation_delta), transpose(c)), rotation_matrix(b0.orientation_xyzw))
            targets[side] = replace(b0, position_m=tuple(p+d for p, d in zip(b0.position_m, delta)),
                                    orientation_xyzw=_from_matrix(desired))
            refs.append(ref)
        self._sequence += 1
        return RobotTarget(f"quest-{self._command_prefix}-{self._sequence}", targets, tuple(dict.fromkeys([*refs, *self._reference_refs])), now_monotonic_ns,
                           deadline, self.profile_id)


def _between(start: Pose, end: Pose, fraction: float) -> Pose:
    return replace(start, position_m=tuple(a + fraction*(b-a) for a, b in zip(start.position_m, end.position_m)),
                   orientation_xyzw=slerp(start.orientation_xyzw, end.orientation_xyzw, fraction))


class PoseGoalFilter:
    """OPEN TEACH's bimanual Filter: position EMA and orientation SLERP.

    Update once per NEW Quest frame. Retain 0.8 of the prior filtered pose,
    matching the authors' enabled bimanual configuration. This is a noise
    filter, not a speed or workspace limit; the bundled license is under licenses/.
    """

    def __init__(self, initial: Mapping[Side, Pose], retention: float = .8):
        self.sides = _selected_sides(tuple(initial))
        _arms(initial, "Initial filter poses", self.sides)
        if not math.isfinite(retention) or not 0 <= retention < 1:
            raise ValueError("Filter retention must be in [0, 1)")
        self.retention = retention
        self._poses = {side: _pose(pose) for side, pose in initial.items()}

    def update(self, goal: RobotTarget) -> RobotTarget:
        _arms(goal.tool_poses, "Filter goal poses", self.sides)
        poses = {}
        for side in self.sides:
            before, after = self._poses[side], _pose(goal.tool_poses[side])
            if (before.parent_frame, before.child_frame) != (after.parent_frame, after.child_frame):
                raise ValueError("Filtering cannot change coordinate frames")
            poses[side] = _between(before, after, 1-self.retention)
        self._poses = poses
        return replace(goal, tool_poses=dict(poses))


class PoseGoalInterpolator:
    """Resample source-timed poses one input period behind the command clock.

    The caller aligns source query times to a fixed host timeline. Position is
    linear and orientation follows the shortest SLERP arc; neither extrapolates.
    """

    def __init__(self, initial: Mapping[Side, Pose], nominal_period_s: float = 1/90):
        self.sides = _selected_sides(tuple(initial))
        _arms(initial, "Initial command poses", self.sides)
        if not math.isfinite(nominal_period_s) or nominal_period_s <= 0:
            raise ValueError("Nominal period must be positive and finite")
        self.delay_ns = round(nominal_period_s * 1e9)
        self._accepted = {side: _pose(pose) for side, pose in initial.items()}
        self._accepted_ns: int | None = None
        self._goal: RobotTarget | None = None
        self._points = deque(maxlen=16)
        self._pending: RobotTarget | None = None
        self._sequence = 0

    def set_goal(self, target: RobotTarget, *, sample_time_ns: int | None = None) -> None:
        _arms(target.tool_poses, "Goal poses", self.sides)
        if target.expires_monotonic_ns <= target.created_monotonic_ns:
            raise ValueError("Expected an unexpired arm-only target")
        if self._goal and (target.control_profile_id != self._goal.control_profile_id or
                           target.created_monotonic_ns < self._goal.created_monotonic_ns):
            raise ValueError("Goal profile changed or goal time moved backwards")
        if self._accepted_ns is not None and target.expires_monotonic_ns <= self._accepted_ns:
            raise ValueError("Goal source deadline has already expired")
        poses = {side: _pose(pose) for side, pose in target.tool_poses.items()}
        for side in self.sides:
            a, b = self._accepted[side], poses[side]
            if (a.parent_frame, a.child_frame) != (b.parent_frame, b.child_frame):
                raise ValueError("Interpolation cannot change a pose's coordinate frames")
        sample_time_ns = target.created_monotonic_ns if sample_time_ns is None else sample_time_ns
        if self._points and sample_time_ns < self._points[-1][0]:
            raise ValueError("Source sample time moved backwards")
        self._goal = replace(target, tool_poses=poses, source_refs=tuple(target.source_refs))
        if self._points and sample_time_ns == self._points[-1][0]:
            self._points.pop()
        self._points.append((sample_time_ns, self._goal))
        self._pending = None

    def sample(self, now_ns: int) -> RobotTarget:
        goal = self._goal
        if goal is None:
            raise ValueError("Set a goal before sampling commands")
        if not goal.created_monotonic_ns <= now_ns < goal.expires_monotonic_ns:
            raise ValueError("Goal is not yet current or its original source deadline expired")
        if self._accepted_ns is not None and now_ns < self._accepted_ns:
            raise ValueError("Command time must advance")
        play_ns = now_ns - self.delay_ns
        while len(self._points) > 1 and self._points[1][0] <= play_ns:
            self._points.popleft()
        before_ns, before = self._points[0]
        after_ns, after = self._points[1] if len(self._points) > 1 else self._points[0]
        fraction = 0.0 if after_ns == before_ns else max(0.0, min(1.0, (play_ns-before_ns)/(after_ns-before_ns)))
        poses = {side: _between(before.tool_poses[side], after.tool_poses[side], fraction) for side in self.sides}
        deadline = min(goal.expires_monotonic_ns, before.expires_monotonic_ns, after.expires_monotonic_ns)
        if now_ns >= deadline:
            raise ValueError("Interpolation source deadline expired")
        refs = tuple(dict.fromkeys((*before.source_refs, *after.source_refs, *goal.source_refs)))
        self._sequence += 1
        target = replace(goal, command_id=f"{goal.command_id}/step-{self._sequence}", tool_poses=poses,
                         source_refs=refs, created_monotonic_ns=now_ns,
                         expires_monotonic_ns=min(deadline, now_ns+50_000_000))
        self._pending = replace(target, tool_poses=dict(poses))
        return target

    def accept(self, target: RobotTarget) -> None:
        """Call only after executor.submit(target).accepted is true."""
        if self._pending is None or target != self._pending:
            raise ValueError("Only the most recently sampled, unchanged command can be accepted")
        self._accepted, self._accepted_ns = dict(self._pending.tool_poses), target.created_monotonic_ns
        self._pending = None
