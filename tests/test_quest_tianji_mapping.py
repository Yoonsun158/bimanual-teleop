"""Offline frame, reference, deadline and interpolation tests; no device SDK."""

from dataclasses import replace
import math
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bimanual_teleop.control.arm.mapping import (
    PoseGoalFilter, PoseGoalInterpolator, QuestTianjiMapper,
    orientation_distance, slerp,
)
from bimanual_teleop.types import (
    OperatorInput, Pose, RobotState, RobotTarget, Sample, SampleHeader, SampleRef, TrackedPose,
)

SIDES = ("left", "right")
T0 = 1_000_000_000


def axis_quat(axis, degrees):
    half = math.radians(degrees) / 2
    values = [0.0, 0.0, 0.0, math.cos(half)]
    values["xyz".index(axis)] = math.sin(half)
    return tuple(values)


def product(a, b):
    """Independent quaternion calculation for expected noncommuting rotations."""
    x, y, z, w = a
    X, Y, Z, W = b
    return (w*X+x*W+y*Z-z*Y, w*Y-x*Z+y*W+z*X,
            w*Z+x*Y-y*X+z*W, w*W-x*X-y*Y-z*Z)


def operator(positions=None, rotations=None, *, sequence=0, received_ns=T0, origin="origin-0"):
    positions = positions or {"left": (1.3, -.8, .7), "right": (-.2, .6, .5)}
    rotations = rotations or {side: axis_quat("y", -90) for side in SIDES}
    header = SampleHeader(SampleRef("quest.poses", f"session/{origin}", sequence), received_ns, True)
    wrists = {side: Sample(header, TrackedPose(
        Pose(f"quest_local_flu/session/{origin}", f"quest_{side}_grip_flu", positions[side], rotations[side]),
        True, True, True, True)) for side in SIDES}
    return OperatorInput(wrists, {})


def robot(positions=None, rotations=None):
    positions = positions or {"left": (.4, -.2, .5), "right": (.3, .2, .6)}
    rotations = rotations or {"left": product(axis_quat("y", -40), axis_quat("z", 15)),
                              "right": product(axis_quat("x", 25), axis_quat("y", 40))}
    header = SampleHeader(SampleRef("tianji.feedback", "robot-session", 20), T0, True)
    return RobotState({}, {side: Sample(header, Pose(f"tianji_{side}_base", f"tianji_{side}_flange",
                                                   positions[side], rotations[side])) for side in SIDES})


def target(poses, *, sequence=1, created_ns=T0, expires_ns=T0+100_000_000):
    return RobotTarget(f"goal-{sequence}", dict(poses), {}, (SampleRef("quest.poses", "session", sequence),),
                       created_ns, expires_ns, "verified-profile")


class MappingTests(unittest.TestCase):
    def setUp(self):
        self.operator, self.robot = operator(), robot()
        self.mapper = QuestTianjiMapper("verified-profile")
        self.mapper.reset_reference(self.operator, self.robot)

    def assertQuaternion(self, actual, expected):
        self.assertAlmostEqual(abs(sum(a*b for a, b in zip(actual, expected))), 1.0, places=8)

    def test_signed_flu_axes_are_fixed_and_control_only_the_opposite_arm(self):
        q0 = {"left": product(axis_quat("x", 65), axis_quat("z", 40)),
              "right": product(axis_quat("y", -35), axis_quat("x", 20))}
        reference = operator(rotations=q0)
        self.mapper.reset_reference(reference, self.robot)
        for controller, arm, axes in (
                ("right", "left", (("x", 1), ("z", 1), ("y", -1))),
                ("left", "right", (("x", 1), ("z", -1), ("y", 1)))):
            for i, (robot_axis, direction) in enumerate(axes):
                for sign in (-1, 1):
                    with self.subTest(controller=controller, axis=i, sign=sign):
                        positions = {s: sample.payload.pose.position_m for s, sample in reference.wrists.items()}
                        positions[controller] = tuple(p + (sign*.02 if j == i else 0.)
                                                      for j, p in enumerate(positions[controller]))
                        rotations = {**q0, controller: product(axis_quat("xyz"[i], sign*20), q0[controller])}
                        result = self.mapper.compute(operator(positions, rotations), self.robot, now_monotonic_ns=T0)
                        for side in SIDES:
                            initial = self.robot.tool_poses[side].payload
                            angle = sign*direction*20 if side == arm else 0
                            delta = tuple(sign*direction*.02 if side == arm and j == "xyz".index(robot_axis)
                                          else 0. for j in range(3))
                            for actual, before, d in zip(result.tool_poses[side].position_m, initial.position_m, delta):
                                self.assertAlmostEqual(actual, before+d)
                            self.assertQuaternion(result.tool_poses[side].orientation_xyzw,
                                                  product(axis_quat(robot_axis, angle), initial.orientation_xyzw))

    def test_engagement_grip_orientation_does_not_change_translation_axes(self):
        for grip in ((0., 0., 0., 1.), axis_quat("y", -90),
                     product(axis_quat("x", 65), axis_quat("z", 40))):
            with self.subTest(grip=grip):
                reference = operator(rotations={s: grip for s in SIDES})
                self.mapper.reset_reference(reference, self.robot)
                positions = {s: tuple(p+d for p, d in zip(sample.payload.pose.position_m, (.01, .02, .03)))
                             for s, sample in reference.wrists.items()}
                result = self.mapper.compute(operator(positions), self.robot, now_monotonic_ns=T0)
                for side, delta in (("left", (.01, -.03, .02)), ("right", (.01, .03, -.02))):
                    for actual, before, d in zip(result.tool_poses[side].position_m,
                                                 self.robot.tool_poses[side].payload.position_m, delta):
                        self.assertAlmostEqual(actual, before+d)

    def test_distinct_noncommuting_controller_rotations_reach_opposite_arms(self):
        left_delta = product(product(axis_quat("z", 35), axis_quat("y", -20)), axis_quat("x", 17))
        right_delta = product(product(axis_quat("y", 31), axis_quat("x", -26)), axis_quat("z", 12))
        rotations = {s: product(delta, self.operator.wrists[s].payload.pose.orientation_xyzw)
                     for s, delta in (("left", left_delta), ("right", right_delta))}
        result = self.mapper.compute(operator(rotations=rotations), self.robot, now_monotonic_ns=T0)
        expected = {
            "left": product(product(axis_quat("z", 31), axis_quat("x", -26)), axis_quat("y", -12)),
            "right": product(product(axis_quat("y", 35), axis_quat("z", 20)), axis_quat("x", 17)),
        }
        for side in SIDES:
            self.assertQuaternion(result.tool_poses[side].orientation_xyzw,
                                  product(expected[side], self.robot.tool_poses[side].payload.orientation_xyzw))

    def test_single_arm_requires_only_the_opposite_physical_controller(self):
        for arm, controller in (("left", "right"), ("right", "left")):
            with self.subTest(arm=arm):
                mapper = QuestTianjiMapper("verified-profile", sides=(arm,))
                selected_robot = replace(self.robot, tool_poses={arm: self.robot.tool_poses[arm]})
                selected_operator = replace(self.operator, wrists={controller: self.operator.wrists[controller]})
                mapper.reset_reference(selected_operator, selected_robot)
                wrist = selected_operator.wrists[controller]
                moved = replace(wrist, payload=replace(wrist.payload, pose=replace(wrist.payload.pose,
                    position_m=tuple(p+d for p, d in zip(wrist.payload.pose.position_m, (.02, 0, 0))))))
                result = mapper.compute(replace(selected_operator, wrists={controller: moved}), selected_robot,
                                        now_monotonic_ns=T0)
                self.assertEqual(set(result.tool_poses), {arm})
                self.assertAlmostEqual(result.tool_poses[arm].position_m[0],
                                       selected_robot.tool_poses[arm].payload.position_m[0]+.02)
                with self.assertRaisesRegex(ValueError, "exactly"):
                    mapper.reset_reference(replace(self.operator, wrists={arm: self.operator.wrists[arm]}), selected_robot)

    def test_rotation_in_place_does_not_translate_a_nonzero_anchor(self):
        rotations = {side: product(axis_quat("z", 25), self.operator.wrists[side].payload.pose.orientation_xyzw)
                     for side in SIDES}
        result = self.mapper.compute(operator(rotations=rotations), self.robot, now_monotonic_ns=T0)
        for side in SIDES:
            self.assertEqual(result.tool_poses[side].position_m, self.robot.tool_poses[side].payload.position_m)

    def test_world_rotation_with_half_turn_robot_reference(self):
        rotations = {side: product(axis_quat("z", 20), self.operator.wrists[side].payload.pose.orientation_xyzw)
                     for side in SIDES}
        for axis in "xyz":
            with self.subTest(axis=axis):
                reference = robot(rotations={side: axis_quat(axis, 180) for side in SIDES})
                self.mapper.reset_reference(self.operator, reference)
                result = self.mapper.compute(operator(rotations=rotations), reference, now_monotonic_ns=T0)
                for side in SIDES:
                    self.assertQuaternion(result.tool_poses[side].orientation_xyzw,
                                          product(axis_quat("y", -20 if side == "left" else 20), axis_quat(axis, 180)))

    def test_tracking_error_does_not_move_reference_and_reengagement_is_continuous(self):
        measured = robot(positions={"left": (.5, -.3, .7), "right": (.1, .4, .9)})
        result = self.mapper.compute(self.operator, measured, now_monotonic_ns=T0)
        for side in SIDES:
            self.assertEqual(result.tool_poses[side].position_m, self.robot.tool_poses[side].payload.position_m)
            self.assertQuaternion(result.tool_poses[side].orientation_xyzw, self.robot.tool_poses[side].payload.orientation_xyzw)
        new_operator = operator(origin="origin-1", sequence=10)
        self.mapper.reset_reference(new_operator, measured)
        result = self.mapper.compute(new_operator, measured, now_monotonic_ns=T0)
        for side in SIDES:
            self.assertEqual(result.tool_poses[side].position_m, measured.tool_poses[side].payload.position_m)
            self.assertQuaternion(result.tool_poses[side].orientation_xyzw, measured.tool_poses[side].payload.orientation_xyzw)

    def test_relative_mapping_has_no_ten_centimeter_or_thirty_degree_envelope(self):
        positions = {side: tuple(p+.25 for p in self.operator.wrists[side].payload.pose.position_m) for side in SIDES}
        rotations = {side: product(axis_quat("x", 90), self.operator.wrists[side].payload.pose.orientation_xyzw)
                     for side in SIDES}
        result = self.mapper.compute(operator(positions=positions, rotations=rotations), self.robot, now_monotonic_ns=T0)
        for side in SIDES:
            initial = self.robot.tool_poses[side].payload
            self.assertAlmostEqual(math.dist(result.tool_poses[side].position_m, initial.position_m), math.sqrt(3)*.25)
            self.assertAlmostEqual(orientation_distance(result.tool_poses[side].orientation_xyzw,
                                                       initial.orientation_xyzw), math.pi/2)

    def test_origin_tracking_and_exact_dual_side_requirements(self):
        with self.assertRaisesRegex(ValueError, "origin changed"):
            self.mapper.compute(operator(origin="origin-1"), self.robot, now_monotonic_ns=T0)
        left = self.operator.wrists["left"]
        invalid = replace(self.operator, wrists={**self.operator.wrists,
                          "left": replace(left, payload=replace(left.payload, position_tracked=False))})
        with self.assertRaisesRegex(ValueError, "untracked"):
            self.mapper.compute(invalid, self.robot, now_monotonic_ns=T0)
        with self.assertRaisesRegex(ValueError, "exactly"):
            self.mapper.compute(replace(self.operator, wrists={"left": left}), self.robot, now_monotonic_ns=T0)

    def test_source_deadline_and_reference_provenance_are_preserved(self):
        current = operator(sequence=7, received_ns=T0+10_000_000)
        a = self.mapper.compute(current, self.robot, now_monotonic_ns=T0+20_000_000)
        b = self.mapper.compute(current, self.robot, now_monotonic_ns=T0+100_000_000)
        self.assertEqual(a.expires_monotonic_ns, T0+110_000_000)
        self.assertEqual(a.expires_monotonic_ns, b.expires_monotonic_ns)
        self.assertIn(current.wrists["left"].header.ref, a.source_refs)
        self.assertIn(self.operator.wrists["left"].header.ref, a.source_refs)
        self.assertIn(self.robot.tool_poses["left"].header.ref, a.source_refs)
        self.assertNotEqual(a.command_id, b.command_id)
        with self.assertRaisesRegex(ValueError, "expired"):
            self.mapper.compute(current, self.robot, now_monotonic_ns=T0+110_000_000)
        with self.assertRaisesRegex(ValueError, "future"):
            self.mapper.compute(current, self.robot, now_monotonic_ns=T0)


class InterpolationTests(unittest.TestCase):
    def setUp(self):
        self.initial = {side: Pose(f"{side}_base", f"{side}_flange", (0, 0, 0), (0, 0, 0, 1)) for side in SIDES}
        self.interpolator = PoseGoalInterpolator(self.initial, nominal_period_s=.01)
        self.interpolator.set_goal(target(self.initial, sequence=0))

    def poses(self, x, degrees=0):
        return {side: replace(pose, position_m=(x, 0, 0), orientation_xyzw=axis_quat("z", degrees))
                for side, pose in self.initial.items()}

    def test_shortest_slerp_crosses_180_degrees_and_handles_opposite_sign(self):
        a, b = axis_quat("z", 170), axis_quat("z", -170)
        middle = slerp(a, b, .5)
        self.assertAlmostEqual(orientation_distance(a, middle), math.radians(10))
        self.assertAlmostEqual(abs(middle[2]), 1.0)
        opposite = tuple(-x for x in a)
        self.assertAlmostEqual(orientation_distance(slerp(a, opposite, .8), a), 0)

    def test_position_and_orientation_follow_source_time_one_period_later(self):
        self.interpolator.set_goal(target(self.poses(.05, 20), created_ns=T0+10_000_000))
        for elapsed, fraction in [(10, 0.), (15, .5), (20, 1.)]:
            candidate = self.interpolator.sample(T0+elapsed*1_000_000)
            for side in SIDES:
                pose = candidate.tool_poses[side]
                self.assertAlmostEqual(pose.position_m[0], .05*fraction)
                self.assertAlmostEqual(orientation_distance(self.initial[side].orientation_xyzw,
                                                            pose.orientation_xyzw), math.radians(20)*fraction)
            self.interpolator.accept(candidate)

    def test_rejection_preserves_last_accepted_pose_without_stalling_the_clock(self):
        self.interpolator.set_goal(target(self.poses(.05), created_ns=T0+10_000_000))
        rejected = self.interpolator.sample(T0+15_000_000)
        later = self.interpolator.sample(T0+50_000_000)
        self.assertEqual(self.interpolator.accepted_poses, self.initial)
        self.assertAlmostEqual(later.tool_poses["left"].position_m[0], .05)
        with self.assertRaises(ValueError):
            self.interpolator.accept(rejected)
        self.interpolator.accept(later)
        self.assertEqual(self.interpolator.accepted_poses, later.tool_poses)

    def test_engagement_anchor_is_held_until_the_delayed_timeline_advances(self):
        first = self.interpolator.sample(T0)
        self.assertEqual(first.tool_poses, self.initial)
        self.interpolator.accept(first)
        self.interpolator.set_goal(target(self.poses(.03), created_ns=T0+5_000_000))
        self.assertEqual(self.interpolator.sample(T0+5_000_000).tool_poses, self.initial)
        self.assertAlmostEqual(self.interpolator.sample(T0+15_000_000).tool_poses["left"].position_m[0], .03)

    def test_batch_uses_query_times_not_equal_host_receipt_times(self):
        # All three frames were processed together at 30 ms. The first segment
        # must remain available; using only the newest pose loses this bend.
        for sequence, (sample_ms, x) in enumerate([(10, .01), (20, -.02), (30, .03)], 1):
            self.interpolator.set_goal(target(self.poses(x), sequence=sequence, created_ns=T0+30_000_000),
                                       sample_time_ns=T0+sample_ms*1_000_000)
        result = self.interpolator.sample(T0+35_000_000)
        self.assertAlmostEqual(result.tool_poses["left"].position_m[0], .005)
        self.assertEqual({ref.sequence for ref in result.source_refs}, {2, 3})
        self.assertEqual(result.created_monotonic_ns, T0+35_000_000)

    def test_equal_sample_time_replaces_point_and_backward_time_is_rejected(self):
        for x in (.01, .02):
            self.interpolator.set_goal(target(self.poses(x), created_ns=T0+10_000_000))
        result = self.interpolator.sample(T0+15_000_000)
        self.assertAlmostEqual(result.tool_poses["left"].position_m[0], .01)
        with self.assertRaisesRegex(ValueError, "Source sample time moved backwards"):
            self.interpolator.set_goal(target(self.poses(.03), created_ns=T0+15_000_000), sample_time_ns=T0+1)

    def test_projected_source_time_can_be_future_while_command_time_is_current(self):
        self.interpolator.set_goal(target(self.poses(.01), created_ns=T0+5_000_000),
                                   sample_time_ns=T0+15_000_000)
        result = self.interpolator.sample(T0+15_000_000)
        self.assertAlmostEqual(result.tool_poses["left"].position_m[0], .01/3)
        self.assertEqual(result.created_monotonic_ns, T0+15_000_000)
        self.assertEqual({ref.sequence for ref in result.source_refs}, {0, 1})

    def test_same_goal_samples_do_not_restart_a_segment(self):
        for elapsed in (10, 20):
            self.interpolator.set_goal(target(self.poses(.01), sequence=elapsed,
                created_ns=T0+elapsed*1_000_000))
        self.assertAlmostEqual(self.interpolator.sample(T0+20_000_000).tool_poses["left"].position_m[0], .01)
        self.assertAlmostEqual(self.interpolator.sample(T0+25_000_000).tool_poses["left"].position_m[0], .01)

    def test_source_deadline_is_never_extended_and_end_does_not_extrapolate(self):
        self.interpolator.set_goal(target(self.poses(.001), created_ns=T0+10_000_000))
        for elapsed in range(10, 100, 5):
            candidate = self.interpolator.sample(T0+elapsed*1_000_000)
            self.assertEqual(candidate.expires_monotonic_ns,
                             min(T0+100_000_000, candidate.created_monotonic_ns+50_000_000))
            self.interpolator.accept(candidate)
        self.assertAlmostEqual(candidate.tool_poses["left"].position_m[0], .001)
        with self.assertRaisesRegex(ValueError, "source deadline expired"):
            self.interpolator.sample(T0+100_000_000)

    def test_both_interpolation_sources_keep_their_original_deadline(self):
        self.interpolator.set_goal(target(self.poses(.01), created_ns=T0+5_000_000,
                                          expires_ns=T0+105_000_000), sample_time_ns=T0+200_000_000)
        result = self.interpolator.sample(T0+99_000_000)
        self.assertEqual(result.expires_monotonic_ns, T0+100_000_000)
        with self.assertRaisesRegex(ValueError, "source deadline expired"):
            self.interpolator.sample(T0+100_000_000)

    def test_snapshot_and_commit_reject_mutated_candidate(self):
        poses = self.poses(.01)
        self.interpolator.set_goal(target(poses, created_ns=T0+10_000_000))
        poses["left"] = replace(poses["left"], position_m=(9, 0, 0))
        candidate = self.interpolator.sample(T0+15_000_000)
        self.assertAlmostEqual(candidate.tool_poses["left"].position_m[0], .005)
        candidate.tool_poses["left"] = replace(candidate.tool_poses["left"], position_m=(9, 0, 0))
        with self.assertRaises(ValueError):
            self.interpolator.accept(candidate)

    def test_72_90_120_hz_constant_velocity_has_no_200_hz_phase_ripple(self):
        for hz in (72, 90, 120):
            with self.subTest(hz=hz):
                interpolator = PoseGoalInterpolator(self.initial, nominal_period_s=1/hz)
                filter_ = PoseGoalFilter(self.initial)
                next_frame = 0
                output = []
                for tick in range(601):
                    elapsed_ns = tick*5_000_000
                    while round(next_frame*1e9/hz) <= elapsed_ns:
                        query_ns = round(next_frame*1e9/hz)
                        goal = target(self.poses(.1*query_ns/1e9), sequence=next_frame,
                                      created_ns=T0+elapsed_ns, expires_ns=T0+elapsed_ns+100_000_000)
                        interpolator.set_goal(filter_.update(goal), sample_time_ns=T0+query_ns)
                        next_frame += 1
                    result = interpolator.sample(T0+elapsed_ns)
                    interpolator.accept(result)
                    output.append(result.tool_poses["left"].position_m[0])
                velocities = [(b-a)/.005 for a, b in zip(output[400:], output[401:])]
                self.assertLess(max(abs(v-.1) for v in velocities), 1e-8)


if __name__ == "__main__":
    unittest.main()
