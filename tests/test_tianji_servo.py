"""Real SDK regression coverage for constrained Cartesian following.

All motion is numerical. No driver, device discovery, or network is involved.
The seven recorded targets are independent engagements, not one trajectory.
"""

from dataclasses import replace
import json
import math
from pathlib import Path
import random
import unittest
from unittest.mock import patch

import numpy as np

from tests.support.geometry import orientation_distance
from bimanual_teleop.control.arm.mapping import (
    _from_matrix, matmul, rotation_matrix,
)
from bimanual_teleop.control.arm.servo import CartesianServo
from bimanual_teleop.devices.tianji.model import KinematicsError, TianjiKinematics


ROOT = Path(__file__).resolve().parents[1]
READY_DEG = {"left": (30, -60, -34, -52, 30, 12, 4),
             "right": (-30, -60, 34, -52, -30, 12, -4)}
DT = (.005, .010, .020)


def radians(values):
    return tuple(math.radians(value) for value in values)


def rotated(pose, rotvec):
    angle = math.sqrt(sum(value * value for value in rotvec))
    quaternion = (tuple(value / angle * math.sin(angle / 2) for value in rotvec)
                  + (math.cos(angle / 2),)) if angle else (0., 0., 0., 1.)
    return replace(pose, orientation_xyzw=_from_matrix(matmul(
        rotation_matrix(quaternion), rotation_matrix(pose.orientation_xyzw))))


class TianjiServoTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kine = TianjiKinematics()
        evidence = json.loads((ROOT / "docs/teleop_ik_20260915_1511_evidence.json").read_text())
        cls.records = [record["input"] for record in evidence["records"]]
        if len(cls.records) != 7:
            raise AssertionError("Expected all seven independently recorded IK failures")

    def servo(self, side, q, velocity_ratio=100, acceleration_ratio=100):
        servo = CartesianServo(self.kine, side, self.kine.model,
                               velocity_ratio=velocity_ratio, acceleration_ratio=acceleration_ratio)
        servo.reset(q)
        return servo

    def assert_positions(self, side, q):
        arm = self.kine.model.arm(side)
        for joint, (value, low, high) in enumerate(zip(q, arm.lower_rad, arm.upper_rad), 1):
            self.assertGreaterEqual(value, low - 1e-8, f"J{joint} lower position limit")
            self.assertLessEqual(value, high + 1e-8, f"J{joint} upper position limit")
        # Independently encode all four facets of the pinned M6 wrist envelope;
        # do not call the controller's own constraint-construction helper.
        for sign6 in (-1, 1):
            for sign7 in (-1, 1):
                self.assertLessEqual(sign6 * 1.025 * q[5] + sign7 * q[6],
                                     math.radians(110.5) + 1e-8, "J6/J7 coupled limit")

    def advance(self, servo, side, target, dt, velocity_ratio=100, acceleration_ratio=100):
        previous, velocity = np.array(servo.q), np.array(servo.velocity)
        step = servo.propose(target, dt, actual_rad=tuple(previous))
        # Planning either arm must remain reversible until group submission.
        np.testing.assert_array_equal(servo.q, previous)
        np.testing.assert_array_equal(servo.velocity, velocity)
        q, following_velocity = np.array(step.joints), np.array(step.velocity)
        self.assertEqual(q.shape, (7,))
        self.assertTrue(np.isfinite(q).all())
        self.assertTrue(np.isfinite(following_velocity).all())
        self.assert_positions(side, q)
        model = self.kine.model.arm(side)
        maximum_velocity = np.array(model.max_joint_velocity_rad_s) * velocity_ratio / 100
        maximum_acceleration = np.radians([row[3] for row in model.limits_native]) * acceleration_ratio / 100
        np.testing.assert_allclose((q - previous) / dt, following_velocity, atol=1e-8, rtol=1e-7)
        self.assertTrue(np.all(np.abs(following_velocity) <= maximum_velocity + 1e-7), "velocity limit")
        self.assertTrue(np.all(np.abs(following_velocity - velocity) <= maximum_acceleration * dt + 1e-7),
                        "acceleration limit")
        actual = self.kine.fk(side, tuple(q))
        self.assertLess(math.dist(actual.position_m, step.pose.position_m), 1e-6)
        self.assertLess(orientation_distance(actual.orientation_xyzw, step.pose.orientation_xyzw), 1e-6)
        self.assertAlmostEqual(step.position_error_m, math.dist(actual.position_m, target.position_m), delta=1e-7)
        self.assertAlmostEqual(step.orientation_error_rad,
                               orientation_distance(actual.orientation_xyzw, target.orientation_xyzw), delta=1e-6)
        servo.accept(step)
        np.testing.assert_array_equal(servo.q, q)
        np.testing.assert_array_equal(servo.velocity, following_velocity)
        return step

    def settle(self, servo, side, target, duration=3., **ratios):
        elapsed, index = 0., 0
        while elapsed < duration:
            dt = DT[index % len(DT)]
            step = self.advance(servo, side, target, dt, **ratios)
            elapsed += dt
            index += 1
        return step

    def assert_tracking(self, step, position=.0002, orientation_deg=.1):
        self.assertLess(step.position_error_m, position)
        self.assertLess(step.orientation_error_rad, math.radians(orientation_deg))

    def test_zero_elapsed_time_preserves_motion_state(self):
        for side in ("left", "right"):
            q = radians(READY_DEG[side])
            servo = self.servo(side, q)
            origin = self.kine.fk(side, q)
            goal = replace(origin, position_m=(origin.position_m[0] + .01, *origin.position_m[1:]))
            self.advance(servo, side, goal, .005)
            previous, velocity = servo.q, servo.velocity
            self.assertGreater(np.linalg.norm(velocity), 0.)
            step = servo.propose(goal, 0.)
            np.testing.assert_array_equal(step.joints, previous)
            np.testing.assert_array_equal(step.velocity, velocity)
            servo.accept(step)
            np.testing.assert_array_equal(servo.q, previous)
            np.testing.assert_array_equal(servo.velocity, velocity)

    def test_unsubmitted_replaced_and_cross_arm_proposals_cannot_corrupt_state(self):
        side, q = "left", radians(READY_DEG["left"])
        servo = self.servo(side, q)
        goal = self.kine.fk(side, q)
        first, other = servo.propose(goal, .005), servo.propose(goal, .010)
        servo.accept(first)
        for stale in (first, other):
            with self.assertRaises(ValueError):
                servo.accept(stale)
            np.testing.assert_array_equal(servo.q, first.joints)
        pending = servo.propose(goal, .005)
        servo.reset(q)
        with self.assertRaises(ValueError):
            servo.accept(pending)
        other_arm = self.servo("right", radians(READY_DEG["right"]))
        with self.assertRaises(ValueError):
            other_arm.accept(servo.propose(goal, .005))
        np.testing.assert_array_equal(other_arm.q, radians(READY_DEG["right"]))

    def test_invalid_numbers_and_shapes_are_rejected_without_changing_state(self):
        q = radians(READY_DEG["left"])
        servo = self.servo("left", q)
        goal = self.kine.fk("left", q)
        for dt in (-.001, math.nan, math.inf):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                servo.propose(goal, dt)
        invalid_goals = (
            replace(goal, position_m=(math.nan, 0., 0.)),
            replace(goal, position_m=(math.inf, 0., 0.)),
            replace(goal, orientation_xyzw=(0., 0., math.nan, 1.)),
            replace(goal, orientation_xyzw=(0., 0., 0., 0.)),
            replace(goal, parent_frame="unrelated_base"),
        )
        for bad in invalid_goals:
            with self.subTest(goal=bad), self.assertRaises(ValueError):
                servo.propose(bad, .005)
        for bad in ((0.,) * 6, (math.nan,) + q[1:], (math.inf,) + q[1:]):
            with self.subTest(joints=bad):
                with self.assertRaises(ValueError):
                    servo.propose(goal, .005, actual_rad=bad)
                with self.assertRaises(ValueError):
                    servo.reset(bad)
        np.testing.assert_array_equal(servo.q, q)
        np.testing.assert_array_equal(servo.velocity, np.zeros(7))

    def test_one_nanosecond_budget_cannot_create_a_joint_velocity_jump(self):
        for side in ("left", "right"):
            q = radians(READY_DEG[side])
            servo = self.servo(side, q)
            origin = self.kine.fk(side, q)
            goal = replace(origin, position_m=(origin.position_m[0] + .01, *origin.position_m[1:]))
            arm = self.kine.model.arm(side)
            amax = np.radians([row[3] for row in arm.limits_native])
            for moving in (False, True):
                if moving:
                    self.advance(servo, side, goal, .005)
                previous, velocity = np.array(servo.q), np.array(servo.velocity)
                dt = 1e-9
                step = servo.propose(goal, dt)
                self.assert_positions(side, step.joints)
                self.assertTrue(np.all(np.abs(np.array(step.velocity) - velocity) <= amax * dt + 1e-8))
                # q subtraction at 1 ns loses precision; compare position
                # integration directly instead of dividing rounding error by dt.
                np.testing.assert_allclose(step.joints, previous + dt * np.array(step.velocity),
                                           atol=2e-15, rtol=0)
                servo.accept(step)

    def test_random_elapsed_times_and_abrupt_reversals_keep_a_legal_braking_path(self):
        configurations = [(side, radians(READY_DEG[side])) for side in ("left", "right")]
        configurations += [(record["side"], radians(record["reference_deg"])) for record in self.records[:2]]
        for case_index, (side, q) in enumerate(configurations):
            with self.subTest(side=side, q=q):
                rng = random.Random(46321 + case_index)
                servo = self.servo(side, q, velocity_ratio=50, acceleration_ratio=40)
                origin = self.kine.fk(side, q)
                arm = self.kine.model.arm(side)
                vmax = np.array(arm.max_joint_velocity_rad_s) * .5
                amax = np.radians([row[3] for row in arm.limits_native]) * .4
                braking_rate = min(amax / vmax)
                for index in range(280):
                    if index % 40 == 0:
                        goal = rotated(origin, tuple(math.radians(rng.uniform(-6., 6.)) for _ in range(3)))
                        goal = replace(goal, position_m=tuple(value + rng.uniform(-.02, .02)
                                                              for value in origin.position_m))
                    dt = rng.uniform(.001, .020)
                    step = self.advance(servo, side, goal, dt, velocity_ratio=50, acceleration_ratio=40)
                    # Exponentially braking each joint with the same rate has
                    # total remaining travel v/k. Its entire path must fit in
                    # the convex joint/wrist envelope after every accepted step.
                    self.assert_positions(side, np.array(step.joints) + np.array(step.velocity) / braking_rate)
                step = self.settle(servo, side, origin, duration=3., velocity_ratio=50, acceleration_ratio=40)
                self.assert_tracking(step, position=.0005, orientation_deg=.2)

    def test_six_recorded_feasible_targets_track_without_rejecting_other_joint_solutions(self):
        from bimanual_teleop.types import Pose
        for index, record in enumerate(self.records[1:], 2):
            with self.subTest(snapshot=index):
                side, q = record["side"], radians(record["reference_deg"])
                goal = Pose(**record["target"])
                initial = self.kine.fk(side, q)
                servo = self.servo(side, q)
                step = self.settle(servo, side, goal)
                initial_position = math.dist(initial.position_m, goal.position_m)
                initial_orientation = orientation_distance(initial.orientation_xyzw, goal.orientation_xyzw)
                self.assert_tracking(step, position=min(.0002, max(.00005, initial_position * .25)),
                                     orientation_deg=min(.1, max(.02, math.degrees(initial_orientation) * .25)))

    def test_recorded_j4_boundary_stays_legal_then_returns_to_reachable_target(self):
        from bimanual_teleop.types import Pose
        record = self.records[0]
        side, q = record["side"], radians(record["reference_deg"])
        servo = self.servo(side, q)
        origin = self.kine.fk(side, q)
        goal = Pose(**record["target"])
        limited = False
        for index in range(240):
            step = self.advance(servo, side, goal, DT[index % 3])
            limited |= bool(step.limited)
        self.assertTrue(limited, "A geometrically impossible target must be reported as limited")
        step = self.settle(servo, side, origin)
        self.assert_tracking(step)

    def test_stationary_targets_including_near_limits_do_not_drive_nullspace_motion(self):
        for side in ("left", "right"):
            configurations = [radians(READY_DEG[side]),
                              radians(self.records[0]["reference_deg"]),
                              radians(self.records[1]["reference_deg"]),
                              radians(self.records[-1]["reference_deg"])]
            for q in configurations:
                with self.subTest(side=side, q=q):
                    servo = self.servo(side, q)
                    goal = self.kine.fk(side, q)
                    for index in range(90):
                        step = self.advance(servo, side, goal, DT[index % 3])
                        np.testing.assert_allclose(step.joints, q, atol=1e-10, rtol=0)
                        np.testing.assert_allclose(step.velocity, 0, atol=1e-10, rtol=0)

    def test_pushing_beyond_j4_boundary_does_not_slide_sideways_or_rotate_wrist(self):
        record = self.records[0]
        side, q = record["side"], radians(record["reference_deg"])
        servo = self.servo(side, q)
        origin = self.kine.fk(side, q)
        direction = np.asarray(record["target"]["position_m"]) - origin.position_m
        direction /= np.linalg.norm(direction)
        goal = replace(origin, position_m=tuple(np.asarray(origin.position_m) + .05 * direction))
        limited = False
        for index in range(240):
            step = self.advance(servo, side, goal, DT[index % 3])
            displacement = np.asarray(step.pose.position_m) - origin.position_m
            perpendicular = displacement - direction * np.dot(displacement, direction)
            self.assertLess(np.linalg.norm(perpendicular), .001, "unrequested sideways motion at joint boundary")
            self.assertLess(orientation_distance(step.pose.orientation_xyzw, origin.orientation_xyzw),
                            math.radians(.2), "unrequested wrist rotation at joint boundary")
            limited |= bool(step.limited)
        self.assertTrue(limited)
        self.assert_tracking(self.settle(servo, side, origin))

    def test_small_pose_noise_near_limits_does_not_accumulate_large_posture_drift(self):
        for record in (self.records[1], self.records[2], self.records[-1]):
            with self.subTest(reference=record["reference_deg"]):
                side, q = record["side"], radians(record["reference_deg"])
                servo = self.servo(side, q)
                origin = self.kine.fk(side, q)
                for index in range(240):
                    sign = -1 if index % 2 else 1
                    goal = rotated(origin, (sign * math.radians(.02), 0., 0.))
                    goal = replace(goal, position_m=(origin.position_m[0] + sign * .00005,
                                                     *origin.position_m[1:]))
                    step = self.advance(servo, side, goal, .005)
                    self.assertLess(max(abs(a - b) for a, b in zip(step.joints, q)), math.radians(2))
                step = self.settle(servo, side, origin)
                self.assert_tracking(step)

    def test_far_unreachable_goals_and_reverse_motion_preserve_all_constraints(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                q = radians(READY_DEG[side])
                servo = self.servo(side, q, velocity_ratio=50, acceleration_ratio=50)
                origin = self.kine.fk(side, q)
                goal = replace(origin, position_m=(1.5, .2, .4))
                limited = False
                for index in range(240):
                    step = self.advance(servo, side, goal, DT[index % 3],
                                        velocity_ratio=50, acceleration_ratio=50)
                    limited |= bool(step.limited)
                self.assertTrue(limited)
                step = self.settle(servo, side, origin, duration=5., velocity_ratio=50, acceleration_ratio=50)
                self.assert_tracking(step, position=.0005, orientation_deg=.2)

    def test_each_wrist_coupling_face_limits_outward_motion_and_allows_return(self):
        for side in ("left", "right"):
            for sign6 in (-1, 1):
                for sign7 in (-1, 1):
                    with self.subTest(side=side, sign6=sign6, sign7=sign7):
                        joints = list(READY_DEG[side])
                        joints[5], joints[6] = sign6 * 50., sign7 * (110.5 - 1.025 * 50. - .05)
                        q = radians(joints)
                        self.assert_positions(side, q)
                        servo = self.servo(side, q)
                        origin = self.kine.fk(side, q)
                        for index in range(12):
                            stationary = self.advance(servo, side, origin, DT[index % 3])
                            np.testing.assert_allclose(stationary.joints, q, atol=1e-10, rtol=0)
                        # FK can evaluate joints outside the envelope. This
                        # gives an outward wrist request without assuming
                        # that the target's other redundant branches are absent.
                        joints[6] += sign7 * 8.
                        goal = self.kine.fk(side, radians(joints))
                        for index in range(120):
                            self.advance(servo, side, goal, DT[index % 3])
                        self.assert_tracking(self.settle(servo, side, origin))

    def test_repeated_closed_cartesian_loops_return_without_large_redundant_drift(self):
        for side in ("left", "right"):
            q = radians(READY_DEG[side])
            servo = self.servo(side, q)
            origin = self.kine.fk(side, q)
            for loop in range(2):
                for index in range(320):
                    angle = 2 * math.pi * (index + 1) / 320
                    goal = rotated(origin, (math.radians(3) * math.sin(angle),
                                            math.radians(3) * (math.cos(angle) - 1), 0.))
                    goal = replace(goal, position_m=(origin.position_m[0] + .015 * (math.cos(angle) - 1),
                                                     origin.position_m[1] + .015 * math.sin(angle),
                                                     origin.position_m[2] + .005 * math.sin(2 * angle)))
                    # Verify this circle's targets are actually reachable,
                    # rather than allowing errors to be hidden by soft tracking.
                    self.kine.ik(side, goal, q)
                    self.advance(servo, side, goal, .010)
                step = self.settle(servo, side, origin, duration=1.5)
                self.assert_tracking(step)
                self.assertLess(max(abs(a - b) for a, b in zip(servo.q, q)), math.radians(2),
                                f"accumulated posture drift after loop {loop + 1}")

    def test_long_mixed_six_axis_motion_with_varying_dt_and_profile_ratios(self):
        rng = random.Random(20260915)
        frequencies = [rng.uniform(.15, .5) for _ in range(7)]
        phases = [rng.uniform(-math.pi, math.pi) for _ in range(7)]
        amplitudes = radians((8, 6, 10, 8, 8, 5, 5))
        for side in ("left", "right"):
            q = radians(READY_DEG[side])
            servo = self.servo(side, q, velocity_ratio=25, acceleration_ratio=40)
            elapsed = 0.
            for index in range(600):
                dt = DT[index % 3]
                elapsed += dt
                target_q = tuple(value + amplitude * (math.sin(2 * math.pi * frequency * elapsed + phase)
                                                       - math.sin(phase))
                                 for value, amplitude, frequency, phase in zip(q, amplitudes, frequencies, phases))
                self.assert_positions(side, target_q)
                goal = self.kine.fk(side, target_q)
                step = self.advance(servo, side, goal, dt, velocity_ratio=25, acceleration_ratio=40)
                self.assert_tracking(step, position=.015, orientation_deg=3.)
            step = self.settle(servo, side, self.kine.fk(side, q), duration=4.,
                               velocity_ratio=25, acceleration_ratio=40)
            self.assert_tracking(step, position=.0005, orientation_deg=.2)

    def test_numerical_failure_retains_replay_state_without_advancing(self):
        q = radians(READY_DEG["right"])
        servo = self.servo("right", q)
        pose = self.kine.fk("right", q)
        with patch("quadprog.solve_qp", side_effect=ValueError("synthetic solver failure")):
            with self.assertRaises(KinematicsError) as caught:
                servo.propose(pose, .005)
        record = json.loads(str(caught.exception).split("[控制诊断] ", 1)[1])
        self.assertEqual(record["schema"], "tianji_cartesian_servo_failure_v1")
        self.assertEqual(record["q_rad"], list(q))
        self.assertEqual(record["velocity_rad_s"], [0.] * 7)
        self.assertEqual(record["anchor_rad"], list(q))
        self.assertEqual(record["dt_s"], .005)
        self.assertEqual(record["model_sha256"], self.kine.model.digest)
        self.assertIn("synthetic solver failure", record["message"])
        self.assertEqual(servo.q, q)

    def test_exact_wrist_boundary_cannot_initialize_a_float32_safe_command_envelope(self):
        q = list(radians(READY_DEG["right"]))
        q[5], q[6] = math.radians(20.), math.radians(90.)
        servo = CartesianServo(self.kine, "right", self.kine.model)
        with self.assertRaisesRegex(ValueError, "float32-safe engagement"):
            servo.reset(tuple(q))
        self.assertEqual(servo.q, ())
        q[6] -= math.radians(.00002)
        servo.reset(tuple(q))
        self.assertEqual(servo.q, tuple(q))


if __name__ == "__main__":
    unittest.main()
