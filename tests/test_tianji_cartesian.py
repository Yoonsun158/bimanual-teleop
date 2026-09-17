"""Constrained servo submission, rollback and expiry without hardware."""

from dataclasses import replace
import json
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from bimanual_teleop.control.arm.cartesian import TianjiCartesianExecutor
from bimanual_teleop.devices.tianji.model import KinematicsError, M6Model
from bimanual_teleop.types import Health, Pose, RobotTarget, SampleRef, Submission


class FakeDriver:
    def __init__(self):
        self.profile = SimpleNamespace(profile_id="test", active_arms=("left", "right"),
            model=M6Model.from_file(), arms={side: SimpleNamespace(velocity_ratio=50, acceleration_ratio=50)
                                            for side in ("left", "right")})
        self.engaged = False
        self.commands, self.holds = [], []
        self.accept = True
        self.joints = (0.0,) * 7
        self.engagement_sample = None

    def configure(self, profile):
        pass

    def get_latest(self):
        return SimpleNamespace(header=SimpleNamespace(ref=SampleRef("feedback", "test", 1),
                                                     received_monotonic_ns=1_000_000_000),
                               payload=SimpleNamespace(arms={
            side: SimpleNamespace(joints=SimpleNamespace(position_rad=self.joints))
            for side in ("left", "right")
        }))

    def health(self):
        return Health(True, 0)

    def engage(self):
        self.engagement_sample = self.get_latest()
        self.engaged = True

    def submit(self, command):
        self.commands.append(command)
        return Submission(command.command_id, self.accept, None if self.accept else "native busy")

    def request_hold(self, reason):
        self.holds.append(reason)
        self.engaged = False


class FakeKinematics:
    def __init__(self):
        self.references = []
        self.fail_side = None
        self.after_ik = None

    def fk(self, side, joints):
        return Pose(f"tianji_{side}_base", f"tianji_{side}_flange",
                    (joints[0], 0.0, 0.0), (0.0, 0.0, math.sin(joints[3]/2), math.cos(joints[3]/2)))

    def jacobian(self, side, reference):
        self.references.append((side, reference))
        # Keep the timing hook used by executor expiry/diagnostic tests.
        if self.after_ik:
            self.after_ik()
        if side == self.fail_side:
            raise KinematicsError(f"{side}: synthetic Jacobian failure")
        return tuple(tuple(float((row, column) in ((0, 0), (5, 3))) for column in range(7))
                     for row in range(6))


class CartesianTests(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000
        self.clock = patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns",
                           side_effect=lambda: self.now)
        self.clock.start()
        self.addCleanup(self.clock.stop)
        self.driver, self.kine = FakeDriver(), FakeKinematics()
        self.executor = TianjiCartesianExecutor(self.driver, self.kine)
        self.executor.engage()
        self.now += 5_000_000

    def target(self, x=0.001):
        return RobotTarget("c1", {s: self.kine.fk(s, (x,) + (0.,)*6) for s in ("left", "right")},
                           (), self.now, self.now + 50_000_000, "test")

    def assert_safe_command(self, command, previous, velocity, dt):
        following_velocity = {}
        for side, joints in command.payload.targets.items():
            model, settings = self.driver.profile.model.arm(side), self.driver.profile.arms[side]
            self.assertEqual(len(joints), 7)
            self.assertTrue(all(math.isfinite(value) for value in joints))
            following_velocity[side] = tuple((q-old)/dt for q, old in zip(joints, previous[side]))
            for index, q in enumerate(joints):
                self.assertGreaterEqual(q, model.lower_rad[index]-1e-8)
                self.assertLessEqual(q, model.upper_rad[index]+1e-8)
                vmax = model.max_joint_velocity_rad_s[index]*settings.velocity_ratio/100
                amax = math.radians(model.limits_native[index][3])*settings.acceleration_ratio/100
                self.assertLessEqual(abs(following_velocity[side][index]), vmax+3e-7)
                self.assertLessEqual(abs(following_velocity[side][index]-velocity[side][index]), amax*dt+3e-7)
            for sign6 in (-1, 1):
                for sign7 in (-1, 1):
                    self.assertLessEqual(sign6*1.025*joints[5]+sign7*joints[6], math.radians(110.5)+1e-8)
            self.assertEqual(command.payload.cartesian_targets[side], self.kine.fk(side, joints))
        return following_velocity

    def test_group_submitted_once_and_next_reference_is_accepted_target(self):
        self.assertTrue(self.executor.submit(self.target()).accepted)
        self.assertEqual(len(self.driver.commands), 1)
        command = self.driver.commands[0]
        self.assertEqual(set(command.payload.targets), {"left", "right"})
        zeros = {side: (0.,)*7 for side in ("left", "right")}
        velocity = self.assert_safe_command(command, zeros, zeros, .005)
        self.assertGreater(command.payload.targets["left"][0], 0.)
        self.assertLess(command.payload.targets["left"][0], .001)
        self.now += 5_000_000
        self.assertTrue(self.executor.submit(replace(self.target(0.002), command_id="c2")).accepted)
        self.assertEqual(dict(self.kine.references[-2:]), command.payload.targets)
        self.assert_safe_command(self.driver.commands[-1], command.payload.targets, velocity, .005)

    def test_right_jacobian_failure_discards_successful_left_proposal_and_holds_group(self):
        servos = dict(self.executor._servos)
        previous = {side: (servo.q, servo.velocity) for side, servo in servos.items()}
        applied = self.executor.applied_poses
        self.kine.fail_side = "right"
        with patch.object(servos["left"], "propose", wraps=servos["left"].propose) as left_propose:
            self.assertFalse(self.executor.submit(self.target()).accepted)
            left_propose.assert_called_once()
        self.assertEqual(self.driver.commands, [])
        self.assertFalse(self.driver.engaged)
        self.assertIn("right: synthetic Jacobian failure", self.driver.holds[-1])
        self.assertEqual({side: (servo.q, servo.velocity) for side, servo in servos.items()}, previous)
        self.assertEqual(self.executor.applied_poses, applied)
        self.assertEqual(self.executor._reference, {})

    def test_failure_snapshot_keeps_measured_joints_and_command_context_before_hold(self):
        def fail(side, reference):
            error = KinematicsError("synthetic Jacobian error")
            error.diagnostic = {"side": side, "reference_deg": [math.degrees(q) for q in reference]}
            raise error
        self.kine.jacobian = fail
        self.driver.joints = (0.003,) * 7
        target = replace(self.target(), source_refs=(SampleRef("quest", "run", 5),))
        result = self.executor.submit(target)
        self.assertFalse(result.accepted)
        self.assertEqual(self.driver.commands, [])
        self.assertFalse(self.driver.engaged)
        snapshot = json.loads(result.reason.split("[IK诊断] ", 1)[1])
        self.assertEqual(snapshot["reference_deg"], [0.] * 7)
        self.assertEqual(snapshot["feedback"]["joints_deg"], [math.degrees(.003)] * 7)
        self.assertEqual(snapshot["feedback"]["received_monotonic_ns"], 1_000_000_000)
        self.assertEqual(snapshot["command_id"], target.command_id)
        self.assertEqual(snapshot["source_refs"], [{"stream": "quest", "epoch": "run", "sequence": 5}])
        self.assertEqual(snapshot["ik_started_monotonic_ns"], self.now)
        self.assertEqual(snapshot["expires_monotonic_ns"], target.expires_monotonic_ns)

    def test_translation_and_rotation_progress_with_q_v_a_bounds_and_actual_fk(self):
        target = self.target(.1)
        poses = {side: replace(pose, orientation_xyzw=(0.0, 0.0, math.sin(0.05), math.cos(0.05)))
                 for side, pose in target.tool_poses.items()}
        previous = {side: (0.,)*7 for side in ("left", "right")}
        velocity = dict(previous)
        for index, dt in enumerate((.005, .010, .020)*4):
            if index:
                self.now += round(dt*1e9)
            target = replace(self.target(.1), command_id=f"c{index}", tool_poses=poses)
            result = self.executor.submit(target)
            self.assertTrue(result.accepted, result.reason)
            command = self.driver.commands[-1]
            velocity = self.assert_safe_command(command, previous, velocity, dt)
            previous = command.payload.targets
            self.assertEqual(self.executor.applied_poses, command.payload.cartesian_targets)
            for side in previous:
                self.assertGreater(previous[side][0], 0.)
                self.assertGreater(previous[side][3], 0.)
                self.assertLess(previous[side][0], .1)
                self.assertLess(previous[side][3], .1)
        self.assertEqual(len(self.driver.commands), 12)
        self.assertEqual(self.driver.holds, [])

    def test_expiry_during_jacobian_never_reaches_driver_or_advances_servos(self):
        servos = dict(self.executor._servos)
        previous = {side: (servo.q, servo.velocity) for side, servo in servos.items()}
        self.kine.after_ik = lambda: setattr(self, "now", self.now + 30_000_000)
        result = self.executor.submit(self.target())
        self.assertFalse(result.accepted)
        self.assertIn("expired during IK", result.reason)
        self.assertEqual(self.driver.commands, [])
        self.assertEqual({side: (servo.q, servo.velocity) for side, servo in servos.items()}, previous)

    def test_driver_rejection_does_not_advance_either_servo_and_reengagement_uses_actual_joints(self):
        self.assertTrue(self.executor.submit(self.target()).accepted)
        self.now += 5_000_000
        servos = dict(self.executor._servos)
        previous = {side: (servo.q, servo.velocity) for side, servo in servos.items()}
        applied = self.executor.applied_poses
        self.driver.accept = False
        self.assertFalse(self.executor.submit(self.target()).accepted)
        self.assertEqual(len(self.driver.commands), 2)
        self.assertEqual({side: (servo.q, servo.velocity) for side, servo in servos.items()}, previous)
        self.assertEqual(self.executor.applied_poses, applied)
        self.assertFalse(self.driver.engaged)
        self.assertIn("native busy", self.driver.holds[-1])
        self.driver.joints = (0.003, 0., 0., .002, 0., 0., 0.)
        self.executor.engage()
        for side, servo in self.executor._servos.items():
            self.assertIsNot(servo, servos[side])
            self.assertEqual(servo.q, self.driver.joints)
            self.assertEqual(servo.velocity, (0.,)*7)
            self.assertEqual(self.executor.engagement_poses[side], self.kine.fk(side, self.driver.joints))
        self.assertEqual(dict(self.kine.references[-2:]), dict.fromkeys(("left", "right"), self.driver.joints))
        self.assertIsNone(self.executor._last_step_ns)
        self.assertEqual(self.executor.tracking_status, {})
        self.driver.accept = True
        self.now += 5_000_000
        self.assertTrue(self.executor.submit(self.target(.004)).accepted)
        self.assert_safe_command(self.driver.commands[-1], dict.fromkeys(("left", "right"), self.driver.joints),
                                 dict.fromkeys(("left", "right"), (0.,)*7), .005)

    def test_before_submit_fault_discards_both_proposals(self):
        servos = dict(self.executor._servos)
        previous = {side: (servo.q, servo.velocity) for side, servo in servos.items()}
        def failed_guard():
            raise RuntimeError("tracking lost during servo proposal")
        with patch.object(servos["left"], "propose", wraps=servos["left"].propose) as left_propose, \
                patch.object(servos["right"], "propose", wraps=servos["right"].propose) as right_propose:
            result = self.executor.submit(self.target(), before_submit=failed_guard)
            left_propose.assert_called_once()
            right_propose.assert_called_once()
        self.assertFalse(result.accepted)
        self.assertIn("tracking lost", result.reason)
        self.assertEqual(self.driver.commands, [])
        self.assertEqual({side: (servo.q, servo.velocity) for side, servo in servos.items()}, previous)

    def test_wrong_group_profile_and_expired_targets_rejected(self):
        target = self.target()
        for invalid in (replace(target, tool_poses={"left": target.tool_poses["left"]}),
                        replace(target, control_profile_id="old"),
                        replace(target, expires_monotonic_ns=self.now)):
            self.executor.engage()
            self.assertFalse(self.executor.submit(invalid).accepted)
        self.assertEqual(self.driver.commands, [])

if __name__ == "__main__":
    unittest.main()
