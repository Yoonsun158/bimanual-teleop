"""Group IK failure and expiry behavior without hardware."""

from dataclasses import replace
import math
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from bimanual_teleop.control.arm.cartesian import TianjiCartesianExecutor
from bimanual_teleop.types import Health, Pose, RobotTarget, SampleRef, Submission


class FakeDriver:
    def __init__(self):
        self.profile = SimpleNamespace(profile_id="test", active_arms=("left", "right"))
        self.engaged = False
        self.commands, self.holds = [], []
        self.accept = True
        self.joints = (0.0,) * 7
        self.engagement_sample = None

    def configure(self, profile):
        pass

    def get_latest(self):
        return SimpleNamespace(header=SimpleNamespace(ref=SampleRef("feedback", "test", 1)),
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
                    (joints[0], 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))

    def ik(self, side, pose, reference):
        self.references.append((side, reference))
        if self.after_ik:
            self.after_ik()
        if side == self.fail_side:
            raise ValueError("unreachable")
        return (pose.position_m[0],) + (0.0,) * 6


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
        return RobotTarget("c1", {s: self.kine.fk(s, (x,) * 7) for s in ("left", "right")},
                           {}, (), self.now, self.now + 50_000_000, "test")

    def test_group_submitted_once_and_next_reference_is_accepted_target(self):
        self.assertTrue(self.executor.submit(self.target()).accepted)
        self.assertEqual(len(self.driver.commands), 1)
        self.assertEqual(set(self.driver.commands[0].payload.targets), {"left", "right"})
        self.now += 5_000_000
        self.assertTrue(self.executor.submit(replace(self.target(0.002), command_id="c2")).accepted)
        self.assertEqual(self.kine.references[-1][1][0], 0.001)

    def test_right_ik_failure_prevents_left_dispatch_and_holds_group(self):
        self.kine.fail_side = "right"
        self.assertFalse(self.executor.submit(self.target()).accepted)
        self.assertEqual(self.driver.commands, [])
        self.assertFalse(self.driver.engaged)
        self.assertIn("unreachable", self.driver.holds[-1])

    def test_reachable_translation_and_rotation_are_not_speed_clipped(self):
        target = self.target(.1)
        poses = {side: replace(pose, orientation_xyzw=(0.0, 0.0, math.sin(0.05), math.cos(0.05)))
                 for side, pose in target.tool_poses.items()}
        result = self.executor.submit(replace(target, tool_poses=poses))
        self.assertTrue(result.accepted, result.reason)
        self.assertEqual(len(self.driver.commands), 1)
        self.assertEqual(self.driver.commands[0].payload.cartesian_targets, poses)
        self.assertEqual(self.driver.holds, [])

    def test_expiry_during_ik_never_reaches_driver(self):
        self.kine.after_ik = lambda: setattr(self, "now", self.now + 30_000_000)
        result = self.executor.submit(self.target())
        self.assertFalse(result.accepted)
        self.assertIn("expired during IK", result.reason)
        self.assertEqual(self.driver.commands, [])

    def test_driver_rejection_requires_new_engagement_at_measured_joints(self):
        self.driver.accept = False
        self.assertFalse(self.executor.submit(self.target()).accepted)
        self.driver.joints = (0.003,) + (0.0,) * 6
        self.executor.engage()
        self.assertEqual(self.kine.references[-1][1], self.driver.joints)

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
