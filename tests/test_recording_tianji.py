"""Recording hooks preserve measured state and accepted targets without hardware."""

from dataclasses import replace
import math
import time
import unittest
from unittest.mock import Mock, call, patch

from bimanual_teleop.control.arm.cartesian import TianjiCartesianExecutor
from bimanual_teleop.devices.tianji import sdk
from bimanual_teleop.devices.tianji.driver import TianjiDriver, classify_feedback, decode_feedback
from bimanual_teleop.devices.tianji.model import TianjiKinematics
from bimanual_teleop.types import RobotTarget
from tests.support.tianji import FakeSDK, Sink, TianjiFixture, packet


class ForceFeedbackTests(unittest.TestCase):
    def test_raw_channels_keep_precision_and_decode_each_sensor_in_si(self):
        dcss = sdk.load_sdk("robot").DCSS()
        values = (12345.625, -20000., 30000., 4000., -5000., 6000.)
        for index, tag in enumerate((116, 216)):
            output = dcss.m_Out[index]
            output.m_OutFrameSerial = index + 10
            output.m_EST_Joint_Firc[0] = tag
            output.m_EST_Joint_Firc_Dot[:] = [value * (-1 if index else 1) for value in values] + [999.]
        snapshot = sdk.FeedbackSnapshot.from_dcss(dcss, 123, 4)
        self.assertEqual(snapshot.force_tag, [116., 216.])
        self.assertEqual(snapshot.wrench_raw, list(values) + [-value for value in values])
        sample = decode_feedback(snapshot, "recording-test")
        self.assertEqual(sample.header.received_monotonic_ns, 123)
        for index, side in enumerate(("left", "right")):
            state = sample.payload.arms[side]
            self.assertEqual(state.source_sequence, index + 10)
            self.assertEqual(state.force_tag, (116, 216)[index])
            self.assertEqual(state.wrench, tuple(value / 10000. * (-1 if index else 1) for value in values))

    def test_bad_channel_or_nonfinite_force_does_not_invalidate_motion(self):
        for bad_tag, bad_value in ((216, 10000.), (116.1, 10000.), (math.nan, 10000.),
                                   (math.inf, 10000.), (116, math.nan), (116, math.inf)):
            with self.subTest(tag=bad_tag, value=bad_value):
                snapshot = packet(1)
                snapshot.force_tag[:] = [bad_tag, 216]
                snapshot.wrench_raw[:] = [bad_value] + [10000.] * 11
                sample = decode_feedback(snapshot, "recording-test")
                self.assertIsNone(sample.payload.arms["left"].wrench)
                self.assertEqual(sample.payload.arms["right"].wrench, (1.,) * 6)
                self.assertTrue(sample.header.valid)
                assessment = classify_feedback(sample)
                self.assertEqual(assessment.control_issues, ())
                self.assertEqual(assessment.observation_issues, ())


class ForceSelectionTests(unittest.TestCase):
    def test_force_selection_is_opt_in_and_uses_the_existing_connection(self):
        for enabled in (False, True):
            with self.subTest(enabled=enabled):
                driver = TianjiDriver("192.0.2.1", **({"record_force": True} if enabled else {}))
                native = FakeSDK(driver)
                native.robot = Mock()
                try:
                    with patch("bimanual_teleop.devices.tianji.driver.ControlSDK", return_value=native) as factory:
                        driver.start(Sink())
                    factory.assert_called_once()
                    expected = [call("A", 116), call("B", 216)] if enabled else []
                    self.assertEqual(native.robot.set_user_specified_data.call_args_list, expected)
                    self.assertEqual([name for name, _ in native.calls], ["open"])
                finally:
                    driver.close()
                self.assertEqual([name for name, _ in native.calls], ["open", "close"])

    def test_force_selection_rejection_closes_connection_before_workers_start(self):
        for results, arm in (([False], "A"), ([True, False], "B")):
            with self.subTest(arm=arm):
                driver = TianjiDriver("192.0.2.1", record_force=True)
                native = FakeSDK(driver)
                native.robot = Mock()
                native.robot.set_user_specified_data.side_effect = results
                with patch("bimanual_teleop.devices.tianji.driver.ControlSDK", return_value=native):
                    with self.assertRaisesRegex(RuntimeError, f"arm {arm} six-axis force"):
                        driver.start()
                self.assertIsNone(driver._sdk)
                self.assertIsNone(driver._receiver)
                self.assertEqual([name for name, _ in native.calls], ["open", "close"])


class SubmittedCommandTests(unittest.TestCase, TianjiFixture):
    def setUp(self):
        TianjiFixture.__init__(self)
        self.addCleanup(self.driver.close)

    def test_requested_pose_and_constrained_pose_are_distinct_in_success_event(self):
        self.configure(("left", "right"))
        kine = TianjiKinematics()
        executor = TianjiCartesianExecutor(self.driver, kine)
        executor.engage()
        self.assertEqual(self.events("tianji_command_submitted"), [])
        now = time.monotonic_ns()
        poses = {side: replace(pose, position_m=(pose.position_m[0] + .05, *pose.position_m[1:]))
                 for side, pose in executor.engagement_poses.items()}
        target = RobotTarget("recorded", poses, (), now, now + 50_000_000, "test")
        with patch("bimanual_teleop.devices.tianji.driver.time.monotonic_ns", return_value=now), \
                patch.object(self.native, "submit", return_value=now + 100):
            result = executor.submit(target)
        self.assertTrue(result.accepted, result.reason)
        event, = self.events("tianji_command_submitted")
        self.assertEqual(event.observed_monotonic_ns, now + 100)
        self.assertEqual(event.source, "tianji")
        command = event.details["command"]
        self.assertEqual(command.command_id, target.command_id)
        self.assertEqual(command.payload.requested_cartesian_targets, poses)
        self.assertEqual(command.payload.cartesian_targets, executor.applied_poses)
        for side in ("left", "right"):
            self.assertNotEqual(command.payload.cartesian_targets[side], poses[side])
            self.assertEqual(command.payload.cartesian_targets[side], kine.fk(side, command.payload.targets[side]))

    def test_sdk_rejection_does_not_record_a_successful_target(self):
        self.engage()
        self.native.send_submit = False
        result = self.driver.submit(self.command())
        self.assertFalse(result.accepted)
        self.assertEqual(self.events("tianji_command_submitted"), [])

    def test_success_is_retained_when_a_later_control_guard_pauses(self):
        self.engage()
        command = self.command()
        self.assertTrue(self.driver.submit(command).accepted)
        self.driver.request_hold("tracking failed after SDK submission")
        event, = self.events("tianji_command_submitted")
        self.assertIs(event.details["command"], command)
        self.assertFalse(self.driver.engaged)


if __name__ == "__main__":
    unittest.main()
