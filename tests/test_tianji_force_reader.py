"""Continuous force output and cleanup with a simulated Tianji SDK."""

from contextlib import redirect_stderr, redirect_stdout
import io
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, call, patch

from bimanual_teleop.cli import read_tianji_force as force
from scripts import read_tianji_force, read_tianji_right_force


def frame(left=0, right=0, *, channels=(116, 216)):
    raw = [10000, -20000, 30000, 4000, -5000, 6000]
    return SimpleNamespace(m_Out=[
        SimpleNamespace(m_OutFrameSerial=sequence, m_EST_Joint_Firc=[channel],
                        m_EST_Joint_Firc_Dot=[sign * value for value in raw])
        for sequence, channel, sign in zip((left, right), channels, (-1, 1))
    ])


class ForceReaderTests(unittest.TestCase):
    def run_reader(self, values, *, args=(), entry=read_tianji_force.main, clock=None, selection=None):
        robot = Mock()
        robot.read.side_effect = values
        robot.robot.set_user_specified_data.side_effect = selection
        with patch.object(force, "ControlSDK", return_value=robot), \
                patch.object(force.time, "sleep"), \
                patch.object(force.time, "monotonic", side_effect=clock, return_value=0.), \
                redirect_stdout(io.StringIO()) as output, redirect_stderr(io.StringIO()) as error:
            code = entry(list(args))
        robot.close.assert_called_once()
        return code, output.getvalue(), error.getvalue(), robot

    def test_defaults_to_both_and_continues_until_interrupted(self):
        code, output, error, robot = self.run_reader([*(frame(i, i) for i in range(1, 13)), KeyboardInterrupt()])
        self.assertEqual(code, 0)
        self.assertEqual(output.count("left frame="), 12)
        self.assertEqual(output.count("right frame="), 12)
        self.assertIn("left frame=1  F[N]=(-1.0000, 2.0000, -3.0000)", output)
        self.assertIn("T[N·m]=(-0.4000, 0.5000, -0.6000)", output)
        self.assertIn("F[N]=(1.0000, -2.0000, 3.0000)", output)
        self.assertIn("T[N·m]=(0.4000, -0.5000, 0.6000)", output)
        self.assertIn("raw=[10000, -20000, 30000, 4000, -5000, 6000]", output)
        self.assertIn("Ctrl+C", error)
        self.assertEqual(robot.robot.set_user_specified_data.call_args_list, [call("A", 116), call("B", 216)])

    def test_single_side_does_not_select_or_read_other_arm(self):
        for side, index, arm, channel in (("left", 0, "A", 116), ("right", 1, "B", 216)):
            with self.subTest(side=side):
                data = frame(1, 1)
                data.m_Out[1 - index] = None
                code, output, _, robot = self.run_reader([data, KeyboardInterrupt()], args=["--side", side])
                self.assertEqual(code, 0)
                self.assertEqual(output.count("frame="), 1)
                self.assertTrue(output.startswith(f"{side} frame=1"))
                robot.robot.set_user_specified_data.assert_called_once_with(arm, channel)

    def test_old_entry_defaults_to_right_and_accepts_side_override(self):
        for args, side, arm, channel in (([], "right", "B", 216), (["--side", "left"], "left", "A", 116)):
            with self.subTest(args=args):
                code, output, _, robot = self.run_reader(
                    [frame(1, 1), KeyboardInterrupt()], args=args, entry=read_tianji_right_force.main)
                self.assertEqual(code, 0)
                self.assertEqual(output.count("frame="), 1)
                self.assertTrue(output.startswith(f"{side} frame=1"))
                robot.robot.set_user_specified_data.assert_called_once_with(arm, channel)

    def test_frame_freshness_is_independent_for_each_arm(self):
        code, output, _, _ = self.run_reader(
            [frame(1, 1), frame(1, 2), frame(2, 2), KeyboardInterrupt()], args=["--side", "both"])
        self.assertEqual(code, 0)
        self.assertEqual([line.split("  ")[0] for line in output.splitlines()],
                         ["left frame=1", "right frame=1", "right frame=2", "left frame=2"])

    def test_zero_frame_and_wrong_channel_timeout_without_output(self):
        for side in ("left", "right"):
            for data in (frame(), frame(1, 1, channels=(216, 116))):
                with self.subTest(side=side, data=data):
                    code, output, error, _ = self.run_reader([data], args=["--side", side], clock=[0., 3.1])
                    self.assertEqual(code, 1)
                    self.assertEqual(output, "")
                    self.assertIn(f"no fresh {side}-arm force data", error)

    def test_healthy_arm_does_not_hide_other_arms_stale_data(self):
        for stale in ("left", "right"):
            with self.subTest(stale=stale):
                values = [frame(1, i) if stale == "left" else frame(i, 1) for i in range(1, 4)]
                code, output, error, _ = self.run_reader(values, clock=[0., 0., 2., 3.1])
                self.assertEqual(code, 1)
                self.assertEqual(output.count(f"{stale} frame="), 1)
                self.assertIn(f"no fresh {stale}-arm force data", error)

    def test_waits_for_matching_channels_before_printing(self):
        code, output, _, _ = self.run_reader(
            [frame(1, 1, channels=(216, 116)), frame(2, 2), KeyboardInterrupt()])
        self.assertEqual(code, 0)
        self.assertNotIn("frame=1", output)
        self.assertEqual(output.count("frame=2"), 2)

    def test_channel_selection_failure_releases_connection(self):
        code, output, error, robot = self.run_reader([], selection=[True, False])
        self.assertEqual(code, 1)
        self.assertEqual(output, "")
        self.assertIn("failed to select right-arm six-axis force data (channel 216)", error)
        robot.read.assert_not_called()

    def test_read_failure_releases_connection(self):
        code, _, error, _ = self.run_reader([RuntimeError("feedback unavailable")])
        self.assertEqual(code, 1)
        self.assertIn("feedback unavailable", error)


if __name__ == "__main__":
    unittest.main()
