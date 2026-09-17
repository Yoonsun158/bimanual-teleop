"""Startup confirmation must precede every motion-capable device connection."""

from contextlib import ExitStack, redirect_stderr, redirect_stdout
import io
import unittest
from unittest.mock import Mock, patch

from bimanual_teleop.common import terminal
from bimanual_teleop.cli import home_tianji as ready, teleop_quest_tianji as quest
from bimanual_teleop.cli import teleop_wuji_hand2 as hands
from bimanual_teleop.control.arm import jog
from bimanual_teleop.control.hand import home, follow


class ConfirmationTests(unittest.TestCase):
    def confirm(self, *keys):
        source = Mock(read=Mock(side_effect=keys))
        with redirect_stderr(io.StringIO()) as output:
            result = terminal.confirm_motion(source, "开始回位")
        return result, output.getvalue(), source

    def test_requires_fresh_enter_after_warning(self):
        result, output, source = self.confirm("\n", None, "w", " ", None, "\n")
        self.assertTrue(result)
        self.assertIn("周围无人员、障碍物", output)
        self.assertIn("按回车键开始回位", output)
        self.assertEqual(source.read.call_count, 6)

    def test_return_key_confirms(self):
        self.assertTrue(self.confirm(None, "\r")[0])

    def test_cancel_and_eof_never_confirm_even_with_enter(self):
        for key in ("q", "Q", "\x03", "\x04", "", "\nq", "q\n"):
            with self.subTest(key=key):
                self.assertFalse(self.confirm(None, key)[0])
                self.assertFalse(self.confirm(key)[0])

    def test_keyboard_interrupt_propagates_without_confirmation(self):
        with self.assertRaises(KeyboardInterrupt):
            self.confirm(None, KeyboardInterrupt())


class MotionEntryTests(unittest.TestCase):
    def entries(self):
        return (
            (ready, lambda: ready.run(ready.parser().parse_args([]))),
            (quest, lambda: quest.main(["--arms-only"])),
            (quest, lambda: quest.main([])),
            (hands, lambda: hands.main([])),
            (jog, lambda: jog.main(["--side", "left"])),
            (home, lambda: home.main(["--side", "both"])),
        )

    def prevent_devices(self, stack):
        guards = []
        for module, name in ((ready, "TianjiDriver"), (quest, "prepare_initial_pose"),
                             (quest, "create_runtime"), (follow, "create_wuji_teleop"),
                             (jog, "prepare_initial_pose"), (jog, "TianjiKinematics"),
                             (jog, "TianjiDriver"), (home, "WujiHandDriver")):
            guards.append(stack.enter_context(patch.object(module, name)))
        stack.enter_context(patch.object(follow, "preflight"))
        return guards

    def test_cancellation_prevents_preparation_and_device_creation(self):
        for module, invoke in self.entries():
            for key in ("q", "Q", "\x04", ""):
                with self.subTest(module=module.__name__, key=key), ExitStack() as stack:
                    guards = self.prevent_devices(stack)
                    stack.enter_context(patch.object(module, "configure_runtime_logging"))
                    factory = stack.enter_context(patch.object(module, "NonblockingTerminal"))
                    source = factory.return_value.__enter__.return_value
                    source.read.side_effect = [None, key]
                    output = stack.enter_context(redirect_stderr(io.StringIO()))
                    self.assertEqual(invoke(), 0)
                    self.assertIn("已取消启动", output.getvalue())
                    factory.return_value.__exit__.assert_called_once()
                    for guard in guards:
                        guard.assert_not_called()

    def test_noninteractive_input_prevents_all_motion(self):
        for module, invoke in self.entries():
            with self.subTest(module=module.__name__), ExitStack() as stack:
                guards = self.prevent_devices(stack)
                stack.enter_context(patch.object(module, "configure_runtime_logging"))
                stack.enter_context(patch.object(terminal.sys, "stdin", io.StringIO("\n")))
                output = stack.enter_context(redirect_stderr(io.StringIO()))
                self.assertEqual(invoke(), 1)
                self.assertIn("交互终端", output.getvalue())
                for guard in guards:
                    guard.assert_not_called()

    def test_confirmed_initial_pose_worker_does_not_prompt_again(self):
        with patch.object(ready, "confirm_motion") as confirm, \
                patch.object(ready, "NonblockingTerminal") as tty, \
                patch.object(ready, "TianjiDriver") as factory, \
                redirect_stderr(io.StringIO()):
            args = ready.parser().parse_args(["--side", "left"])
            self.assertEqual(ready.run(args, confirmed=True), 0)
        confirm.assert_not_called()
        tty.assert_not_called()
        factory.return_value.move_joints.assert_called_once()
        factory.return_value.close.assert_called_once()

    def test_all_command_help_omits_removed_options(self):
        from bimanual_teleop.cli import calibrate_wuji_glove, view_quest, view_wuji_glove
        from scripts import read_tianji_right_force

        entries = (lambda args: ready.parser().parse_args(args), quest.main, hands.main,
                   jog.main, home.main, calibrate_wuji_glove.main, view_quest.main,
                   view_wuji_glove.main, read_tianji_right_force.main)
        removed = ("--enable-motion", "--execute", "--user-id", "--sdk-user-id")
        for entry in entries:
            with self.subTest(entry=entry), redirect_stdout(io.StringIO()) as output:
                with self.assertRaises(SystemExit) as result:
                    entry(["--help"])
                self.assertEqual(result.exception.code, 0)
                for flag in removed:
                    self.assertNotIn(flag, output.getvalue())
        for flag in ("--count", "--interval"):
            self.assertNotIn(flag, output.getvalue())


if __name__ == "__main__":
    unittest.main()
