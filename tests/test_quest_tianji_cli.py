"""Keyboard and scheduler checks with fake devices; never load a native SDK."""

from contextlib import redirect_stderr, redirect_stdout
import logging
import io
import os
from pathlib import Path
import sys
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import yaml

from bimanual_teleop.types import ControlProfile, Health
from bimanual_teleop.common.console import (LiveProgress, StatusConsole,
    configure_runtime_logging, format_message, print_message)
from bimanual_teleop.cli import teleop_quest_tianji as cli
from bimanual_teleop.common import terminal as terminal_module
from bimanual_teleop.control.arm import preparation


ROOT = Path(__file__).resolve().parents[1]
PROFILE = ControlProfile("test-profile", "cartesian_impedance", {"active_arms": ["left", "right"]})


class SideSelectionTests(unittest.TestCase):
    def test_profile_selects_only_requested_arm_without_mutating_loaded_profile(self):
        for side in ("left", "right"):
            selected = cli.select_profile_side(PROFILE, side)
            self.assertEqual(selected.parameters["active_arms"], [side])
            self.assertEqual(selected.profile_id, f"test-profile-{side}")
        self.assertEqual(PROFILE.parameters["active_arms"], ["left", "right"])
        self.assertIs(cli.select_profile_side(PROFILE, "both"), PROFILE)


class Runtime:
    def __init__(self):
        self.calls = []
        self.ticks = []

    def status(self, *, include_target=True):
        return {"state": "PAUSED", "last_error": None}

    def tick(self, now):
        self.ticks.append(now)

    def health(self): return Health(True, 0)
    def start(self): self.calls.append(("start",))
    def close(self): self.calls.append(("close",))
    def engage(self, profile): self.calls.append(("engage", profile))
    def pause(self, reason):
        self.calls.append(("pause", reason))


class KeyboardTests(unittest.TestCase):
    def setUp(self):
        self.runtime = Runtime()
        self.messages = []
        self.ui = cli.TeleopUI(self.runtime, PROFILE, enable_motion=True, emit=self.messages.append)

    def test_enter_engages_directly_in_preview_and_motion_modes(self):
        for motion in (False, True):
            self.ui.enable_motion = motion
            self.ui.handle("\n")
            self.assertEqual(self.runtime.calls[-1], ("engage", PROFILE))
            self.ui.handle(" ")
            self.assertEqual(self.runtime.calls[-1][0], "pause")

    def test_removed_calibration_keys_have_no_device_effect(self):
        for key in "hb j1234567rs".replace(" ", ""):
            self.ui.handle(key)
        self.assertEqual(self.runtime.calls, [])

    def test_handled_runtime_pause_is_reported_once_and_survives_keyboard_exit(self):
        self.runtime.state = "paused"
        self.runtime.last_error = "right: IK failed"
        self.ui.report_runtime_pause()
        self.ui.report_runtime_pause()
        self.assertEqual(self.ui.motion_pauses, 1)
        self.assertEqual(self.ui.last_motion_error, "right: IK failed")
        self.assertIn("遥操作已暂停", self.messages[-1])
        self.ui.handle("q")
        self.assertEqual(self.ui.last_motion_error, "right: IK failed")

    def test_runtime_rejection_and_quit_always_request_pause(self):
        self.runtime.engage = Mock(side_effect=ValueError("tracking lost"))
        self.ui.handle("\n")
        self.assertEqual(self.runtime.calls[-1], ("pause", "tracking lost"))
        self.ui.handle("q")
        self.assertTrue(self.ui.quit)
        self.assertEqual(self.runtime.calls[-1], ("pause", "keyboard exit"))

    def test_status_ignores_clock_counters_and_reports_readiness_changes_once(self):
        status = {"state": "ready", "health": {"ready": False, "detail": "left tracking unavailable"}}
        self.ui.report_status(status)
        for index in range(5):
            self.ui.report_status({**status, "cycles": index, "last_compute_ns": index*100,
                                   "scheduler_cycles": index, "skipped_deadlines": index})
        self.assertEqual(len(self.messages), 1)
        status["health"] = {"ready": True, "detail": "ready", "observed_monotonic_ns": 123}
        self.ui.report_status(status)
        self.ui.report_status({**status, "health": {**status["health"], "observed_monotonic_ns": 456}})
        self.assertEqual(len(self.messages), 2)
        self.assertIn("设备已就绪", self.messages[-1])
        status["health"] = {"ready": False, "detail": "right tracking unavailable"}
        self.ui.report_status(status)
        self.assertEqual(len(self.messages), 3)
        self.assertIn("right tracking unavailable", self.messages[-1])

    def test_pending_enter_prompt_is_not_repeated_by_status_or_extra_enter(self):
        self.runtime.health = lambda: Health(False, 0, "left tracking unavailable")
        self.ui.handle("\n")
        self.ui.handle("\n")
        self.ui.report_status({"state": "ready", "health": {"ready": False, "detail": "left tracking unavailable"}})
        self.assertEqual(len(self.messages), 1)
        self.assertIn("就绪后自动接合", self.messages[0])
        self.ui.handle(" ")
        self.assertFalse(self.ui.engage_pending)
        self.assertIn("暂停", self.messages[-1])

    def test_common_wait_reasons_are_short_chinese_without_changing_diagnostics(self):
        examples = (
            ("Waiting for Quest frames", "等待 Quest 数据"),
            ("Quest left tracking unavailable (flags=0, active=False)", "左手柄未被追踪"),
            ("Quest right tracking unavailable (flags=0)", "右手柄未被追踪"),
            ("Quest XR session not focused; close the headset system menu", "请关闭 Quest 系统菜单"),
            ("No Tianji feedback", "等待机器人反馈"),
        )
        for raw, expected in examples:
            self.assertIn(expected, self.ui.waiting_message(raw))
            self.assertNotIn("flags=", self.ui.waiting_message(raw))
        raw = "tj_move_joints: OnSetTargetState_A returned 1"
        self.assertEqual(cli.brief_reason(raw), raw)


class ConsoleTests(unittest.TestCase):
    def test_tty_colours_levels_but_redirect_and_no_color_are_plain(self):
        stream = io.StringIO()
        stream.isatty = lambda: True
        with patch.dict(os.environ, {}, clear=True):
            for level, label in (("info", "提示"), ("ready", "就绪"), ("warning", "警告"),
                                 ("error", "错误"), ("done", "完成")):
                self.assertIn("\x1b[", format_message("测试", level, stream=stream))
                self.assertIn(label, format_message("测试", level, stream=stream))
            print_message("测试", "ready", stream=stream)
            self.assertIn("测试", stream.getvalue())
            self.assertEqual(format_message("测试", "ready", stream=io.StringIO()), "[就绪] 测试")
        with patch.dict(os.environ, {"NO_COLOR": ""}):
            self.assertEqual(format_message("测试", "error", stream=stream), "[错误] 测试")

    def test_state_dedup_warning_limit_and_tty_progress(self):
        stream = io.StringIO()
        stream.isatty = lambda: True
        console = StatusConsole(stream=stream, warning_interval_s=5)
        progress = LiveProgress(stream=stream, interval_s=.2)
        with patch("bimanual_teleop.common.console.time.monotonic", side_effect=[0, 1, 6, 7, 7.1, 7.3]):
            console.state("就绪", "ready")
            console.state("就绪", "ready")
            console.warning("重复故障")
            console.warning("重复故障")
            console.warning("重复故障")
            progress.update("目标 1")
            progress.update("目标 2")
            progress.update("目标 3")
            progress.clear()
        output = stream.getvalue()
        self.assertEqual(output.count("重复故障"), 2)
        self.assertEqual(output.count("就绪"), 2)  # coloured label and message
        self.assertIn("\r", output)
        self.assertNotIn("目标 2", output)

    def test_sdk_and_python_logging_keep_warnings(self):
        sdk = SimpleNamespace(set_log_level=Mock())
        logger = logging.getLogger("bimanual_teleop")
        handlers, level, propagate = list(logger.handlers), logger.level, logger.propagate
        try:
            with patch.dict(sys.modules, {"wuji_sdk": sdk}):
                configure_runtime_logging(wuji=True)
                configure_runtime_logging(wuji=True)
            self.assertEqual(sdk.set_log_level.call_args_list, [
                unittest.mock.call("warn"), unittest.mock.call("warn")])
            self.assertEqual(logger.level, logging.WARNING)
        finally:
            logger.handlers[:] = handlers
            logger.setLevel(level)
            logger.propagate = propagate


class SchedulerTests(unittest.TestCase):
    def test_terminal_escape_sequences_do_not_become_motion_or_direction_keys(self):
        terminal = cli.NonblockingTerminal(io.StringIO())
        self.assertIsNone(terminal._keys("\x1b["))
        self.assertIsNone(terminal._keys("B\x1b[6~\x1bOP"))
        self.assertEqual(terminal._keys("h j"), "h j")

    def test_keyboard_does_not_block_ticks_and_delayed_cycles_are_skipped(self):
        clock = [0]
        runtime = Runtime()
        ui = cli.TeleopUI(runtime, PROFILE, enable_motion=True, emit=lambda _: None)
        keys = iter(("\n", None, None, None, None, "q"))

        def read(timeout):
            self.assertLessEqual(timeout, .005)
            key = next(keys)
            clock[0] += 30_000_000 if len(runtime.ticks) == 3 else 5_000_000
            return key

        with patch.object(cli.time, "monotonic_ns", side_effect=lambda: clock[0]):
            timing = cli.run_loop(runtime, ui, Mock(read=read))
        self.assertEqual(len(runtime.ticks), 6)
        self.assertEqual(timing["skipped_deadlines"], 5)
        self.assertEqual(sum(call[0] == "engage" for call in runtime.calls), 1)
        self.assertEqual(len(set(runtime.ticks)), len(runtime.ticks))

    def test_late_wakeup_and_slow_tick_do_not_cause_catch_up_bursts(self):
        for work_ns in (0, 12_000_000):
            with self.subTest(work_ns=work_ns):
                clock, completed = [0], []
                runtime = Runtime()
                ui = cli.TeleopUI(runtime, PROFILE, emit=lambda _: None)

                def tick(now):
                    runtime.ticks.append(now)
                    clock[0] += work_ns if len(runtime.ticks) == 2 else 0
                    completed.append(clock[0])

                def read(timeout):
                    clock[0] += round(timeout * 1e9)
                    if len(runtime.ticks) == 1:
                        clock[0] += 4_700_000
                    return "q" if len(runtime.ticks) == 4 else None

                runtime.tick = tick
                with patch.object(cli.time, "monotonic_ns", side_effect=lambda: clock[0]):
                    timing = cli.run_loop(runtime, ui, Mock(read=read))
                self.assertEqual(len(runtime.ticks), 4)
                self.assertTrue(all(b - a >= cli.PERIOD_NS
                                    for a, b in zip(runtime.ticks, runtime.ticks[1:])))
                # A slow tick must finish before the next one starts; elapsed
                # deadlines are counted instead of executing catch-up work.
                self.assertGreater(runtime.ticks[2], completed[1])
                self.assertEqual(timing["skipped_deadlines"], 2 if work_ns else 0)

    def test_repeated_status_is_quiet_without_records(self):
        clock = [0]
        runtime, messages = Runtime(), []
        ui = cli.TeleopUI(runtime, PROFILE, emit=messages.append)

        def read(timeout):
            clock[0] += 1_000_000_000
            return "q" if clock[0] >= 4_000_000_000 else None

        with patch.object(cli.time, "monotonic_ns", side_effect=lambda: clock[0]):
            cli.run_loop(runtime, ui, Mock(read=read))
        self.assertEqual(len(messages), 1)
        self.assertIn("设备已就绪", messages[0])
        self.assertNotIn("{", "".join(messages))

    def test_gesture_status_poll_does_not_repeat_console_output(self):
        clock = [0]
        runtime, messages = Runtime(), []
        diagnostics = {"hands": {side: {"v": False, "rock": False, "detail": "等待 V 手势"}
                                 for side in ("left", "right")},
                       "start_armed": True, "hold_ms": 0.}
        gesture = Mock(poll=Mock(return_value=None), status=Mock(return_value=diagnostics))
        ui = cli.TeleopUI(runtime, PROFILE, gesture=gesture, emit=messages.append)

        def read(timeout):
            clock[0] += 1_000_000_000
            return "q" if clock[0] >= 4_000_000_000 else None

        with patch.object(cli.time, "monotonic_ns", side_effect=lambda: clock[0]):
            cli.run_loop(runtime, ui, Mock(read=read))
        gesture.status.assert_not_called()
        self.assertEqual(sum("手势：" in message for message in messages), 0)

    def test_terminal_restores_settings_after_failure_and_treats_eof_as_exit(self):
        stream = Mock()
        stream.fileno.return_value = 12
        stream.isatty.return_value = True
        with patch.object(terminal_module.termios, "tcgetattr", return_value=["original"]), \
             patch.object(terminal_module.termios, "tcsetattr") as restore, \
             patch.object(terminal_module.tty, "setcbreak"):
            with self.assertRaisesRegex(RuntimeError, "test failure"):
                with cli.NonblockingTerminal(stream):
                    raise RuntimeError("test failure")
            restore.assert_called_once_with(12, terminal_module.termios.TCSADRAIN, ["original"])
        runtime = Runtime()
        ui = cli.TeleopUI(runtime, PROFILE, emit=lambda _: None)
        cli.run_loop(runtime, ui, Mock(read=lambda timeout: ""))
        self.assertEqual(runtime.calls[-1], ("pause", "keyboard exit"))


class MainTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.config = self.root / "config.yaml"
        self.settings = {"controller_ip": "192.0.2.7", "profile": {
            "profile_id": PROFILE.profile_id, "mode": PROFILE.mode, "parameters": PROFILE.parameters}}
        self.config.write_text(yaml.safe_dump(self.settings))
        self.args = ["--arms-only", "--config", str(self.config)]
        self.runtime = Runtime()
        self.stdout, self.stderr = io.StringIO(), io.StringIO()

    def invoke(self, *, start_error=None, prepare_error=None, keys="q"):
        if start_error:
            self.runtime.start = Mock(side_effect=start_error)
        terminal = Mock()
        terminal.__enter__ = Mock(return_value=Mock(read=lambda timeout: keys))
        terminal.__exit__ = Mock(return_value=False)
        with patch.object(cli, "NonblockingTerminal", return_value=terminal), \
             patch.object(cli, "confirm_motion", return_value=True), \
             patch.object(cli, "prepare_initial_pose", side_effect=prepare_error,
                          return_value={"side": "both"}) as prepare, \
             patch.object(cli, "create_runtime", return_value=self.runtime) as create, \
             patch.object(cli, "configure_runtime_logging"), \
             redirect_stdout(self.stdout), redirect_stderr(self.stderr):
            self.order = Mock()
            self.order.attach_mock(prepare, "prepare")
            self.order.attach_mock(create, "create")
            result = cli.main(self.args)
        self.prepare, self.create = prepare, create
        if prepare_error:
            self.assertEqual(self.runtime.calls, [])
            create.assert_not_called()
        else:
            self.assertEqual(self.runtime.calls[-1], ("close",))
        terminal.__exit__.assert_called_once()
        return result, create.call_args.args[0] if create.called else None

    def test_arms_only_prepares_then_waits_for_engagement_and_closes_cleanly(self):
        result, args = self.invoke()
        self.assertEqual(result, 0)
        self.assertTrue(args.arms_only)
        self.assertEqual(args.coordinate_frame, "headset")
        self.prepare.assert_called_once()
        self.assertFalse(any(call[0] in ("engage", "load", "jog") for call in self.runtime.calls))
        self.assertIn("实机遥操作", self.stderr.getvalue())
        self.assertIn("Enter 开始/恢复", self.stderr.getvalue())
        self.assertNotIn("手柄映射", self.stderr.getvalue())
        self.assertIn("已退出", self.stderr.getvalue())
        self.assertEqual(self.stdout.getvalue(), "")
        self.assertNotIn("\x1b[", self.stderr.getvalue())

    def test_shared_config_supplies_controller_ip_and_coordinate_frame(self):
        self.settings.update(controller_ip="192.0.2.8", quest={"coordinate_frame": "world"})
        self.config.write_text(yaml.safe_dump(self.settings))
        result, args = self.invoke()
        self.assertEqual(result, 0)
        self.assertEqual(args.robot_ip, "192.0.2.8")
        self.assertEqual(args.coordinate_frame, "world")

    def test_tianji_config_option_reaches_profile_and_initial_pose_preparation(self):
        self.settings.update(controller_ip="192.0.2.8", quest={"coordinate_frame": "world"})
        self.settings["profile"]["profile_id"] = "custom-tianji"
        self.config.write_text(yaml.safe_dump(self.settings))
        self.args = ["--arms-only", "--tianji-config", str(self.config)]
        result, args = self.invoke()
        self.assertEqual(result, 0)
        self.assertEqual(args.config, self.config)
        self.assertEqual(args.robot_ip, "192.0.2.8")
        self.assertEqual(args.coordinate_frame, "world")
        self.assertEqual(self.create.call_args.args[1].profile_id, "custom-tianji")
        self.assertIs(self.prepare.call_args.args[0], args)

    def test_cli_ip_overrides_shared_configuration(self):
        self.args += ["--robot-ip", "192.0.2.9"]
        result, args = self.invoke()
        self.assertEqual(result, 0)
        self.assertEqual(args.robot_ip, "192.0.2.9")

    def test_invalid_coordinate_frame_fails_before_preparation_or_connection(self):
        self.settings["quest"] = {"coordinate_frame": "grip"}
        self.config.write_text(yaml.safe_dump(self.settings))
        with patch.object(cli, "prepare_initial_pose") as prepare, \
                patch.object(cli, "create_runtime") as create, \
                redirect_stderr(self.stderr):
            self.assertEqual(cli.main([*self.args]), 1)
        prepare.assert_not_called()
        create.assert_not_called()
        self.assertIn("coordinate_frame", self.stderr.getvalue())

    def test_motion_prepares_before_creating_runtime_and_still_waits_for_enter(self):
        result, args = self.invoke()
        self.assertEqual(result, 0)
        self.assertEqual([call[0] for call in self.order.mock_calls], ["prepare", "create"])
        self.assertEqual(args.config, self.config)
        self.assertEqual(self.runtime.calls, [("start",), ("pause", "keyboard exit"), ("close",)])
        self.assertEqual(self.create.call_args.args[2], None)
        self.assertIn("已退出", self.stderr.getvalue())

    def test_single_side_cli_passes_selected_profile(self):
        self.args += ["--side", "left"]
        result, args = self.invoke()
        self.assertEqual(result, 0)
        self.assertEqual(args.side, "left")
        selected = self.create.call_args.args[1]
        self.assertEqual(selected.parameters["active_arms"], ["left"])
        self.assertEqual(selected.profile_id, "test-profile-left")

    def test_preparation_failure_prevents_all_teleop_device_creation(self):
        result, _ = self.invoke(prepare_error=RuntimeError("initial pose rejected"))
        self.assertEqual(result, 1)
        self.assertIn("initial pose rejected", self.stderr.getvalue())

    def test_start_failure_still_closes_both_components(self):
        result, _ = self.invoke(start_error=RuntimeError("USB unavailable"))
        self.assertEqual(result, 1)
        self.assertIn("USB unavailable", self.stderr.getvalue())
        self.assertNotIn("Traceback", self.stderr.getvalue())

    def test_noninteractive_stdin_fails_before_creating_devices(self):
        with patch.object(sys, "stdin", io.StringIO()), \
             patch.object(cli, "create_runtime") as runtime, \
             patch.object(cli, "configure_runtime_logging"), \
             redirect_stdout(self.stdout), redirect_stderr(self.stderr):
            self.assertEqual(cli.main(self.args), 1)
        runtime.assert_not_called()
        self.assertIn("交互终端", self.stderr.getvalue())

    def test_cli_has_no_output_option_or_runtime_records(self):
        with patch.object(sys, "stderr", io.StringIO()):
            with self.assertRaises(SystemExit):
                cli.main(self.args + ["--output", str(self.root / "logs")])
        self.assertFalse((self.root / "logs").exists())


class PreparationStartupTests(unittest.TestCase):
    """The position subprocess must finish before teleop opens a device."""

    def setUp(self):
        temp = tempfile.TemporaryDirectory()
        self.addCleanup(temp.cleanup)
        self.root = Path(temp.name)
        self.args = SimpleNamespace(robot_ip="192.0.2.7", config=self.root / "custom-config.yaml",
                                    library=self.root / "custom.so")
        self.process = Mock()
        self.process.poll.return_value = 0
        self.process.wait.return_value = 0

    def spawn(self, command, **kwargs):
        self.command, self.options = command, kwargs
        return self.process

    def run_preparation(self, keys=(None, "\n", None)):
        terminal = Mock(read=Mock(side_effect=keys))
        with patch.object(preparation, "ROOT", self.root), \
                patch.object(preparation.subprocess, "Popen", side_effect=self.spawn) as spawn, \
                redirect_stderr(io.StringIO()):
            result = cli.prepare_initial_pose(self.args, terminal)
        return result, spawn

    def test_complete_child_releases_connection_before_return(self):
        result, _ = self.run_preparation()
        self.process.wait.assert_called_once_with()
        self.process.send_signal.assert_not_called()
        self.assertEqual(self.command[0], sys.executable)
        for flag, expected in (("--ip", self.args.robot_ip), ("--tianji-config", self.args.config),
                               ("--library", self.args.library)):
            self.assertEqual(self.command[self.command.index(flag)+1], str(expected))
        self.assertEqual(self.command[1], "-c")
        self.assertIn("confirmed=True", self.command[2])
        self.assertNotIn("--output", self.command)
        self.assertTrue(self.options["start_new_session"])
        self.assertEqual(self.options["stdin"], preparation.subprocess.DEVNULL)
        self.assertNotIn("env", self.options)
        self.assertEqual(result["side"], "both")
        self.assertFalse((self.root / "data").exists())

    def test_single_side_preparation_forwards_selection(self):
        self.args.side = "right"
        result, _ = self.run_preparation()
        self.assertEqual(self.command[self.command.index("--side")+1], "right")
        self.assertEqual(result["side"], "right")

    def test_failed_exit_blocks_teleop_without_summary_file(self):
        self.process.poll.return_value = 1
        with self.assertRaisesRegex(RuntimeError, "准备失败.*退出码 1"):
            self.run_preparation()
        self.assertFalse((self.root / "data").exists())

    def test_space_quit_eof_and_ctrl_c_stop_child_and_wait_for_cleanup(self):
        for key in (" ", "q", "Q", "", "\x03", KeyboardInterrupt()):
            with self.subTest(key=key):
                self.process.reset_mock()
                self.process.poll.return_value = None
                self.process.wait.side_effect = [preparation.subprocess.TimeoutExpired("prepare", .1),
                                                 KeyboardInterrupt(), 1]
                exception = KeyboardInterrupt if isinstance(key, BaseException) else RuntimeError
                with self.assertRaises(exception):
                    self.run_preparation((None, key))
                self.process.send_signal.assert_called_once_with(preparation.signal.SIGINT)
                self.assertEqual(self.process.wait.call_count, 3)

    def test_cancel_before_launch_never_starts_preparation(self):
        with patch.object(preparation.subprocess, "Popen") as spawn:
            with self.assertRaisesRegex(RuntimeError, "已取消"):
                cli.prepare_initial_pose(self.args, Mock(read=lambda _: "q"))
        spawn.assert_not_called()

    def test_real_subprocess_cancellation_waits_for_child_finally(self):
        package = self.root / "bimanual_teleop"
        scripts = package / "cli"
        scripts.mkdir(parents=True)
        (package / "__init__.py").touch()
        (scripts / "__init__.py").touch()
        (scripts / "prepare_tianji_teleop.py").write_text(
            "import argparse\n"
            "from pathlib import Path\n"
            "import time\n"
            "def parser():\n"
            "    result = argparse.ArgumentParser()\n"
            "    for name in ('--ip', '--tianji-config', '--side', '--library'):\n"
            "        result.add_argument(name)\n"
            "    return result\n"
            "def run(args, *, confirmed=False):\n"
            "    assert confirmed\n"
            "    try:\n"
            "        Path('started').touch()\n"
            "        time.sleep(10)\n"
            "    except KeyboardInterrupt:\n"
            "        pass\n"
            "    finally:\n"
            "        time.sleep(.05)\n"
            "        Path('held-and-closed').touch()\n"
            "    return 0\n")
        started = self.root / "started"
        deadline = time.monotonic()+3

        def read(timeout):
            if started.exists():
                return " "
            if time.monotonic() > deadline:
                raise RuntimeError("test child did not start")
            time.sleep(timeout)
            return None

        with patch.object(preparation, "ROOT", self.root), redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError, "已取消"):
                cli.prepare_initial_pose(self.args, Mock(read=read))
        self.assertTrue((self.root / "held-and-closed").exists())


class RuntimeIntegrationTests(unittest.TestCase):
    def test_enter_before_tracking_is_ready_engages_once_after_controller_wakes(self):
        from test_quest_tianji_runtime import RuntimeFixture

        fx = RuntimeFixture(motion=True)
        self.addCleanup(fx.runtime.close)
        fx.quest.emit(invalid="left")
        ui = cli.TeleopUI(fx.runtime, fx.profile, enable_motion=True, emit=lambda _: None)
        ui.handle("\n")
        self.assertTrue(ui.engage_pending)
        self.assertFalse(fx.driver.engaged)
        reads = 0

        def read(timeout):
            nonlocal reads
            reads += 1
            fx.advance()
            if reads == 4:
                self.assertTrue(fx.driver.engaged)
                self.assertFalse(ui.engage_pending)
                self.assertGreater(fx.runtime.cycles, 0)
                return "q"

        with patch.object(cli.time, "monotonic_ns", side_effect=fx.clock):
            cli.run_loop(fx.runtime, ui, Mock(read=read))
        self.assertEqual(fx.driver.calls.count("engage"), 1)

    def test_space_and_quit_cancel_pending_engagement(self):
        for key in (" ", "q"):
            runtime = Runtime()
            runtime.health = lambda: Health(False, 0, "left tracking unavailable")
            ui = cli.TeleopUI(runtime, PROFILE, emit=lambda _: None)
            ui.handle("\n")
            self.assertTrue(ui.engage_pending)
            ui.handle(key)
            self.assertFalse(ui.engage_pending)
            self.assertFalse(any(call[0] == "engage" for call in runtime.calls))

    def test_real_runtime_enter_engages_without_calibration_files_or_gestures(self):
        from test_quest_tianji_runtime import RuntimeFixture

        fx = RuntimeFixture(motion=True)
        self.addCleanup(fx.runtime.close)
        with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock):
            ui = cli.TeleopUI(fx.runtime, fx.profile, enable_motion=True, emit=lambda _: None)
            self.assertEqual(fx.driver.calls, ["start"])
            ui.handle("\n")
            self.assertEqual(fx.runtime.status()["mode"], "follow")
            self.assertIsNotNone(fx.runtime.tick())
            ui.handle(" ")
            self.assertFalse(fx.driver.engaged)
            fx.advance()
            ui.handle("\n")
            self.assertEqual(fx.runtime.status()["mode"], "follow")


if __name__ == "__main__":
    unittest.main()
