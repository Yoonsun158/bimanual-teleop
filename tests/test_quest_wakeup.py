"""Quest startup power override and cleanup, without touching hardware."""

import io
import subprocess
import unittest
from unittest.mock import Mock, patch

from bimanual_teleop.devices.quest.adapter import (
    POWER_OVERRIDE_ACTION, POWER_RESTORE_ACTION, QuestSource,
)


class QuestWakeupTests(unittest.TestCase):
    def setUp(self):
        self.calls = []
        self.process = Mock(stdout=io.StringIO(""))
        self.process.poll.return_value = None
        self.source = QuestSource()

    def adb(self, *args):
        self.calls.append(args)
        output = ""
        if args == ("devices", "-l"):
            output = "List of devices attached\nquest-usb device usb:1-2 model:Quest_3S\n"
        elif "pm" in args:
            output = "package:/data/app/quest/base.apk\n"
        elif "pidof" in args:
            raise subprocess.CalledProcessError(1, args)
        return subprocess.CompletedProcess(args, 0, output, "")

    def start(self):
        with patch.object(self.source, "_adb", side_effect=self.adb), \
             patch("bimanual_teleop.devices.quest.adapter.subprocess.Popen", return_value=self.process), \
             patch("bimanual_teleop.devices.quest.adapter.threading.Thread", side_effect=lambda **kwargs: Mock()):
            self.source.start()

    def close(self):
        with patch.object(self.source, "_adb", side_effect=self.adb):
            self.source.close()

    def test_wake_before_launch_and_restore_once_after_stop(self):
        self.start()
        self.close()
        self.close()
        operations = [args[3:] for args in self.calls if args[:3] == ("-s", "quest-usb", "shell")]
        override = operations.index(("am", "broadcast", "-a", POWER_OVERRIDE_ACTION))
        wake = operations.index(("input", "keyevent", "KEYCODE_WAKEUP"))
        launch = next(i for i, args in enumerate(operations) if args[:2] == ("am", "start"))
        stop = next(i for i, args in enumerate(operations) if args[:2] == ("am", "force-stop"))
        restore = operations.index(("am", "broadcast", "-a", POWER_RESTORE_ACTION))
        self.assertLess(wake, override)
        self.assertLess(override, launch)
        self.assertLess(stop, restore)
        self.assertEqual(sum(POWER_RESTORE_ACTION in args for args in self.calls), 1)

    def test_opt_out_and_unstarted_close_do_not_change_power(self):
        self.close()
        self.assertEqual(self.calls, [])
        self.source = QuestSource(keep_awake=False)
        self.start()
        self.close()
        self.assertFalse(any("broadcast" in args or "keyevent" in args for args in self.calls))

    def test_override_timeout_restores_detection_without_launching_app(self):
        def failing_adb(*args):
            result = self.adb(*args)
            if POWER_OVERRIDE_ACTION in args:
                raise subprocess.TimeoutExpired(args, 10)
            return result

        with patch.object(self.source, "_adb", side_effect=failing_adb), \
             patch("bimanual_teleop.devices.quest.adapter.subprocess.Popen") as popen:
            with self.assertRaises(subprocess.TimeoutExpired):
                self.source.start()
            popen.assert_not_called()
        self.assertTrue(any(POWER_RESTORE_ACTION in args for args in self.calls))
        self.assertFalse(any("force-stop" in args for args in self.calls))
        self.assertFalse(self.source._power_override_requested)

    def test_existing_session_is_not_woken_or_stopped(self):
        def existing_adb(*args):
            if "pidof" in args:
                return subprocess.CompletedProcess(args, 0, "1234\n", "")
            return self.adb(*args)

        with patch.object(self.source, "_adb", side_effect=existing_adb):
            with self.assertRaisesRegex(RuntimeError, "already running"):
                self.source.start()
        self.close()
        self.assertFalse(any("broadcast" in args or "force-stop" in args for args in self.calls))

    def test_failed_restore_can_be_retried(self):
        self.start()
        with patch.object(self.source, "_adb", side_effect=subprocess.TimeoutExpired("adb", 10)), \
             self.assertLogs("bimanual_teleop.devices.quest.adapter", level="WARNING"):
            self.source.close()
        self.assertTrue(self.source._power_override_requested)
        self.close()
        self.assertFalse(self.source._power_override_requested)


if __name__ == "__main__":
    unittest.main()
