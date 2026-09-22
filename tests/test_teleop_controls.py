"""Configurable operator controls, with no hardware connections."""

import threading
import unittest
from unittest.mock import Mock

from bimanual_teleop.cli.runtime import TeleopUI, run_loop
from bimanual_teleop.recording.ui import RecordingUI
from bimanual_teleop.system import SystemState
from bimanual_teleop.types import Health


class ControlsTests(unittest.TestCase):
    def setUp(self):
        self.runtime = Mock(state=SystemState.READY, last_error=None)
        self.runtime.health.return_value = Health(True, 0)
        self.runtime.engage.side_effect = lambda _: setattr(self.runtime, "state", SystemState.ENGAGED)
        self.runtime.pause.side_effect = lambda _: setattr(self.runtime, "state", SystemState.PAUSED)
        self.messages = []
        self.gesture = Mock()
        self.ui = TeleopUI(self.runtime, None, toggle_engagement_key="t", ready_pose_key="r",
                           home_enabled=True, gesture=self.gesture, gesture_engagement_enabled=False,
                           emit=self.messages.append)
        self.addCleanup(self.ui.close)

    def test_toggle_engages_pauses_and_reengages(self):
        self.ui.handle("e")
        self.runtime.engage.assert_not_called()
        self.ui.handle("T")
        self.assertEqual(self.runtime.state, SystemState.ENGAGED)
        self.ui.handle("t")
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.ui.handle("t")
        self.assertEqual(self.runtime.engage.call_count, 2)
        self.assertEqual(self.runtime.state, SystemState.ENGAGED)

    def test_toggle_cancels_pending_engagement(self):
        self.runtime.health.return_value = Health(False, 0, "waiting")
        self.ui.handle("t")
        self.assertTrue(self.ui.engage_pending)
        self.ui.handle("t")
        self.assertFalse(self.ui.engage_pending)
        self.runtime.health.return_value = Health(True, 0)
        self.runtime.engage.assert_not_called()
        self.ui.handle("t")
        self.runtime.engage.assert_called_once()

    def test_toggle_cancels_background_engagement(self):
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def engage(_):
            entered.set()
            release.wait(1.)
        self.runtime.engage.side_effect = engage
        self.ui.background_engage = True
        self.ui.handle("t")
        self.assertTrue(entered.wait(.5))
        self.ui.handle("t")
        self.assertTrue(self.ui._operation_cancel.is_set())
        self.runtime.pause.assert_called_once()
        release.set()
        self.ui._operation_thread.join(1.)
        self.ui.poll_operation()
        self.assertEqual(self.runtime.state, SystemState.PAUSED)

    def test_custom_home_key_requires_pause_and_does_not_resume(self):
        self.ui.handle("r")
        self.runtime.home.assert_not_called()
        self.assertIn("按 R", self.messages[-1])
        self.ui.handle(" ")
        self.ui.handle("h")
        self.runtime.home.assert_not_called()
        self.ui.handle("R")
        self.ui._operation_thread.join(1.)
        self.ui.poll_operation()
        self.runtime.home.assert_called_once()
        self.runtime.engage.assert_not_called()
        self.assertEqual(self.runtime.state, SystemState.PAUSED)

    def test_toggle_cancels_running_home(self):
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        def home(cancel):
            self.runtime.state = SystemState.HOMING
            entered.set()
            release.wait(1.)
        self.runtime.home.side_effect = home
        self.runtime.state = SystemState.PAUSED
        self.ui.handle("r")
        self.assertTrue(entered.wait(.5))
        self.ui.handle("t")
        self.assertTrue(self.runtime.home.call_args.args[0].is_set())
        release.set()
        self.ui._operation_thread.join(1.)
        self.ui.poll_operation()
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.runtime.engage.assert_not_called()

    def test_disabled_gestures_preserve_keyboard_controls(self):
        for state in (SystemState.READY, SystemState.ENGAGED, SystemState.PAUSED, SystemState.HOMING):
            self.runtime.state = state
            for command in ("engage", "pause", "home"):
                with self.subTest(state=state, command=command):
                    self.assertEqual(self.ui.handle_gesture(command), "ignored")
        self.runtime.engage.assert_not_called()
        self.runtime.pause.assert_not_called()
        self.runtime.home.assert_not_called()
        self.runtime.state = SystemState.READY
        self.assertNotIn("比 V", self.ui.help_text)
        self.assertNotIn("比 V", self.ui.start_hint)
        self.assertNotIn("摇滚", self.ui.help_text)
        self.assertNotIn("张开", self.ui.help_text)
        self.ui.handle("\n")
        self.runtime.engage.assert_called_once()
        self.ui.handle(" ")
        self.runtime.pause.assert_called_once()
        self.ui.handle("r")
        self.ui._operation_thread.join(1.)
        self.ui.poll_operation()
        self.runtime.home.assert_called_once()

    def test_disabled_gestures_are_not_polled(self):
        self.runtime.status.return_value = {"state": "ready"}
        run_loop(self.runtime, self.ui, Mock(read=Mock(return_value="q")))
        self.gesture.poll.assert_not_called()

    def test_disabled_gestures_do_not_end_recording(self):
        recorder = Mock()
        ui = RecordingUI(self.runtime, None, recorder=recorder, gesture=self.gesture,
                         gesture_engagement_enabled=False, emit=lambda _: None)
        for command in ("engage", "pause", "home"):
            self.assertEqual(ui.handle_gesture(command), "ignored")
        recorder.end.assert_not_called()
        self.runtime.engage.assert_not_called()
        self.runtime.pause.assert_not_called()
        self.runtime.home.assert_not_called()

    def test_toggle_stop_finishes_recording_before_pause(self):
        recorder = Mock()
        self.runtime.state = SystemState.ENGAGED
        calls = Mock()
        calls.attach_mock(recorder.end, "end")
        calls.attach_mock(self.runtime.pause, "pause")
        ui = RecordingUI(self.runtime, None, recorder=recorder, toggle_engagement_key="t",
                         emit=lambda _: None)
        ui.handle("T")
        self.assertEqual(calls.mock_calls[0][0], "end")
        self.assertEqual(calls.mock_calls[0].kwargs, {})
        self.assertEqual(calls.mock_calls[-1][0], "pause")
        self.assertEqual(self.runtime.state, SystemState.PAUSED)


if __name__ == "__main__":
    unittest.main()
