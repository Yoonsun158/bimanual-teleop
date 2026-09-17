"""Homing uses one connection, stops promptly, and never resumes itself."""

from copy import deepcopy
from contextlib import redirect_stderr
import io
import threading
import time
import unittest
from unittest.mock import Mock, patch

from bimanual_teleop.cli.runtime import TeleopUI
from bimanual_teleop.control.arm.preparation import load_targets, ready_profile, move_to_ready_pose
from bimanual_teleop.control.combined import QuestTianjiWujiTeleop
from bimanual_teleop.devices.tianji.config import load_config
from bimanual_teleop.devices.tianji.driver import TianjiDriver, FeedbackSnapshot
from bimanual_teleop.system import SystemState
from bimanual_teleop.types import ControlProfile, Health
from tests.support.tianji import FakeSDK, packet
from tests.support.quest import RuntimeFixture


class JointMoveCancellationTests(unittest.TestCase):
    def setUp(self):
        self.driver = TianjiDriver("192.0.2.1", watchdog_s=.5)
        self.native = FakeSDK(self.driver)
        self.driver._sdk = self.native
        self.driver._on_feedback(packet())
        self.driver.configure(ControlProfile(**load_config()["profile"]))
        self.native.complete_joint_move = False
        value = deepcopy(self.driver._packet)
        value.state[:] = [1, 1]
        value.sequence[:] = [s+1 for s in value.sequence]
        self.driver._on_feedback(value)
        self.addCleanup(self.driver.close)

    def test_accepted_target_is_not_measured_arrival_and_timeout_stops(self):
        def feedback(_):
            value = deepcopy(self.driver._packet)
            value.sequence[:] = [s+1 for s in value.sequence]
            value.received_ns = time.monotonic_ns()
            value.low_speed[:] = [1, 1]
            value.target[:7] = [0.] * 7
            self.driver._on_feedback(value)
        with patch.object(self.driver._stop, "wait", side_effect=feedback):
            with self.assertRaisesRegex(RuntimeError, "timed out"):
                self.driver.move_joints("left", (0.,)*7, timeout_s=.02)
        self.assertFalse(self.driver.engaged)
        self.assertTrue(any(name == "hold" for name, _ in self.native.calls))

    def test_pause_acquires_driver_lock_while_move_waits(self):
        errors = []
        def move():
            try:
                self.driver.move_joints("left", (0.,)*7)
            except RuntimeError as error:
                errors.append(str(error))
        worker = threading.Thread(target=move)
        worker.start()
        deadline = time.monotonic()+1
        while not self.driver.engaged and time.monotonic() < deadline:
            time.sleep(.001)
        acquired = self.driver._lock.acquire(timeout=.1)
        self.assertTrue(acquired, "move held the command lock while waiting")
        if acquired:
            try:
                self.driver.request_hold("test pause")
            finally:
                self.driver._lock.release()
        worker.join(timeout=1.)
        self.assertFalse(worker.is_alive())
        self.assertTrue(errors)

    def test_pre_cancelled_move_sends_nothing(self):
        cancel = threading.Event()
        cancel.set()
        self.native.calls.clear()
        with self.assertRaisesRegex(RuntimeError, "cancelled"):
            self.driver.move_joints("left", (0.,)*7, cancel=cancel)
        self.assertFalse(self.native.calls)


class ReadyPoseOperationTests(unittest.TestCase):
    def setUp(self):
        self.settings = load_config()
        self.profile = ControlProfile(**self.settings["profile"])
        source, order, self.targets = load_targets(self.settings["ready_pose"])
        self.ready = ready_profile(self.profile, source, order)

    def test_clearing_failure_blocks_configuration_and_motion(self):
        driver = Mock()
        driver.clear_errors.side_effect = RuntimeError("fault remains")
        with redirect_stderr(io.StringIO()), self.assertRaisesRegex(RuntimeError, "fault remains"):
            move_to_ready_pose(driver, self.ready, self.targets)
        driver.configure.assert_not_called()
        driver.move_joints.assert_not_called()

    def test_cancel_between_arms_and_ready_profile_does_not_mutate_teleop(self):
        driver, cancel = Mock(), threading.Event()
        driver.move_joints.side_effect = lambda *a, **kw: cancel.set()
        with redirect_stderr(io.StringIO()), self.assertRaisesRegex(RuntimeError, "取消"):
            move_to_ready_pose(driver, self.ready, self.targets, cancel=cancel)
        self.assertEqual(driver.move_joints.call_count, 1)
        self.assertEqual(self.profile.parameters, self.settings["profile"]["parameters"])
        self.assertEqual(self.ready.parameters["arms"]["left"]["velocity_ratio"], 25)

    def test_runtime_keeps_connection_and_reconfigures_on_next_engagement(self):
        fx = RuntimeFixture()
        self.addCleanup(fx.runtime.close)
        fx.runtime.profile = self.profile
        fx.runtime.ready_pose = self.settings["ready_pose"]
        fx.runtime.pause("test")
        fx.driver.clear_errors = Mock()
        fx.driver.move_joints = Mock()
        with redirect_stderr(io.StringIO()):
            fx.runtime.home(threading.Event())
        self.assertEqual(fx.runtime.state, SystemState.PAUSED)
        self.assertEqual(fx.driver.calls.count("start"), 1)
        self.assertNotIn("close", fx.driver.calls)
        self.assertEqual(fx.driver.move_joints.call_count, 2)
        before = fx.driver.calls.count("configure")
        with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock):
            fx.runtime.engage()
        self.assertEqual(fx.driver.calls.count("configure"), before+1)
        self.assertEqual(fx.runtime.state, SystemState.ENGAGED)

    def test_combined_home_never_engages_hands(self):
        arms, hands = Mock(), Mock()
        runtime = QuestTianjiWujiTeleop(arms, hands)
        runtime._state = SystemState.PAUSED
        cancel = threading.Event()
        runtime.home(cancel)
        arms.home.assert_called_once_with(cancel)
        hands.pause.assert_called_once()
        hands.prepare_engage.assert_not_called()
        hands.begin_follow.assert_not_called()
        self.assertEqual(runtime.state, SystemState.PAUSED)


class HomeKeyboardTests(unittest.TestCase):
    def setUp(self):
        self.entered, self.release = threading.Event(), threading.Event()
        self.runtime = Mock(state=SystemState.PAUSED, last_error=None)
        self.runtime.health.return_value = Health(True, 0)
        self.ui = TeleopUI(self.runtime, None, home_enabled=True, emit=lambda _: None)
        self.addCleanup(self.ui.close)
        self.addCleanup(self.release.set)

        def home(cancel):
            self.entered.set()
            self.runtime.state = SystemState.HOMING
            self.release.wait(1.)
            self.runtime.state = SystemState.PAUSED
        self.runtime.home.side_effect = home

    def test_home_is_paused_only_and_mutually_exclusive_with_engage(self):
        self.runtime.state = SystemState.ENGAGED
        self.ui.handle("h")
        self.runtime.home.assert_not_called()
        self.runtime.state = SystemState.PAUSED
        self.ui.handle("h")
        self.assertTrue(self.entered.wait(.5))
        self.ui.handle("h")
        self.ui.handle("\n")
        self.runtime.home.assert_called_once()
        self.runtime.engage.assert_not_called()
        self.release.set()
        self.ui._operation_thread.join(1.)
        self.ui.poll_operation()
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.runtime.engage.assert_not_called()

    def test_pause_sets_cancel_before_stop_and_cancels_pending_resume(self):
        self.ui.engage_pending = True
        self.ui.handle("h")
        self.assertTrue(self.entered.wait(.5))
        self.assertFalse(self.ui.engage_pending)
        self.ui.handle(" ")
        self.assertTrue(self.ui._operation_cancel.is_set())
        self.runtime.pause.assert_called_once()


if __name__ == "__main__":
    unittest.main()
