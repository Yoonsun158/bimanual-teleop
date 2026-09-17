"""Real pause/engage/home paths with synthetic device packets; no hardware I/O."""

from copy import deepcopy
from contextlib import redirect_stderr
import io
import math
import unittest
from unittest.mock import patch

from bimanual_teleop.cli.runtime import TeleopUI
from bimanual_teleop.control.arm.cartesian import TianjiCartesianExecutor
from bimanual_teleop.control.arm.quest import QuestTianjiTeleop
from bimanual_teleop.control.combined import QuestTianjiWujiTeleop
from bimanual_teleop.control.hand.gesture import GestureCommands
from bimanual_teleop.devices.tianji.config import load_config
from bimanual_teleop.devices.tianji.driver import TianjiDriver, FeedbackSnapshot
from bimanual_teleop.system import SystemState
from bimanual_teleop.types import ControlProfile
from tests.support.quest import Kinematics, Quest
from tests.support.clock import Clock
from tests.support.tianji import FakeSDK, packet
from tests.support.gesture import OPEN, ROCK, V, frame
from tests.support.combined import Runtime


class PauseRecoveryTests(unittest.TestCase):
    def setUp(self):
        self.clock = Clock()
        timer = patch("time.monotonic_ns", self.clock)
        timer.start()
        self.addCleanup(timer.stop)
        settings = load_config()
        profile = ControlProfile(**settings["profile"])
        self.driver = TianjiDriver("192.0.2.1")
        self.native = FakeSDK(self.driver)
        self.native.stop_nack = True  # The reported failure must not block normal pause.
        self.driver._sdk = self.native
        self.driver._on_feedback(packet())
        self.quest, kinematics = Quest(self.clock), Kinematics()
        arms = QuestTianjiTeleop(self.quest, self.driver,
            TianjiCartesianExecutor(self.driver, kinematics),
            profile=profile, ready_pose=settings["ready_pose"],
            coordinate_frame="world", clock_ns=self.clock)
        self.hand_calls = []
        self.hands = Runtime("hands", self.hand_calls)
        self.runtime = QuestTianjiWujiTeleop(arms, self.hands)
        with patch.object(self.driver, "start"):
            self.runtime.start()
        self.samples = {side: None for side in ("left", "right")}
        self.gesture = GestureCommands({s: lambda s=s: self.samples[s] for s in self.samples})
        self.ui = TeleopUI(self.runtime, profile, gesture=self.gesture,
                          home_enabled=True, background_engage=True, emit=lambda _: None)
        self.addCleanup(self.close_runtime)
        wait = patch.object(self.driver._stop, "wait", side_effect=lambda _: self.advance(5_000_000))
        wait.start()
        self.addCleanup(wait.stop)
        self.targets = settings["ready_pose"]["target_deg"]

    def close_runtime(self):
        with patch("time.sleep", side_effect=lambda _: self.advance(5_000_000)):
            self.ui.close()

    def advance(self, ns=20_000_000):
        self.clock.advance(ns)
        p = deepcopy(self.driver._packet)
        p.packet_index += 1
        p.sequence[:] = [s + 1 for s in p.sequence]
        p.received_ns = self.clock()
        p.low_speed[:] = [1, 1]
        self.driver._on_feedback(p)
        self.quest.emit()

    def finish_operation(self):
        if self.ui._operation_thread is not None:
            self.ui._operation_thread.join(2.)
            self.assertFalse(self.ui._operation_thread.is_alive())
            self.ui.poll_operation()
            self.assertIsNone(self.ui._operation_error)

    def hold(self, left, right, frames=16):
        actions = []
        for _ in range(frames):
            self.advance()
            for side, pose in (("left", left), ("right", right)):
                self.samples[side] = frame(side, pose, self.clock(), self.driver._packet.packet_index)
            self.ui.report_runtime_pause()
            action = self.gesture.poll(self.clock(), start_ready=self.runtime.health().ready,
                home_ready=self.runtime.state == SystemState.PAUSED)
            if action is not None:
                actions.append(self.ui.handle_gesture(action[0]))
                self.finish_operation()
            self.runtime.tick(self.clock())
        return actions

    def test_rock_pause_enter_and_v_resume_then_open_palms_home_and_resume(self):
        self.assertEqual(self.hold(V, V), ["engage"])
        self.assertEqual(self.hold(ROCK, OPEN), ["pause"])
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertTrue(self.runtime.health().ready, self.runtime.health().detail)

        self.ui.handle("\n")
        self.finish_operation()
        self.assertEqual(self.runtime.state, SystemState.ENGAGED)
        self.assertEqual(self.hold(OPEN, OPEN), [])
        self.assertEqual(self.hold(ROCK, OPEN), ["pause"])
        self.hold(ROCK, OPEN, frames=1)
        self.assertEqual(self.hold(V, V), ["engage"])

        self.assertEqual(self.hold(ROCK, OPEN), ["pause"])
        following_before = self.hand_calls.count("hands.follow")
        with redirect_stderr(io.StringIO()):
            self.assertEqual(self.hold(OPEN, OPEN, frames=51), ["home"])
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertEqual(self.hands.state, SystemState.PAUSED)
        self.assertEqual(self.hand_calls.count("hands.follow"), following_before)
        for side, target in self.targets.items():
            measured = self.driver.get_latest().payload.arms[side].joints.position_rad
            for actual, expected in zip(measured, target):
                self.assertAlmostEqual(math.degrees(actual), expected, places=3)
        self.assertEqual(self.hold(OPEN, OPEN, frames=60), [])
        self.assertEqual(self.hold(V, V), ["engage"])
        self.assertEqual(self.runtime.state, SystemState.ENGAGED)
        self.assertNotIn("hold", [name for name, _ in self.native.calls])

    def test_paused_home_gesture_remains_available_when_quest_is_unavailable(self):
        self.assertEqual(self.hold(V, V), ["engage"])
        self.assertEqual(self.hold(ROCK, OPEN), ["pause"])
        self.quest.ready = False
        self.assertFalse(self.runtime.health().ready)
        with redirect_stderr(io.StringIO()):
            self.assertEqual(self.hold(OPEN, OPEN, frames=51), ["home"])
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertFalse(self.runtime.health().ready)
        self.assertEqual(self.hand_calls.count("hands.follow"), 1)


if __name__ == "__main__":
    unittest.main()
