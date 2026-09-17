"""Shutdown regressions with synthetic feedback only; no hardware access."""

from copy import deepcopy
import errno
import math
import threading
import time
import unittest
from unittest.mock import patch

from bimanual_teleop.devices.tianji.driver import TianjiDriver, FeedbackSnapshot
from tests.support.tianji import FakeSDK, Sink, packet
from tests.support import tianji as fixtures


class ShutdownTests(unittest.TestCase):
    def setUp(self):
        self.driver = TianjiDriver("192.0.2.1", watchdog_s=.2)
        self.native = FakeSDK(self.driver)
        self.sink = Sink()
        self.driver._sdk, self.driver._sink = self.native, self.sink
        self.driver._on_feedback(packet())
        timeout = patch("bimanual_teleop.devices.tianji.driver.SHUTDOWN_TIMEOUT_S", .06)
        timeout.start()
        self.addCleanup(timeout.stop)
        self.addCleanup(self.driver.close)

    def configure(self, sides=("left",)):
        self.driver.configure(fixtures.TianjiFixture.make_profile(sides))

    def engage(self, sides=("left",)):
        self.configure(sides)
        self.driver.engage()

    def disable_masks(self):
        return [args[0] for name, args in self.native.calls if name == "disable"]

    def fail_close(self):
        with self.assertLogs("bimanual_teleop.devices.tianji.driver", level="ERROR") as logs:
            with self.assertRaisesRegex(RuntimeError, "退出停机未确认.*实体急停"):
                self.driver.close()
        self.assertIn("退出停机未确认", logs.output[0])
        self.assertEqual(self.native.calls[-1][0], "close")
        self.assertIsNone(self.driver._sdk)
        self.assertFalse(any(getattr(event, "kind", None) == "tianji.disable_confirmed"
                             for event in self.sink.events))

    def test_readonly_and_configure_only_close_do_not_disable_unowned_arms(self):
        self.configure()
        self.driver.close()
        self.assertEqual(self.disable_masks(), [])

    def test_paused_arm_is_still_disabled_on_exit(self):
        self.engage(("right",))
        self.driver.request_hold("operator pause")
        self.native.calls.clear()
        self.driver.close()
        self.assertEqual([name for name, _ in self.native.calls], ["disable", "close"])
        self.assertEqual(self.disable_masks(), [2])

    def test_finished_home_accepts_idle_with_command_status_minus_one(self):
        self.configure(("left", "right"))
        self.driver.move_joints("left", (0.,)*7)
        self.driver.move_joints("right", (0.,)*7)
        self.assertFalse(self.driver.engaged)
        self.driver.close()
        self.assertEqual(self.disable_masks(), [3])
        self.assertEqual([(arm.state, arm.commanded_state)
                          for arm in self.driver.get_latest().payload.arms.values()], [(0, -1), (0, -1)])

    def test_sdk_rejected_disable_cannot_confirm_exit(self):
        self.engage()
        self.native.disable_send = False
        self.fail_close()

    def test_accepted_disable_without_new_feedback_is_visible_without_observer(self):
        self.engage()
        self.driver._sink = None
        self.native.disable_feedback = False
        self.fail_close()

    def test_stale_idle_feedback_cannot_confirm_exit(self):
        self.engage()
        p = packet(100, stamp=time.monotonic_ns()-1_000_000_000)
        p.low_speed[:] = [1, 1]
        self.driver._on_feedback(p)
        self.native.disable_feedback = False
        self.fail_close()

    def test_invalid_idle_feedback_cannot_confirm_exit(self):
        for fault in ("counter", "timestamp", "state", "error", "low_speed", "velocity", "nan", "other_arm"):
            with self.subTest(fault=fault):
                driver = TianjiDriver("192.0.2.1", watchdog_s=.2)
                native = FakeSDK(driver)
                driver._sdk = native
                driver._on_feedback(packet())
                driver.configure(fixtures.TianjiFixture.make_profile(("left", "right")))
                driver.engage()
                call = native.call
                before = deepcopy(driver._packet)

                def mutate(name, *args):
                    result = call(name, *args)
                    if name == "disable":
                        p = deepcopy(driver._packet)
                        if fault == "counter":
                            driver._advanced["left"] = before.received_ns
                        elif fault == "timestamp":
                            p.received_ns = before.received_ns
                        elif fault == "state":
                            p.state[0] = 3
                        elif fault == "error":
                            p.error[0] = 2
                        elif fault == "low_speed":
                            p.low_speed[0] = 0
                        elif fault == "velocity":
                            p.dq[0] = 10
                        elif fault == "nan":
                            p.dq[0] = math.nan
                        else:
                            p.state[1] = 3
                        driver._on_feedback(p)
                    return result

                native.call = mutate
                with self.assertLogs("bimanual_teleop.devices.tianji.driver", level="ERROR"):
                    with self.assertRaisesRegex(RuntimeError, "退出停机未确认"):
                        driver.close()
                self.assertIsNone(driver._sdk)

    def test_rejected_disable_does_not_accept_idle_feedback_or_retry_motion(self):
        self.engage()
        call = self.native.call

        def fail_send(name, *args):
            result = call(name, *args)
            if name == "disable":
                raise RuntimeError("SDK disable rejected")
            return result

        self.native.call = fail_send
        self.native.calls.clear()
        self.fail_close()
        self.assertEqual([name for name, _ in self.native.calls], ["disable", "close"])

    def test_previous_target_failure_does_not_prevent_disable(self):
        self.engage()
        self.driver._problem = "previous target failed"
        self.driver.close()
        self.assertEqual(self.disable_masks(), [1])

    def test_busy_slot_is_retried_without_pause_or_target_fallback(self):
        self.engage()
        call = self.native.call
        attempts = []

        def busy(name, *args):
            if name == "disable":
                attempts.append(args)
                if len(attempts) == 1:
                    raise RuntimeError("a datagram is pending")
            return call(name, *args)

        self.native.call = busy
        self.native.calls.clear()
        self.driver.close()
        self.assertEqual(len(attempts), 2)
        self.assertEqual(attempts[0], attempts[1])
        self.assertEqual([name for name, _ in self.native.calls], ["disable", "close"])

    def test_concurrent_close_sends_only_one_disable(self):
        self.engage()
        entered, release = threading.Event(), threading.Event()
        call = self.native.call

        def block(name, *args):
            if name == "close":
                entered.set()
                release.wait(.5)
            return call(name, *args)

        self.native.call = block
        errors = []

        def close():
            try:
                self.driver.close()
            except Exception as error:
                errors.append(error)

        first, second = threading.Thread(target=close), threading.Thread(target=close)
        first.start()
        self.assertTrue(entered.wait(.5))
        second.start()
        release.set()
        first.join(.5)
        second.join(.5)
        self.assertFalse(first.is_alive() or second.is_alive())
        self.assertEqual(errors, [])
        self.assertEqual(self.disable_masks(), [1])
        self.assertEqual([name for name, _ in self.native.calls].count("close"), 1)

    def test_receiver_stays_alive_until_delayed_idle_feedback(self):
        self.engage()
        self.native.disable_feedback = False
        self.driver._receiver = threading.Thread(target=self.driver._receive)
        self.driver._receiver.start()
        call = self.native.call
        delivered = threading.Event()

        def delayed(name, *args):
            result = call(name, *args)
            if name == "disable":
                def deliver():
                    time.sleep(.01)
                    p = packet(100)
                    p.low_speed[:] = [1, 1]
                    self.native.feedback.append(p)
                    delivered.set()
                threading.Thread(target=deliver).start()
            if name == "close":
                self.assertTrue(delivered.is_set())
            return result

        self.native.call = delayed
        self.driver.close()
        self.assertFalse(self.driver._receiver.is_alive())

    def test_close_cancels_engagement_wait_without_rsta_or_late_enable(self):
        self.configure()
        self.native.echo = False
        started, finished = threading.Event(), threading.Event()
        call = self.native.call

        def notice(name, *args):
            result = call(name, *args)
            if name == "engage":
                started.set()
            return result

        self.native.call = notice
        errors = []

        def engage():
            try:
                self.driver.engage()
            except RuntimeError as error:
                errors.append(str(error))
            finally:
                finished.set()

        worker = threading.Thread(target=engage)
        worker.start()
        self.assertTrue(started.wait(.5))
        self.driver.close()
        worker.join(.5)
        self.assertTrue(finished.is_set())
        self.assertEqual(errors, ["Tianji engagement cancelled by shutdown"])
        self.assertNotIn("hold", [name for name, _ in self.native.calls])
        with self.assertRaisesRegex(RuntimeError, "not running"):
            self.driver.engage()

    def test_close_cancels_home_without_a_pause_or_next_target(self):
        self.configure(("left", "right"))
        self.native.complete_joint_move = False
        entered = threading.Event()
        call = self.native.call

        def notice(name, *args):
            result = call(name, *args)
            if name == "move_joints":
                entered.set()
            return result

        self.native.call = notice
        errors = []

        def home():
            try:
                self.driver.move_joints("left", (0.,)*7)
                self.driver.move_joints("right", (0.,)*7)
            except RuntimeError as error:
                errors.append(str(error))

        worker = threading.Thread(target=home)
        worker.start()
        self.assertTrue(entered.wait(.5))
        self.driver.close()
        worker.join(.5)
        self.assertFalse(worker.is_alive())
        self.assertEqual(errors, ["Tianji joint move cancelled"])
        self.assertEqual(self.disable_masks(), [1])
        self.assertNotIn("hold", [name for name, _ in self.native.calls])
        self.assertEqual([args[0] for name, args in self.native.calls if name == "move_joints"], [0])


if __name__ == "__main__":
    unittest.main()
