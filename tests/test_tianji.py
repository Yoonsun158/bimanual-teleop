"""Driver boundary tests with synthetic packets; never connect to a robot."""

from collections import deque
from copy import deepcopy
import ctypes as ct
from dataclasses import asdict
import errno
import json
import math
from pathlib import Path
import sys
import threading
import time
import unittest
from unittest.mock import patch

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bimanual_teleop.devices.tianji.driver import (
    TianjiDriver, TianjiJointCommand, _Feedback, _Send, _Stats, decode_feedback,
)
from bimanual_teleop.types import CommandEvent, CommandStatus, ControlProfile, DeviceCommand, Event


class Sink:
    def __init__(self):
        self.samples, self.events = [], []
        self.accept = True

    def try_publish(self, sample):
        if self.accept:
            self.samples.append(sample)
        return self.accept

    def try_event(self, event):
        self.events.append(event)
        return True


def packet(index=0, sequences=None, stamp=None):
    value = _Feedback()
    value.packet_index = index
    value.received_ns = time.monotonic_ns() if stamp is None else stamp
    value.sequence[:] = sequences or (index, index)
    value.q[:] = [12, -22, 31, -48, 17, 15, -12] * 2
    return value


class FakeNative:
    """Native acceptance does not generate a physical-send receipt by itself."""
    def __init__(self, driver=None, export_text=""):
        self.driver, self.export_text = driver, export_text
        self.feedback, self.sent = deque(), deque()
        self.calls, self.counters = [], _Stats()
        self.version = (100343014, 100343015)
        self.echo = True
        self.hold_failures = 0
        self.stop_nack = False
        self.fail_open = False
        self.reset_rejected = False
        self.reset_without_state_change = False
        self.complete_joint_move = True
        self.send_joint_move = True
        self.send_configure = True

    def call(self, name, *args):
        self.calls.append((name, args))
        if name == "tj_open" and self.fail_open:
            raise RuntimeError("already owned")
        if name == "tj_download_config":
            Path(args[0].decode()).write_text(self.export_text)
        if name == "tj_hold" and self.hold_failures:
            self.hold_failures -= 1
            raise RuntimeError("datagram pending")
        if name == "tj_hold" and self.stop_nack:
            raise RuntimeError("tj_hold: RSTA0 returned 1; inspect feedback")
        if name == "tj_reset_emergency":
            if self.reset_rejected:
                raise RuntimeError("RESET returned 1")
            if not self.reset_without_state_change:
                p = _Feedback.from_buffer_copy(self.driver._packet)
                p.state[args[0]], p.error[args[0]] = 0, 0
                p.sequence[0] += 1
                p.sequence[1] += 1
                p.received_ns = time.monotonic_ns()
                self.driver._on_feedback(p)
        joint_submit = name == "tj_submit" and self.driver._native_move_side is not None
        if (name == "tj_move_joints" or joint_submit) and self.send_joint_move:
            send = _Send()
            send.token, send.attempted, send.result, send.size = args[2] if joint_submit else args[4], 1, 1, 1
            send.sent_ns = time.monotonic_ns()
            self.driver._on_send(send)
        if name == "tj_configure" and self.send_configure:
            send = _Send()
            send.token, send.attempted, send.result, send.size = args[2], 1, 1, 1
            send.sent_ns = time.monotonic_ns()
            self.driver._on_send(send)
        if name == "tj_move_joints" and self.complete_joint_move:
            p = _Feedback.from_buffer_copy(self.driver._packet)
            index = args[0]
            p.sequence[index] += 1
            p.received_ns = time.monotonic_ns()
            # Model controller mode entry overriding a same-packet target.
            target = p.q[index*7:index*7+7] if p.state[index] != 1 else args[1]
            p.state[index], p.low_speed[index] = 1, 1
            p.target[index*7:index*7+7] = [ct.c_float(q).value for q in target]
            self.driver._on_feedback(p)
        if joint_submit and self.complete_joint_move:
            p = _Feedback.from_buffer_copy(self.driver._packet)
            for index in range(2):
                if args[0] & (1 << index):
                    p.sequence[index] += 1
                    p.low_speed[index] = 1
                    p.target[index*7:index*7+7] = [ct.c_float(q).value for q in args[1][index*7:index*7+7]]
            p.received_ns = time.monotonic_ns()
            self.driver._on_feedback(p)
        if self.echo and name in ("tj_configure", "tj_engage", "tj_position_mode"):
            p = _Feedback.from_buffer_copy(self.driver._packet)
            p.packet_index += 1
            p.received_ns = time.monotonic_ns()
            for i in range(2):
                p.sequence[i] += 1
                if not ((1 << args[0]) if name == "tj_position_mode" else args[0]) & (1 << i):
                    continue
                if name == "tj_configure":
                    profile = args[1][i]
                    p.cart_k[i*7:i*7+7] = profile.k
                    p.cart_d[i*7:i*7+7] = profile.d
                    p.tool_pose[i*6:i*6+6] = profile.tool_pose
                    p.tool_dynamics[i*10:i*10+10] = profile.tool_dynamics
                    p.velocity_ratio[i], p.acceleration_ratio[i] = profile.velocity_ratio, profile.acceleration_ratio
                    p.force_type[i] = 1
                elif name == "tj_position_mode":
                    p.state[i], p.low_speed[i] = 1, 1
                    p.velocity_ratio[i], p.acceleration_ratio[i] = 10, 10
                    p.target[i*7:i*7+7] = args[1]
                else:
                    p.state[i], p.impedance_type[i] = 3, 2
            self.driver._on_feedback(p)
        return 0

    def poll_feedback(self):
        return self.feedback.popleft() if self.feedback else None

    def poll_send(self):
        return self.sent.popleft() if self.sent else None

    def versions(self):
        return self.version

    def get_int(self, name):
        return 1017 if name.endswith("Type") else 7

    def stats(self):
        return self.counters


class TianjiDriverTests(unittest.TestCase):
    def setUp(self):
        self.sink = Sink()
        self.driver = TianjiDriver("192.0.2.1", watchdog_s=0.5)
        self.native = FakeNative(self.driver)
        self.driver._native, self.driver._sink = self.native, self.sink
        self.driver._on_feedback(packet())
        self.addCleanup(self.driver.close)

    def make_profile(self, sides=("left",)):
        arm = {"stiffness": [1]*6, "damping": [0.5]*6, "nullspace_stiffness": 1,
               "nullspace_damping": 0.5, "tool_dyn10": [1, 0, 0, 0, .01, 0, 0, .01, 0, .01],
               "velocity_ratio": 100, "acceleration_ratio": 100}
        return ControlProfile("test", "cartesian_impedance", {
            "active_arms": list(sides), "arms": {side: deepcopy(arm) for side in sides}})

    def configure(self, sides=("left",)):
        self.driver.configure(self.make_profile(sides))

    def test_selected_health_ignores_other_arm_fault_and_counter_reset(self):
        p = packet(1, (1, 1))
        p.error[1] = 13
        self.driver._on_feedback(p)
        self.assertTrue(self.driver.health(sides=("left",)).ready)
        self.assertFalse(self.driver.health().ready)
        self.configure(("left",))
        self.assertTrue(self.driver.health().ready)
        reset = packet(2, (2, 0xFFFFFFFF))
        self.driver._on_feedback(reset)
        self.assertIsNone(self.driver._motion_fault)
        self.assertTrue(self.driver.health().ready)

    def position_feedback(self):
        p = _Feedback.from_buffer_copy(self.driver._packet)
        p.state[:] = [1, 1]
        p.sequence[:] = [q+1 for q in p.sequence]
        p.received_ns = time.monotonic_ns()
        self.driver._on_feedback(p)

    def engage(self, sides=("left",)):
        self.configure(sides)
        seed = self.driver.get_latest()
        self.driver.engage()
        return seed

    def command(self, command_id="move", *, expires=None, targets=None):
        now = time.monotonic_ns()
        targets = targets or dict(self.driver._last_targets)
        return DeviceCommand("tianji", command_id, TianjiJointCommand(targets, {}), (), now,
                             now + 100_000_000 if expires is None else expires, "test")

    def events(self, kind):
        return [e for e in self.sink.events if isinstance(e, Event) and e.kind == kind]

    def position_engage(self):
        self.configure()
        p = _Feedback.from_buffer_copy(self.driver._packet)
        p.packet_index += 1
        p.sequence[0] += 1
        p.low_speed[0] = 1
        p.received_ns = time.monotonic_ns()
        self.driver._on_feedback(p)
        self.driver.engage_position()

    def test_native_joint_move_uses_sdk_ratios_and_trajectory_target_completion(self):
        self.configure(("left", "right"))
        target = tuple(math.radians(q) for q in [35.12345, -55, 0, -65, 0, 0, 0])
        result = self.driver.move_joints("right", target)
        mode = next(args for name, args in self.native.calls if name == "tj_move_joints")
        call = next(args for name, args in self.native.calls if name == "tj_submit")
        self.assertEqual((mode[0], mode[2], mode[3]), (1, 100, 100))
        for actual, expected in zip(mode[1], self.driver._packet.q[7:]):
            self.assertAlmostEqual(actual, expected)
        self.assertEqual(call[0], 2)
        self.assertAlmostEqual(call[1][7], 35.12345)
        self.assertIs(result, self.driver.get_latest())
        self.assertNotEqual(result.payload.arms["right"].joints.position_rad, target)
        self.assertFalse(self.driver.engaged)
        self.assertIsNone(self.driver._native_move_side)
        self.assertEqual([name for name, _ in self.native.calls], ["tj_configure", "tj_move_joints", "tj_submit"])

    def test_first_move_from_idle_or_cartesian_survives_mode_entry_target_overwrite(self):
        self.configure(("left", "right"))
        target = tuple(map(math.radians, (35, -55, 0, -65, 0, 0, 0)))
        for side, index in (("left", 0), ("right", 1)):
            for state in (0, 3):
                with self.subTest(side=side, state=state):
                    p = _Feedback.from_buffer_copy(self.driver._packet)
                    p.state[index] = state
                    p.sequence[index] += 1
                    p.received_ns = time.monotonic_ns()
                    self.driver._on_feedback(p)
                    self.native.calls.clear()
                    result = self.driver.move_joints(side, target)
                    self.assertEqual([name for name, _ in self.native.calls], ["tj_move_joints", "tj_submit"])
                    self.assertEqual(result.payload.arms[side].state, 1)
                    self.assertEqual(tuple(map(lambda q: round(math.degrees(q)),
                                               result.payload.arms[side].controller_target_rad)),
                                     (35, -55, 0, -65, 0, 0, 0))

    def test_already_in_position_sends_only_the_final_target(self):
        self.configure()
        self.position_feedback()
        self.native.calls.clear()
        self.driver.move_joints("left", (0,)*7)
        self.assertEqual([name for name, _ in self.native.calls], ["tj_submit"])

    def test_final_target_waits_for_mode_transition_feedback(self):
        self.configure()
        self.native.complete_joint_move = False
        native_call = self.native.call
        states = []

        def call(name, *args):
            if name == "tj_submit":
                self.assertEqual(states, [101, 1])
                self.native.complete_joint_move = True
            return native_call(name, *args)

        def transition(_):
            p = _Feedback.from_buffer_copy(self.driver._packet)
            p.state[0] = 101 if not states else 1
            states.append(p.state[0])
            p.sequence[0] += 1
            p.received_ns = time.monotonic_ns()
            self.driver._on_feedback(p)

        with patch.object(self.native, "call", side_effect=call), \
                patch.object(self.driver._stop, "wait", side_effect=transition):
            self.driver.move_joints("left", (0,)*7)
        self.assertEqual(states, [101, 1])

    def test_failed_position_transition_emits_no_final_motion_target(self):
        self.configure()
        self.native.complete_joint_move = False
        self.driver.engagement_timeout_ns = 10_000_000

        def transitioning(_):
            p = _Feedback.from_buffer_copy(self.driver._packet)
            p.state[0] = 101
            p.sequence[0] += 1
            p.received_ns = time.monotonic_ns()
            self.driver._on_feedback(p)

        with patch.object(self.driver._stop, "wait", side_effect=transitioning):
            with self.assertRaisesRegex(RuntimeError, "position mode was not reported"):
                self.driver.move_joints("left", (0,)*7)
        self.assertFalse(any(name == "tj_submit" for name, _ in self.native.calls))
        self.assertTrue(any(name == "tj_hold" for name, _ in self.native.calls))
        self.assertFalse(self.events("tianji.joint_move_completed"))

    def test_joint_move_waits_for_final_internal_target_and_low_speed_without_stream_expiry(self):
        self.configure()
        self.position_feedback()
        self.native.complete_joint_move = False
        self.driver._deadline_ns = 0
        final_target = [35, -55, 0, -65, 0, 0, 0]
        steps = []

        def progress(_):
            p = _Feedback.from_buffer_copy(self.driver._packet)
            p.sequence[0] += 1
            p.received_ns = time.monotonic_ns()
            p.state[0] = 1
            p.dq[:7] = [100]*7
            p.low_speed[0] = 0 if len(steps) == 1 else 1
            p.target[:7] = [0]*7 if not steps else final_target
            self.driver._on_feedback(p)
            steps.append(p)

        with patch.object(self.driver._stop, "wait", side_effect=progress):
            result = self.driver.move_joints("left", tuple(map(math.radians, final_target)))
        self.assertEqual(len(steps), 3)
        self.assertEqual(result.payload.arms["left"].low_speed, 1)
        self.assertFalse(any(name == "tj_hold" for name, _ in self.native.calls))

    def test_already_at_target_waits_for_own_send_and_subsequent_feedback(self):
        self.configure()
        self.position_feedback()
        self.native.send_joint_move = False
        steps = []

        def progress(_):
            if len(steps) == 1:
                sent = _Send()
                sent.token = self.driver._token
                sent.sent_ns, sent.attempted, sent.result, sent.size = time.monotonic_ns(), 1, 1, 1
                self.driver._on_send(sent)
            else:
                p = _Feedback.from_buffer_copy(self.driver._packet)
                p.sequence[0] += 1
                p.received_ns = time.monotonic_ns()
                self.driver._on_feedback(p)
            steps.append(True)

        with patch.object(self.driver._stop, "wait", side_effect=progress):
            self.driver.move_joints("left", self.driver.get_latest().payload.arms["left"].joints.position_rad)
        self.assertEqual(len(steps), 3)

    def test_failed_native_move_send_cannot_complete_from_matching_feedback(self):
        self.configure()
        self.position_feedback()
        self.native.send_joint_move = False

        def fail_send(_):
            sent = _Send()
            sent.token = self.driver._token
            sent.sent_ns, sent.attempted, sent.result, sent.size = time.monotonic_ns(), 1, -1, 1
            sent.error_number = errno.EIO
            self.driver._on_send(sent)

        with patch.object(self.driver._stop, "wait", side_effect=fail_send):
            with self.assertRaisesRegex(RuntimeError, "UDP send failed"):
                self.driver.move_joints("left", (0,)*7)
        self.assertFalse(self.events("tianji.joint_move_completed"))
        self.assertTrue(any(name == "tj_hold" for name, _ in self.native.calls))

    def test_missing_move_send_receipt_fails_even_when_feedback_keeps_advancing(self):
        self.configure()
        self.position_feedback()
        self.native.send_joint_move = False
        self.driver.watchdog_ns = 20_000_000
        feedback_count = []

        def feedback(_):
            time.sleep(.001)
            p = _Feedback.from_buffer_copy(self.driver._packet)
            p.sequence[0] += 1
            p.received_ns = time.monotonic_ns()
            self.driver._on_feedback(p)
            feedback_count.append(p.sequence[0])

        with patch.object(self.driver._stop, "wait", side_effect=feedback):
            with self.assertRaisesRegex(RuntimeError, "send result missing"):
                self.driver.move_joints("left", (0,)*7)
        self.assertTrue(feedback_count)
        self.assertFalse(self.events("tianji.joint_move_completed"))
        self.assertTrue(any(name == "tj_hold" for name, _ in self.native.calls))

    def test_interrupted_native_joint_move_stops_only_moving_arm_with_position_fallback(self):
        self.configure(("left", "right"))
        self.native.stop_nack = True
        native_call = self.native.call

        def interrupt(name, *args):
            result = native_call(name, *args)
            if name == "tj_move_joints" and len([1 for n, _ in self.native.calls if n == name]) == 1:
                raise KeyboardInterrupt
            return result

        with patch.object(self.native, "call", side_effect=interrupt), self.assertRaises(KeyboardInterrupt):
            self.driver.move_joints("right", (0,)*7)
        self.assertEqual([args[0] for name, args in self.native.calls if name == "tj_hold"], [2])
        fallback = [args for name, args in self.native.calls if name == "tj_move_joints"][-1]
        self.assertEqual((fallback[0], fallback[2], fallback[3]), (1, 100, 100))
        for actual, expected in zip(fallback[1], self.driver._packet.q[7:]):
            self.assertAlmostEqual(actual, expected)
        self.assertIsNone(self.driver._hold_reason)
        self.assertFalse(self.driver.engaged)

    def test_interrupting_final_target_stops_after_successful_mode_transition(self):
        self.configure(("left", "right"))
        native_call = self.native.call

        def interrupt(name, *args):
            result = native_call(name, *args)
            if name == "tj_submit":
                raise KeyboardInterrupt
            return result

        with patch.object(self.native, "call", side_effect=interrupt), self.assertRaises(KeyboardInterrupt):
            self.driver.move_joints("right", (0,)*7)
        self.assertEqual([args[0] for name, args in self.native.calls if name == "tj_hold"], [2])
        self.assertFalse(self.events("tianji.joint_move_completed"))
        self.assertFalse(self.driver.engaged)

    def test_native_joint_move_rejects_nonfinite_and_real_controller_fault(self):
        self.configure()
        for target in ((0,)*6, (float("nan"),)*7):
            with self.assertRaises(ValueError):
                self.driver.move_joints("left", target)
        self.driver._packet.error[0] = 13
        self.driver._on_feedback(self.driver._packet)
        with self.assertRaisesRegex(RuntimeError, "controller error 13"):
            self.driver.move_joints("left", (0,)*7)
        self.assertFalse(any(name == "tj_move_joints" for name, _ in self.native.calls))

    def test_observer_rejection_latches_fault_and_blocks_next_target(self):
        self.engage()
        self.sink.accept = False
        self.sink.try_event = lambda _: False
        self.driver._on_feedback(packet(1))
        self.assertFalse(self.driver.health().ready)
        self.assertIn("observer_error", self.driver.metadata)
        result = self.driver.submit(self.command())
        self.assertFalse(result.accepted)
        self.assertFalse(any(name == "tj_submit" for name, _ in self.native.calls))

    def test_position_seed_is_measured_and_target_has_no_extra_speed_or_model_gate(self):
        self.position_engage()
        call = next(args for name, args in self.native.calls if name == "tj_move_joints")
        self.assertEqual(call[0], 0)
        for actual, expected in zip(call[1], [12, -22, 31, -48, 17, 15, -12]):
            self.assertAlmostEqual(actual, expected)
        q = list(self.driver._last_targets["left"])
        q[1] += 10
        self.assertTrue(self.driver.submit(self.command(targets={"left": q})).accepted)
        self.assertTrue(self.driver.engaged)
        self.assertFalse(any(name == "tj_hold" for name, _ in self.native.calls))

    def test_position_to_cartesian_does_not_require_stationary_receipt(self):
        self.position_engage()
        with self.assertRaisesRegex(RuntimeError, "already engaged"):
            self.driver.engage()
        self.driver.request_hold("joint preset complete")
        p = _Feedback.from_buffer_copy(self.driver._packet)
        p.sequence[0] += 1
        p.received_ns = time.monotonic_ns()
        p.low_speed[0], p.dq[0] = 0, 2
        self.driver._on_feedback(p)
        self.driver.engage()
        self.assertEqual(self.driver._control_mode, "cartesian")
        self.assertTrue(self.driver.health().ready)

    def test_position_mode_accepts_existing_position_and_dual_arm_profile(self):
        self.configure(("left", "right"))
        p = _Feedback.from_buffer_copy(self.driver._packet)
        p.sequence[:] = [v+1 for v in p.sequence]
        p.received_ns = time.monotonic_ns()
        p.state[:], p.low_speed[:], p.dq[:] = [1, 3], [0, 0], [2]*14
        self.driver._on_feedback(p)
        self.driver.engage_position()
        self.assertTrue(self.driver.engaged)
        self.assertEqual([args[0] for name, args in self.native.calls if name == "tj_move_joints"], [0, 1])

    def test_position_feedback_mode_change_stops_and_target_expiry_is_enforced(self):
        self.position_engage()
        self.assertFalse(self.driver.submit(self.command(expires=time.monotonic_ns()-1)).accepted)
        self.assertFalse(self.driver.engaged)
        p = _Feedback.from_buffer_copy(self.driver._packet)
        p.sequence[0] += 1
        p.received_ns = time.monotonic_ns()
        self.driver._on_feedback(p)
        self.driver.engage_position()
        p.state[0] = 2
        p.sequence[0] += 2
        p.received_ns = time.monotonic_ns()
        self.driver._on_feedback(p)
        self.assertFalse(self.driver.submit(self.command()).accepted)
        self.assertFalse(self.driver.engaged)

    def test_rejected_position_stop_replaces_measured_target(self):
        self.position_engage()
        self.native.stop_nack = True
        self.driver.request_hold("complete")
        self.assertIsNone(self.driver._hold_reason)
        self.assertFalse(self.driver._held_feedback)
        self.assertEqual(len(self.events("tianji.position_hold_target_replaced")), 1)
        call = [args for name, args in self.native.calls if name == "tj_move_joints"][-1]
        for actual, expected in zip(call[1], self.driver._packet.q[:7]):
            self.assertAlmostEqual(actual, expected)


    def set_emergency_feedback(self, errors=(13, 13)):
        p = packet(self.driver._packet.packet_index+1)
        p.sequence[:] = [s+1 for s in self.driver._packet.sequence]
        p.error[:] = errors
        p.state[:] = [100 if e else 0 for e in errors]
        p.low_speed[:] = [1, 1]
        self.driver._on_feedback(p)

    def feed_latest_until(self, stop):
        while not stop.wait(.005):
            p = _Feedback.from_buffer_copy(self.driver._packet)
            p.sequence[:] = [s+1 for s in p.sequence]
            p.received_ns = time.monotonic_ns()
            self.driver._on_feedback(p)

    def test_emergency_reset_requires_operator_confirmation_and_does_not_enable(self):
        self.set_emergency_feedback()
        with self.assertRaisesRegex(ValueError, "explicitly confirmed"):
            self.driver.reset_released_emergency()
        stop = threading.Event()
        worker = threading.Thread(target=self.feed_latest_until, args=(stop,))
        worker.start()
        try:
            result = self.driver.reset_released_emergency(physical_release_confirmed=True)
        finally:
            stop.set()
            worker.join()
        self.assertTrue(result["confirmed_disabled"])
        self.assertEqual(result["requested_arms"], ["left", "right"])
        self.assertEqual([args[0] for name, args in self.native.calls if name == "tj_reset_emergency"], [0, 1])
        self.assertFalse(self.driver.engaged)
        self.assertIsNone(self.driver.profile)
        self.assertFalse(any(name in ("tj_engage", "tj_position_mode", "tj_submit", "tj_configure")
                             for name, _ in self.native.calls))

    def test_emergency_reset_skips_healthy_disabled_arm(self):
        self.set_emergency_feedback((13, 0))
        stop = threading.Event()
        worker = threading.Thread(target=self.feed_latest_until, args=(stop,))
        worker.start()
        try:
            result = self.driver.reset_released_emergency(physical_release_confirmed=True)
        finally:
            stop.set()
            worker.join()
        self.assertEqual(result["requested_arms"], ["left"])
        self.assertEqual(len([1 for name, _ in self.native.calls if name == "tj_reset_emergency"]), 1)

    def test_emergency_reset_rejects_unrelated_fault_invalid_and_stale_feedback(self):
        for problem in ("unrelated", "invalid", "stale"):
            with self.subTest(problem=problem):
                self.set_emergency_feedback((2 if problem == "unrelated" else 13, 13))
                if problem == "invalid":
                    p = _Feedback.from_buffer_copy(self.driver._packet)
                    p.dq[0] = float("nan")
                    p.sequence[:] = [s+1 for s in p.sequence]
                    self.driver._on_feedback(p)
                if problem == "stale":
                    self.driver._advanced["left"] = 0
                before = len(self.native.calls)
                with self.assertRaises(RuntimeError):
                    self.driver.reset_released_emergency(physical_release_confirmed=True)
                self.assertFalse(any(name == "tj_reset_emergency" for name, _ in self.native.calls[before:]))

    def test_rejected_emergency_reset_is_not_retried(self):
        self.set_emergency_feedback()
        self.native.reset_rejected = True
        with self.assertRaisesRegex(RuntimeError, "returned 1"):
            self.driver.reset_released_emergency(physical_release_confirmed=True)
        self.assertEqual(len([1 for name, _ in self.native.calls if name == "tj_reset_emergency"]), 1)
        self.assertFalse(self.driver.engaged)

    def test_emergency_reset_ack_without_recovered_feedback_cannot_enable(self):
        self.set_emergency_feedback()
        self.native.reset_without_state_change = True
        stop = threading.Event()
        worker = threading.Thread(target=self.feed_latest_until, args=(stop,))
        worker.start()
        try:
            with self.assertRaisesRegex(RuntimeError, "did not produce"):
                self.driver.reset_released_emergency(physical_release_confirmed=True)
        finally:
            stop.set()
            worker.join()
        self.assertFalse(self.driver.engaged)
        self.assertIsNone(self.driver.profile)

    def test_adoption_applies_profile_without_old_target_or_stationary_receipts(self):
        profile = self.make_profile(("left", "right"))
        p = _Feedback.from_buffer_copy(self.driver._packet)
        p.state[:], p.low_speed[:], p.dq[:] = [1, 3], [0, 0], [2]*14
        p.target[:] = [80]*14
        p.received_ns = time.monotonic_ns()
        self.driver._on_feedback(p)
        self.driver.adopt_stationary_control(profile)
        self.assertEqual([name for name, _ in self.native.calls], ["tj_configure"])
        self.driver.adopt_stationary_position(profile, (0,)*7)
        self.assertEqual([name for name, _ in self.native.calls], ["tj_configure", "tj_configure"])
        self.assertFalse(self.driver.engaged)
        self.driver.engage()
        self.assertTrue(self.driver.engaged)


    def test_feedback_units_partial_invalid_and_no_derived_or_fixed_fields(self):
        p = packet(8)
        p.dq[0], p.current[0], p.torque[0], p.external_torque[0] = 180, 240, 2, 3
        p.target[0], p.q[8] = 90, float("nan")
        sample = decode_feedback(p, "run")
        arm = sample.payload.arms["left"]
        self.assertAlmostEqual(arm.joints.velocity_rad_s[0], math.pi)
        self.assertEqual(arm.native_current_permille[0], 240)
        self.assertIsNone(arm.joints.motor_current_a)
        self.assertEqual(arm.joints.measured_torque_nm[0], 2)
        self.assertEqual(arm.joints.estimated_external_torque_nm[0], 3)
        self.assertAlmostEqual(arm.controller_target_rad[0], math.pi/2)
        self.assertIsNone(sample.payload.arms["right"].joints.position_rad[1])
        self.assertIsNone(sample.header.source_time)
        self.assertFalse(sample.header.valid)
        encoded = json.dumps(asdict(sample), allow_nan=False)
        self.assertNotIn("external_wrench", encoded)
        self.assertNotIn("stiffness", encoded)
        self.driver._on_feedback(p)
        self.assertEqual(len(self.events("tianji.invalid_feedback")), 1)

    def test_every_packet_published_latest_does_not_publish(self):
        self.native.feedback.extend([packet(i) for i in range(1, 7)])
        self.driver._drain()
        feedback = [s for s in self.sink.samples if s.header.ref.stream == "tianji.feedback"]
        self.assertEqual([s.payload.packet_index for s in feedback], list(range(7)))
        before = len(self.sink.samples)
        a, b = self.driver.get_latest(), self.driver.get_latest()
        self.assertIs(a, b)
        self.assertEqual(len(self.sink.samples), before)
        self.assertEqual(len(self.events("tianji.reported_config")), 2)

    def test_independent_source_gap_wrap_stall_and_reset(self):
        self.driver._sequences = {"left": 999999, "right": 10}
        now = time.monotonic_ns()
        self.driver._advanced = {"left": now, "right": now - 1_000_000_000}
        self.driver._on_feedback(packet(1, (0, 10), now))
        self.assertFalse(self.driver.health().ready)
        self.assertIn("right", self.driver.health().detail)
        self.assertFalse(self.events("tianji.source_gap"))
        self.driver._on_feedback(packet(2, (3, 11)))
        self.assertEqual(self.events("tianji.source_gap")[0].details["missing"], 2)
        advanced = self.driver._advanced["left"]
        self.driver._on_feedback(packet(3, (1, 12)))
        self.assertEqual(self.driver._advanced["left"], advanced)
        self.assertIn("re-engagement", self.driver._motion_fault)
        self.assertEqual(len(self.events("tianji.source_reset")), 1)

    def test_native_overflow_and_observer_rejection_are_visible(self):
        stats = _Stats()
        stats.feedback_dropped, stats.first_feedback_lost, stats.last_feedback_lost = 2, 4, 9
        self.driver._check_stats(stats)
        event = self.events("tianji.native_queue_overflow")[0]
        self.assertEqual(event.details["new_losses"], 2)
        self.assertTrue(event.details["bounds_are_not_contiguous_loss_interval"])
        self.assertTrue(self.driver.health().ready)
        self.assertIn("overflow", self.driver.metadata["queue_overflow"])
        self.sink.accept = False
        self.driver._on_feedback(packet(1))
        self.driver._on_feedback(packet(2))
        self.assertFalse(self.events("tianji.recording_gap"))
        self.assertFalse(self.driver.health().ready)
        self.assertIn("rejected", self.driver.metadata["observer_error"])

    def test_engage_requires_applied_profile_without_idle_gate(self):
        with self.assertRaisesRegex(RuntimeError, "profile"):
            self.driver.engage()
        self.driver._packet.state[0] = 3
        self.driver._on_feedback(self.driver._packet)
        self.configure()
        self.driver.engage()
        self.assertTrue(self.driver.engaged)

    def test_configure_does_not_download_or_require_matching_versions_or_echo(self):
        self.native.version = (100343014, 100344001)
        self.native.echo = False
        self.configure()
        self.assertEqual([name for name, _ in self.native.calls], ["tj_configure"])
        self.assertIsNotNone(self.driver.profile)

    def test_configure_finishes_send_before_subsequent_joint_move(self):
        self.native.send_configure = False
        sent = []
        native_call = self.native.call

        def call(name, *args):
            if name == "tj_move_joints":
                self.assertEqual(sent, ["configuration sent"])
            return native_call(name, *args)

        def finish_send(_):
            packet = _Send()
            packet.token, packet.attempted, packet.result, packet.size = self.driver._token, 1, 1, 1
            packet.sent_ns = time.monotonic_ns()
            self.driver._on_send(packet)
            sent.append("configuration sent")

        with patch.object(self.native, "call", side_effect=call):
            with patch.object(self.driver._stop, "wait", side_effect=finish_send):
                self.configure()
            self.driver.move_joints("left", (0,)*7)

    def test_dual_position_seeds_use_distinct_tokens_and_wait_for_pending_send(self):
        self.configure(("left", "right"))
        self.native.send_joint_move = False
        native_call = self.native.call
        pending = []
        tokens = []

        def call(name, *args):
            if name == "tj_move_joints":
                self.assertFalse(pending)
                pending.append(args[4])
                tokens.append(args[4])
            return native_call(name, *args)

        def finish_send(_):
            packet = _Send()
            packet.token, packet.attempted, packet.result, packet.size = pending.pop(), 1, 1, 1
            packet.sent_ns = time.monotonic_ns()
            self.driver._on_send(packet)

        with patch.object(self.native, "call", side_effect=call), \
             patch.object(self.driver._stop, "wait", side_effect=finish_send):
            self.driver.engage_position()
        self.assertEqual(len(tokens), 2)
        self.assertLess(tokens[0], tokens[1])
        self.assertFalse(pending)

    def test_requested_configuration_is_recorded(self):
        profile = self.make_profile()
        self.driver.configure(profile)
        event = self.events("tianji.configure_requested")[0]
        self.assertEqual(event.details["profile"], asdict(profile))
        self.assertEqual(self.driver.metadata["control_profile"], asdict(profile))
        self.assertTrue(self.events("tianji.configure_sdk_returned"))

    def test_verified_empty_load_reaches_native_configuration_and_readback(self):
        profile = self.make_profile(("left", "right"))
        for arm in profile.parameters["arms"].values():
            arm["tool_dyn10"] = [0]*10
        self.driver.configure(profile)
        native_profiles = next(args[1] for name, args in self.native.calls if name == "tj_configure")
        for index, side in enumerate(("left", "right")):
            self.assertEqual(tuple(native_profiles[index].tool_dynamics), (0.0,)*10)
            self.assertEqual(self.driver.profile.arms[side].tool_dyn10, (0.0,)*10)
        self.assertTrue(self.events("tianji.configure_sdk_returned"))
        self.assertFalse(self.driver.engaged)

    def test_measured_group_seed_exact_sample_and_acceptance_not_sent(self):
        seed = self.engage(("left", "right"))
        self.assertIs(self.driver.engagement_sample, seed)
        call = next(args for name, args in self.native.calls if name == "tj_engage")
        self.assertEqual(call[0], 3)
        for actual, expected in zip(call[1], [12, -22, 31, -48, 17, 15, -12]*2):
            self.assertAlmostEqual(actual, expected)
        self.assertTrue(self.driver.engaged)
        self.assertTrue(any(isinstance(e, CommandEvent) and e.status == CommandStatus.ACCEPTED for e in self.sink.events))
        self.assertFalse(any(isinstance(e, CommandEvent) and e.command_id.startswith("engage-")
                             and e.status == CommandStatus.SENT for e in self.sink.events))

    def test_expired_target_not_native_accepted(self):
        self.engage()
        result = self.driver.submit(self.command(expires=time.monotonic_ns()-1))
        self.assertFalse(result.accepted)
        self.assertFalse(any(name == "tj_submit" for name, _ in self.native.calls))

    def test_cartesian_command_has_no_host_joint_speed_cap(self):
        self.engage()
        q = list(self.driver._last_targets["left"])
        q[0] += .2
        result = self.driver.submit(self.command(targets={"left": tuple(q)}))
        self.assertTrue(result.accepted, result.reason)
        self.assertTrue(any(name == "tj_submit" for name, _ in self.native.calls))
        self.assertFalse(any(name == "tj_hold" for name, _ in self.native.calls))

    def test_cartesian_command_rejects_nonfinite_joints(self):
        self.engage()
        q = list(self.driver._last_targets["left"])
        q[0] = math.nan
        result = self.driver.submit(self.command(targets={"left": tuple(q)}))
        self.assertFalse(result.accepted)
        self.assertIn("finite", result.reason)

    def test_send_receipt_and_expiry_distinguish_attempted(self):
        self.engage()
        self.assertTrue(self.driver.submit(self.command()).accepted)
        token = self.driver._token
        p = _Send()
        p.token, p.sent_ns, p.packet_index, p.attempted, p.result, p.size = token, time.monotonic_ns(), 0, 1, 3, 3
        p.payload[:3] = b"abc"
        self.driver._on_send(p)
        self.assertTrue(any(isinstance(e, CommandEvent) and e.command_id == "move" and e.status == CommandStatus.SENT for e in self.sink.events))
        p.packet_index, p.attempted, p.result, p.error_number = 1, 0, -1, errno.ETIMEDOUT
        self.driver._on_send(p)
        self.assertEqual(len(self.events("tianji.command_expired")), 1)
        self.assertFalse(self.driver.health().ready)
        self.assertEqual(self.driver._send_results[token],
                         f"Tianji UDP send failed (token {token}, errno {errno.ETIMEDOUT})")

    def test_command_payload_snapshot_and_native_expiry_bound(self):
        self.engage()
        targets = dict(self.driver._last_targets)
        command = self.command(targets=targets, expires=time.monotonic_ns()+5_000_000_000)
        self.assertTrue(self.driver.submit(command).accepted)
        snapshot = self.driver._last_targets["left"]
        targets["left"] = (99,)*7
        self.assertNotEqual(snapshot, targets["left"])
        call = [args for name, args in self.native.calls if name == "tj_submit"][-1]
        self.assertLess(call[3], command.expires_monotonic_ns)

    def test_hold_records_sdk_return_without_blocking_resume_on_stationary_receipt(self):
        self.engage()
        self.driver.request_hold("test pause")
        self.assertFalse(self.driver._held_feedback)
        self.assertFalse(self.events("tianji.hold_sdk_returned")[-1].details["physical_stop_confirmed"])
        self.driver.engage()
        self.assertTrue(self.driver.engaged)

    def test_engage_missing_mode_echo_times_out_and_holds(self):
        self.configure()
        self.native.echo = False
        self.driver.engagement_timeout_ns = 20_000_000
        with self.assertRaisesRegex(RuntimeError, "confirm.*engagement"):
            self.driver.engage()
        self.assertFalse(self.driver.engaged)
        self.assertTrue(any(name == "tj_hold" for name, _ in self.native.calls))

    def test_startup_grace_accepts_delayed_mode_with_fresh_feedback(self):
        self.configure()
        self.native.echo = False
        self.driver.watchdog_ns = 30_000_000
        self.driver.engagement_timeout_ns = 500_000_000
        published = threading.Event()
        finish = threading.Event()

        def feedback():
            while not finish.is_set():
                if any(name == "tj_engage" for name, _ in self.native.calls):
                    break
                finish.wait(.001)
            ready_at = time.monotonic() + .10
            while not finish.is_set():
                p = _Feedback.from_buffer_copy(self.driver._packet)
                p.received_ns, p.packet_index = time.monotonic_ns(), p.packet_index + 1
                p.sequence[0] += 1
                p.state[0] = 3 if time.monotonic() >= ready_at else 103
                p.impedance_type[0] = 2
                self.driver._on_feedback(p)
                published.set()
                finish.wait(.002)

        worker = threading.Thread(target=feedback)
        worker.start()
        self.driver._watchdog = threading.Thread(target=self.driver._watch)
        self.driver._watchdog.start()
        try:
            self.driver.engage()
            self.assertTrue(published.is_set())
            self.assertTrue(self.driver.engaged)
            report = self.events("tianji.engagement_mode_reported")[-1]
            self.assertGreater(report.details["elapsed_ms"], 30)
            self.assertLessEqual(self.driver._deadline_ns - report.observed_monotonic_ns, 30_000_000)
            engage_call = next(args for name, args in self.native.calls if name == "tj_engage")
            self.assertLessEqual(engage_call[3] - report.observed_monotonic_ns, 40_000_000)
            # Fresh feedback alone does not extend the runtime target deadline.
            deadline = time.monotonic() + .3
            while self.driver.engaged and time.monotonic() < deadline:
                time.sleep(.002)
            self.assertFalse(self.driver.engaged)
            self.assertTrue(self.events("tianji.hold_requested"))
        finally:
            finish.set()
            worker.join()

    def test_startup_grace_does_not_allow_stale_feedback(self):
        self.configure()
        self.native.echo = False
        self.driver.watchdog_ns = 20_000_000
        self.driver.engagement_timeout_ns = 500_000_000
        self.driver._watchdog = threading.Thread(target=self.driver._watch)
        self.driver._watchdog.start()
        started = time.monotonic()
        with self.assertRaisesRegex(RuntimeError, "confirm.*engagement"):
            self.driver.engage()
        self.assertLess(time.monotonic() - started, .4)
        self.assertIn("source counter", self.events("tianji.hold_requested")[0].details["reason"])

    def test_transition_low_speed_flag_cannot_confirm_hold(self):
        self.engage()
        self.driver.request_hold("test pause during transition")
        for state in (103, 3, 103):
            p = _Feedback.from_buffer_copy(self.driver._packet)
            p.received_ns, p.packet_index = time.monotonic_ns(), p.packet_index + 1
            p.sequence[0] += 1
            p.state[0], p.low_speed[0] = state, 1
            self.driver._on_feedback(p)
            self.assertEqual(self.driver._held_feedback, state == 3)
        self.assertEqual(len(self.events("tianji.hold_low_speed_observed")), 1)

    def test_reported_configuration_change_is_recorded_without_stopping(self):
        self.engage()
        p = _Feedback.from_buffer_copy(self.driver._packet)
        p.received_ns, p.packet_index = time.monotonic_ns(), p.packet_index+1
        p.sequence[0] += 1
        p.cart_k[0] += 1
        self.driver._on_feedback(p)
        result = self.driver.submit(self.command())
        self.assertTrue(result.accepted)
        self.assertTrue(self.driver.engaged)
        self.assertTrue(self.events("tianji.reported_config"))

    def test_invalid_measured_velocity_prevents_another_target(self):
        self.engage()
        p = _Feedback.from_buffer_copy(self.driver._packet)
        p.received_ns, p.packet_index = time.monotonic_ns(), p.packet_index + 1
        p.sequence[0] += 1
        p.dq[0] = float("nan")
        self.driver._on_feedback(p)
        result = self.driver.submit(self.command())
        self.assertFalse(result.accepted)
        self.assertIn("position/velocity", result.reason)
        self.assertFalse(self.driver.engaged)

    def test_hold_pending_is_retried_without_touching_other_arm(self):
        self.engage()
        self.native.hold_failures = 1
        self.driver.request_hold("test")
        self.assertIsNotNone(self.driver._hold_reason)
        self.driver.request_hold("test")
        self.assertIsNone(self.driver._hold_reason)
        self.assertEqual([args[0] for name, args in self.native.calls if name == "tj_hold"], [1, 1])

    def test_close_requests_hold_before_native_close(self):
        self.engage()
        self.driver.watchdog_ns = 10_000_000
        self.driver.close()
        names = [name for name, args in self.native.calls]
        self.assertLess(names.index("tj_hold"), names.index("tj_close"))
        self.assertFalse(self.driver.engaged)
        self.assertTrue(self.events("tianji.hold_unconfirmed"))

    def test_worker_watchdog_holds_without_a_new_submit(self):
        self.engage()
        self.driver._deadline_ns = time.monotonic_ns() - 1
        self.driver._watchdog = threading.Thread(target=self.driver._watch)
        self.driver._watchdog.start()
        deadline = time.monotonic() + .2
        while self.driver.engaged and time.monotonic() < deadline:
            time.sleep(.001)
        self.assertFalse(self.driver.engaged)
        self.assertEqual([args[0] for name, args in self.native.calls if name == "tj_hold"], [1])

    def test_receiver_batch_does_not_starve_send_receipts(self):
        self.native.feedback.extend(packet(i) for i in range(1, 601))
        sent = _Send()
        sent.attempted, sent.result, sent.size = 1, 3, 3
        sent.sent_ns = time.monotonic_ns()
        self.native.sent.append(sent)
        self.assertEqual(self.driver._drain(), 257)
        self.assertEqual(len(self.native.feedback), 344)
        self.assertEqual(self.driver._send_results[sent.token], sent.sent_ns)
        self.driver.close()
        self.assertFalse(self.native.feedback)

    def test_expiry_fault_can_reengage_after_observed_hold(self):
        self.engage()
        sent = _Send()
        sent.sent_ns, sent.result, sent.error_number = time.monotonic_ns(), -1, errno.ETIMEDOUT
        self.driver._on_send(sent)
        self.driver.request_hold("expiry")
        self.assertIsNone(self.driver._problem)
        p = _Feedback.from_buffer_copy(self.driver._packet)
        p.received_ns, p.packet_index = time.monotonic_ns(), p.packet_index+1
        p.sequence[0] += 1
        p.low_speed[0] = 1
        self.driver._on_feedback(p)
        self.driver.engage()
        self.assertTrue(self.driver.engaged)


    def test_start_is_capture_only_and_worker_drains_packets(self):
        driver = TianjiDriver("192.0.2.2")
        native = FakeNative(driver)
        native.feedback.extend(packet(i) for i in range(4))
        sink = Sink()
        with patch("bimanual_teleop.devices.tianji.driver._NativeBridge", return_value=native):
            driver.start(sink)
        deadline = time.monotonic() + .5
        while len(sink.samples) < 4 and time.monotonic() < deadline:
            time.sleep(.001)
        driver.close()
        self.assertEqual(len(sink.samples), 4)
        self.assertEqual([name for name, _ in native.calls], ["tj_open", "tj_close"])

    def test_failed_open_does_not_close_some_other_owner(self):
        driver = TianjiDriver("192.0.2.2")
        native = FakeNative(driver)
        native.fail_open = True
        with patch("bimanual_teleop.devices.tianji.driver._NativeBridge", return_value=native):
            with self.assertRaisesRegex(RuntimeError, "owned"):
                driver.start()
        self.assertEqual([name for name, _ in native.calls], ["tj_open"])


if __name__ == "__main__":
    unittest.main()
