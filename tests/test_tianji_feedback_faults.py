"""Feedback classification and pause semantics with synthetic packets only."""

from copy import deepcopy
from dataclasses import replace
import math
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from bimanual_teleop.devices.tianji.driver import (
    TianjiDriver, TianjiJointCommand, FeedbackSnapshot, classify_feedback, decode_feedback,
)
from bimanual_teleop.system import SystemState
from bimanual_teleop.types import DeviceCommand
from tests.support.quest import RuntimeFixture
from tests.support.tianji import FakeSDK, Sink, packet


def changed_arm(sample, side, channel, index=2, value=None):
    arm = sample.payload.arms[side]
    if channel in ("q", "dq", "torque", "external_torque"):
        field = {"q": "position_rad", "dq": "velocity_rad_s", "torque": "measured_torque_nm",
                 "external_torque": "estimated_external_torque_nm"}[channel]
        values = list(getattr(arm.joints, field) or (0.,) * 7)
        values[index] = value
        arm = replace(arm, joints=replace(arm.joints, **{field: tuple(values)}))
    elif channel in ("target", "current"):
        field = "controller_target_rad" if channel == "target" else "native_current_permille"
        values = list(getattr(arm, field))
        values[index] = value
        arm = replace(arm, **{field: tuple(values)})
    else:
        arm = replace(arm, **{channel: value})
    return replace(sample, payload=replace(sample.payload, arms={**sample.payload.arms, side: arm}))


class FeedbackClassificationTests(unittest.TestCase):
    def test_optional_observations_remain_missing_without_invalidating_motion(self):
        p = packet(17, (34, 35), 1_234_567)
        p.torque[1], p.external_torque[9], p.current[13] = math.nan, math.inf, -math.inf
        sample = decode_feedback(p, "run")
        self.assertTrue(sample.header.valid)
        self.assertIsNone(sample.payload.arms["left"].joints.measured_torque_nm[1])
        self.assertIsNone(sample.payload.arms["right"].joints.estimated_external_torque_nm[2])
        self.assertIsNone(sample.payload.arms["right"].native_current_permille[6])
        assessment = classify_feedback(sample)
        self.assertIsNone(assessment.control_problem)
        self.assertEqual([(i.side, i.channel, i.joint_indices) for i in assessment.observation_issues],
                         [("left", "torque", (2,)), ("right", "external_torque", (3,)),
                          ("right", "current", (7,))])

    def test_required_channels_each_invalidate_motion_and_identify_joint_and_packet(self):
        for channel in ("q", "dq", "target"):
            with self.subTest(channel=channel):
                p = packet(17, (34, 35), 1_234_567)
                p.input_sequence[:] = (19, 20)
                getattr(p, channel)[9] = math.nan
                sample = decode_feedback(p, "run")
                self.assertFalse(sample.header.valid)
                assessment = classify_feedback(sample)
                issue, = assessment.control_issues
                self.assertEqual((issue.side, issue.channel, issue.joint_indices), ("right", channel, (3,)))
                reason = assessment.control_problem
                for detail in (f"right invalid {channel} at J3", "packet_index=17",
                               "received_monotonic_ns=1234567", "ref=tianji.feedback/run/17",
                               "right sequence=35 input_sequence=20"):
                    self.assertIn(detail, reason)
                self.assertIsNone(classify_feedback(sample, ("left",)).control_problem)

    def test_controller_error_reports_decimal_hex_side_and_independent_invalid_channel(self):
        p = packet(8, stamp=30)
        p.error[0], p.error[1], p.target[10] = 13, 100, math.inf
        assessment = classify_feedback(decode_feedback(p, "fault"))
        self.assertIn("left controller error 13 (0x0000000D)", assessment.control_problem)
        self.assertIn("right controller error 100 (0x00000064)", assessment.control_problem)
        self.assertIn("right invalid target at J4", assessment.control_problem)

    def test_direct_samples_with_nonfinite_or_incomplete_required_channels_are_rejected(self):
        sample = decode_feedback(packet(), "test")
        for channel in ("q", "dq", "target"):
            for value in (None, math.nan, math.inf):
                with self.subTest(channel=channel, value=value):
                    self.assertIn(f"left invalid {channel} at J3",
                                  classify_feedback(changed_arm(sample, "left", channel, value=value)).control_problem)
        arm = sample.payload.arms["left"]
        for dq, invalid in ((None, (1, 2, 3, 4, 5, 6, 7)), ((0.,)*6, (7,)), ((0.,)*8, (8,))):
            incomplete = replace(sample, payload=replace(sample.payload, arms={**sample.payload.arms,
                "left": replace(arm, joints=replace(arm.joints, velocity_rad_s=dq))}))
            self.assertEqual(classify_feedback(incomplete).control_issues[0].joint_indices, invalid)

    def test_measured_target_difference_does_not_invent_a_hardware_fault(self):
        p = packet(1)
        p.target[0] = p.q[0] + 20.
        assessment = classify_feedback(decode_feedback(p, "test"))
        # Finite feedback alone cannot identify a stall, encoder failure or load.
        self.assertIsNone(assessment.control_problem)
        self.assertEqual(assessment.observation_issues, ())


class FeedbackDriverTests(unittest.TestCase):
    def setUp(self):
        self.driver, self.sink = TianjiDriver("192.0.2.1"), Sink()
        self.driver._sdk, self.driver._sink = FakeSDK(self.driver), self.sink

    def events(self, prefix):
        return [e for e in self.sink.events if getattr(e, "kind", "").startswith(prefix)]

    def test_one_classification_per_packet_is_reused_for_selected_health_checks(self):
        self.driver.profile = SimpleNamespace(active_arms=("left",))
        self.driver.engaged = True
        bad = packet(1)
        bad.error[1] = 13
        with patch("bimanual_teleop.devices.tianji.driver.classify_feedback", wraps=classify_feedback) as classify:
            self.driver._on_feedback(bad)
            for _ in range(3):
                self.assertTrue(self.driver.health(sides=("left",)).ready)
                self.assertIn("right controller error 13", self.driver.health(sides=("right",)).detail)
            self.assertEqual(classify.call_count, 1)
            self.driver._on_feedback(packet(2))
            self.assertTrue(self.driver.health(sides=("right",)).ready)
            self.assertEqual(classify.call_count, 2)

    def test_unrecorded_config_is_decoded_only_when_metadata_is_requested(self):
        self.driver._record_reported_config = False
        with patch.object(self.driver, "_packet_config", wraps=self.driver._packet_config) as decode:
            self.driver._on_feedback(packet(1))
            latest = packet(2)
            latest.cart_k[0] = 321.
            self.driver._on_feedback(latest)
            self.assertEqual(decode.call_count, 0)
            self.assertTrue(self.driver.health().ready)
            metadata = self.driver.metadata
            self.assertEqual(metadata["reported_config"]["left"]["stiffness_native"][0], 321.)
            self.assertEqual(decode.call_count, 2)
        self.assertFalse(self.events("tianji.reported_config"))

    def test_optional_field_changes_are_reported_once_and_recovery_is_observable(self):
        for index in range(1, 21):
            p = packet(index)
            p.current[1] = math.nan
            self.driver._on_feedback(p)
            self.assertTrue(self.driver.health().ready)
        events = self.events("tianji.observation_")
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].details["packet_index"], 1)
        self.assertEqual(events[0].details["observation_issues"][0]["joint_indices"], (2,))
        self.assertEqual(self.driver.metadata["feedback_observation_issues"][0]["channel"], "current")
        p = packet(21)
        p.current[2] = math.nan
        self.driver._on_feedback(p)
        self.assertEqual(len(self.events("tianji.observation_")), 2)
        self.driver._on_feedback(packet(22))
        self.driver._on_feedback(packet(23))
        self.assertEqual([e.kind for e in self.events("tianji.observation_")],
                         ["tianji.observation_unavailable", "tianji.observation_unavailable",
                          "tianji.observation_recovered"])
        self.assertEqual(self.driver.metadata["feedback_observation_issues"], [])
        self.assertFalse(self.events("tianji.invalid_feedback"))

    def test_target_validity_is_checked_before_engagement(self):
        p = packet(1)
        p.target[0] = math.nan
        self.driver._on_feedback(p)
        self.assertFalse(self.driver.health(sides=("left",)).ready)
        self.assertIn("left invalid target at J1", self.driver.health(sides=("left",)).detail)
        self.assertTrue(self.driver.health(sides=("right",)).ready)

    def test_selected_control_fault_survives_valid_burst_until_explicit_reengagement(self):
        self.driver.profile = SimpleNamespace(active_arms=("left",))
        self.driver.engaged = True
        p = packet(1)
        p.error[0] = 13
        self.driver._on_feedback(p)
        self.driver._on_feedback(packet(2))
        self.assertIn("left controller error 13", self.driver._motion_fault)
        self.assertFalse(self.driver.health().ready)
        self.assertEqual(len(self.events("tianji.invalid_feedback")), 1)
        self.assertEqual(len(self.events("tianji.feedback_recovered")), 1)
        self.assertIn("received_monotonic_ns", self.events("tianji.invalid_feedback")[0].details)
        # Observation recovery and feedback recovery do not acknowledge motion faults.
        self.assertEqual(self.driver._sdk.calls, [])

    def test_unselected_fault_does_not_latch_selected_motion(self):
        self.driver.profile = SimpleNamespace(active_arms=("left",))
        self.driver.engaged = True
        p = packet(1)
        p.error[1], p.q[7], p.dq[8], p.target[9] = 13, math.nan, math.nan, math.nan
        self.driver._on_feedback(p)
        self.assertIsNone(self.driver._motion_fault)
        self.assertTrue(self.driver.health().ready)

    def test_driver_explicit_reengagement_acknowledges_a_recovered_critical_fault(self):
        self.driver.profile = SimpleNamespace(active_arms=("left",), profile_id="test")
        self.driver.engaged = True
        p = packet(1)
        p.dq[0] = math.nan
        self.driver._on_feedback(p)
        self.driver._on_feedback(packet(2))
        reason = self.driver._motion_fault
        self.assertIn("left invalid dq at J1", reason)
        self.driver.request_hold(reason)
        self.driver._on_feedback(packet(3))
        self.assertEqual(self.driver._motion_fault, reason)
        self.driver.engage()
        self.assertTrue(self.driver.engaged)
        self.assertIsNone(self.driver._motion_fault)
        self.assertTrue(self.driver.health().ready)
        self.driver.request_hold("test complete")

    def test_optional_observations_do_not_mask_real_errors_or_expired_feedback(self):
        p = packet(1, stamp=1_000_000_000)
        p.current[0], p.error[0] = math.nan, 13
        self.driver._on_feedback(p)
        self.assertIn("left controller error 13", self.driver._feedback_problem(("left",), p.received_ns))
        p = packet(2, stamp=1_005_000_000)
        p.torque[0] = math.nan
        self.driver._on_feedback(p)
        self.assertIn("watchdog", self.driver._feedback_problem(("left",), p.received_ns+self.driver.watchdog_ns))

    def assert_mode_fault_requires_reengagement(self, bad_state, bad_impedance):
        driver, sink = TianjiDriver("192.0.2.1", watchdog_s=.5), Sink()
        driver._sdk, driver._sink = FakeSDK(driver), sink
        driver.profile = SimpleNamespace(active_arms=("left",), profile_id="test", arms={
            "left": SimpleNamespace(velocity_ratio=50, acceleration_ratio=50)})
        driver._on_feedback(packet(1))
        driver.engage()
        self.assertTrue(driver._mode_confirmed)

        def feedback(state, impedance):
            p = deepcopy(driver._packet)
            p.packet_index += 1
            p.received_ns = time.monotonic_ns()
            p.sequence[:] = [sequence+1 for sequence in p.sequence]
            p.state[0], p.impedance_type[0] = state, impedance
            driver._on_feedback(p)
            return p

        def command():
            sample = driver.get_latest()
            now = time.monotonic_ns()
            return DeviceCommand("tianji", "mode-recovery", TianjiJointCommand({
                "left": sample.payload.arms["left"].joints.position_rad}, {}),
                (sample.header.ref,), now, now+100_000_000, "test")

        bad = feedback(bad_state, bad_impedance)
        feedback(3, 2)
        reason = driver._motion_fault
        self.assertIn("left is not reporting the engaged cartesian mode", reason)
        self.assertIn(f"packet_index={bad.packet_index}", reason)
        self.assertIn(f"received_monotonic_ns={bad.received_ns}", reason)
        self.assertIn("explicit re-engagement", reason)
        result = driver.submit(command())
        self.assertFalse(result.accepted)
        self.assertEqual(result.reason, reason)
        self.assertFalse(driver.engaged)
        submits = [args for name, args in driver._sdk.calls if name == "submit"]
        self.assertEqual(len(submits), 1)
        for args in submits:
            self.assertTrue(driver._commands[driver._token].startswith("hold-"))
            for sent, actual in zip(args[1][:7], driver._packet.q[:7]):
                self.assertAlmostEqual(sent, actual)
        events = [event for event in sink.events if getattr(event, "kind", None) == "tianji.control_mode_changed"]
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0].details["state"], bad_state)

        # A hold may return the controller to idle. That transition does not
        # become a new fault, and it cannot acknowledge the original fault.
        feedback(0, 0)
        self.assertEqual(driver._motion_fault, reason)
        driver.engage()
        self.assertIsNone(driver._motion_fault)
        self.assertTrue(driver.submit(command()).accepted)
        driver.request_hold("test complete")

    def test_transient_cartesian_state_and_impedance_changes_require_explicit_reengagement(self):
        for state, impedance in ((1, 2), (3, 1)):
            with self.subTest(state=state, impedance=impedance):
                self.assert_mode_fault_requires_reengagement(state, impedance)

    def test_mode_transition_before_confirmation_and_after_hold_do_not_latch(self):
        self.driver.profile = SimpleNamespace(active_arms=("left",))
        self.driver.engaged = True
        self.driver._on_feedback(packet(1))  # Idle feedback during mode entry.
        self.assertFalse(self.driver._mode_confirmed)
        self.assertIsNone(self.driver._motion_fault)
        self.driver._mode_confirmed = True
        p = packet(2)
        p.state[0], p.impedance_type[0] = 3, 2  # Unselected right remains idle.
        self.driver._on_feedback(p)
        self.assertIsNone(self.driver._motion_fault)
        self.driver.request_hold("operator pause")
        self.driver._on_feedback(packet(3))
        self.assertIsNone(self.driver._motion_fault)
        self.assertFalse(self.events("tianji.control_mode_changed"))

    def test_transient_position_mode_loss_aborts_real_move_even_after_feedback_recovers(self):
        driver = self.driver
        self.addCleanup(driver.close)
        driver.profile = SimpleNamespace(active_arms=("left",), profile_id="test", arms={
            "left": SimpleNamespace(velocity_ratio=50, acceleration_ratio=50)})
        driver._on_feedback(packet(1))
        target = driver.get_latest().payload.arms["left"].joints.position_rad
        original_call = driver._sdk.call

        def call(name, *args):
            result = original_call(name, *args)
            if name == "submit" and driver._moving_side == "left":
                for state in (3, 1):
                    feedback = deepcopy(driver._packet)
                    feedback.packet_index += 1
                    feedback.sequence[0] += 1
                    feedback.received_ns = time.monotonic_ns()
                    feedback.state[0] = state
                    driver._on_feedback(feedback)
            return result

        with patch.object(driver._sdk, "call", side_effect=call):
            with self.assertRaisesRegex(RuntimeError, "left is not reporting the engaged position mode"):
                driver.move_joints("left", target)
        self.assertFalse(driver.engaged)
        self.assertIsNone(driver._moving_side)
        self.assertIn("explicit re-engagement", driver._motion_fault)
        self.assertEqual(len(self.events("tianji.control_mode_changed")), 1)
        self.assertFalse(self.events("tianji.joint_move_completed"))
        self.assertTrue(any(name == "hold" for name, _ in driver._sdk.calls))

    def test_position_control_ignores_impedance_type_and_inactive_move_arm(self):
        self.driver.profile = SimpleNamespace(active_arms=("left", "right"))
        self.driver.engaged, self.driver._mode_confirmed = True, True
        self.driver._control_mode, self.driver._moving_side = "position", "left"
        for index, impedance in enumerate((0, 1, 2), 1):
            p = packet(index)
            p.state[0], p.impedance_type[0] = 1, impedance
            self.driver._on_feedback(p)
            self.assertIsNone(self.driver._motion_fault)
        self.assertFalse(self.events("tianji.control_mode_changed"))


class FeedbackRuntimeTests(unittest.TestCase):
    def fixture(self, side="both"):
        fx = RuntimeFixture(side=side)
        self.addCleanup(fx.runtime.close)
        clock = patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock)
        clock.start()
        self.addCleanup(clock.stop)
        fx.runtime.engage()
        self.assertIsNotNone(fx.runtime.tick(), fx.runtime.last_error)
        return fx

    def test_optional_observation_loss_keeps_both_arms_following(self):
        fx = self.fixture()
        for channel in ("torque", "external_torque", "current"):
            fx.advance()
            bad = changed_arm(fx.driver.latest, "left", channel)
            fx.driver.latest = bad
            fx.driver.receive(bad)
            self.assertIsNotNone(fx.runtime.tick(), fx.runtime.last_error)
            self.assertEqual(fx.runtime.state, SystemState.ENGAGED)
        self.assertEqual(fx.driver.holds, [])

    def test_critical_transients_pause_pair_and_only_explicit_resume_clears_fault(self):
        for channel, value in (("q", None), ("dq", math.nan), ("target", None), ("error", 13)):
            with self.subTest(channel=channel):
                fx = self.fixture()
                before = len(fx.driver.commands)
                fx.advance()
                good = fx.driver.latest
                bad = changed_arm(good, "right", channel, value=value)
                fx.driver.receive(bad)
                fx.driver.receive(good)
                self.assertIsNone(fx.runtime.tick())
                self.assertEqual(fx.runtime.state, SystemState.PAUSED)
                self.assertIn("right", fx.runtime.last_error)
                self.assertIn("packet_index=", fx.runtime.last_error)
                self.assertIn("received_monotonic_ns=", fx.runtime.last_error)
                self.assertEqual(len(fx.driver.commands), before)
                self.assertEqual(len(fx.driver.holds), 1)
                fx.advance()
                self.assertIsNone(fx.runtime.tick())
                self.assertEqual(len(fx.driver.commands), before)
                fx.runtime.engage()
                self.assertIsNotNone(fx.runtime.tick(), fx.runtime.last_error)
                self.assertEqual(fx.runtime.state, SystemState.ENGAGED)
                fx.runtime.close()

    def test_single_arm_target_fault_is_critical_but_other_arm_fault_is_ignored(self):
        fx = self.fixture("left")
        for channel, value in (("q", None), ("dq", None), ("target", None), ("error", 13)):
            fx.driver.receive(changed_arm(fx.driver.latest, "right", channel, value=value))
        self.assertIsNone(fx.driver._motion_fault)
        fx.driver.receive(changed_arm(fx.driver.latest, "left", "target"))
        self.assertIsNone(fx.runtime.tick())
        self.assertIn("left invalid target at J3", fx.runtime.last_error)

    def test_first_control_fault_is_not_overwritten_by_later_mode_fault(self):
        fx = self.fixture()
        fx.driver.receive(changed_arm(fx.driver.latest, "left", "error", value=13))
        first = fx.driver._motion_fault
        fx.driver.receive(changed_arm(fx.driver.latest, "right", "impedance_type", value=1))
        self.assertEqual(fx.driver._motion_fault, first)
        self.assertIsNone(fx.runtime.tick())
        self.assertIn("left controller error 13", fx.runtime.last_error)

    def test_transient_mode_fault_still_pauses_after_good_feedback(self):
        fx = self.fixture()
        good = fx.driver.latest
        fx.driver.receive(changed_arm(good, "right", "impedance_type", value=1))
        fx.driver.receive(good)
        self.assertIsNone(fx.runtime.tick())
        self.assertIn("Cartesian impedance mode changed", fx.runtime.last_error)


if __name__ == "__main__":
    unittest.main()
