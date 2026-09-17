"""Synthetic acquisition and the real Cartesian executor; no device access."""

import math
from dataclasses import replace
import unittest
from unittest.mock import patch

from bimanual_teleop.devices.tianji.driver import TianjiFrame
from bimanual_teleop.system import SystemState
from bimanual_teleop.control.arm.quest import QuestInputMonitor
from bimanual_teleop.types import Event, Health

from tests.support.quest import SIDES, Sink, Quest, RuntimeFixture
from tests.support.clock import Clock


class InputMonitorTests(unittest.TestCase):
    def setUp(self):
        self.clock, self.sink = Clock(), Sink()
        self.monitor = QuestInputMonitor(self.sink)
        self.quest = Quest(self.clock)
        self.quest.start(self.monitor)

    def test_transient_invalid_frame_is_latched_even_with_a_valid_latest(self):
        self.clock.advance()
        self.quest.emit(invalid="left")
        self.clock.advance(1)
        self.quest.emit()
        self.assertTrue(self.monitor.latest.header.valid)
        with self.assertRaisesRegex(RuntimeError, "tracking"):
            self.monitor.current(self.clock(), check_latch=True)
        self.monitor.current(self.clock(), acknowledge=True)
        self.monitor.current(self.clock(), check_latch=True)
        self.assertEqual(len(self.sink.samples), 3)

    def test_observer_failure_cannot_be_acknowledged_as_tracking_recovery(self):
        self.sink.ready = False
        self.clock.advance()
        self.quest.emit()
        with self.assertRaisesRegex(RuntimeError, "observer rejected"):
            self.monitor.current(self.clock(), acknowledge=True)
        self.assertFalse(self.monitor.health().ready)

    def test_supported_refresh_rates_do_not_block_tracking(self):
        for hz in (72., 80., 90., 120.):
            self.clock.advance()
            sample = self.quest.emit()
            self.monitor.latest = replace(sample, payload=replace(sample.payload, refresh_hz=hz))
            self.monitor.try_event(Event("quest.refresh_rate", self.clock(), "quest", {"to_hz": hz}))
            self.monitor.current(self.clock(), check_latch=True)

    def test_source_stall_deadline_does_not_get_refreshed_by_repeated_reads(self):
        sample, deadline, *_ = self.monitor.current(self.clock())
        self.clock.advance(90_000_000)
        current, actual, *_ = self.monitor.current(self.clock())
        self.assertIs(sample, current)
        self.assertEqual(deadline, actual)
        self.clock.advance(10_000_000)
        with self.assertRaisesRegex(RuntimeError, "silent"):
            self.monitor.current(self.clock(), check_latch=True)
        self.assertEqual(len(self.sink.samples), 1)

    def test_interframe_timeout_is_latched_without_a_control_poll_during_the_gap(self):
        for receive_gap, query_gap in ((100_000_000, 100_000_000), (100_000_000, 1), (1, 100_000_000)):
            with self.subTest(receive_gap=receive_gap, query_gap=query_gap):
                clock, monitor = Clock(), QuestInputMonitor()
                quest = Quest(clock)
                quest.start(monitor)
                previous = quest.latest.payload.query_monotonic_ns
                clock.advance(receive_gap)
                quest.emit(query_ns=previous + query_gap)
                self.assertIn("stream gap reached 100 ms", monitor.fault)
                # Resume receiving fresh samples before the next control poll.
                # Use this sample's clock offset, including the source-only gap.
                clock.advance()
                quest.emit(query_ns=previous + max(receive_gap, query_gap) + 5_000_000)
                with self.assertRaisesRegex(RuntimeError, "stream gap"):
                    monitor.current(clock(), check_latch=True)
                monitor.current(clock(), acknowledge=True)
                monitor.current(clock(), check_latch=True)

    def test_small_source_gaps_bursts_and_fixed_clock_offset_do_not_latch(self):
        for period in (round(1e9 / 72), round(1e9 / 90), round(1e9 / 120), 99_999_999):
            self.clock.advance(period)
            self.quest.sequence += 1  # One lost frame, but no timeout.
            self.quest.emit()
            self.assertEqual(self.monitor._baseline_ns, 500_000_000)
            self.monitor.current(self.clock(), check_latch=True)
        self.quest.emit(query_ns=self.quest.latest.payload.query_monotonic_ns + 1)
        self.monitor.current(self.clock(), check_latch=True)

    def test_new_session_clock_reset_is_reported_as_reference_change(self):
        self.clock.advance(500_000_000)
        self.quest.emit(session="restarted", query_ns=1)
        self.assertEqual(self.monitor.fault, "Quest reference changed; press Enter to re-engage")
        self.assertEqual(self.monitor._baseline_ns, self.clock() - 1)
        self.monitor.current(self.clock(), acknowledge=True)
        self.monitor.current(self.clock(), check_latch=True)

    def test_origin_change_keeps_clock_baseline_and_requires_reengagement(self):
        baseline = self.monitor._baseline_ns
        self.clock.advance()
        self.quest.emit(origin=1)
        self.assertEqual(self.monitor.fault, "Quest reference changed; press Enter to re-engage")
        self.assertEqual(self.monitor._baseline_ns, baseline)
        self.monitor.current(self.clock(), acknowledge=True)
        self.monitor.current(self.clock(), check_latch=True)

    def test_interframe_timeout_uses_configured_threshold(self):
        monitor, quest = QuestInputMonitor(timeout_ns=50_000_000), Quest(self.clock)
        quest.start(monitor)
        self.clock.advance(50_000_000)
        quest.emit()
        with self.assertRaisesRegex(RuntimeError, "stream gap reached 50 ms"):
            monitor.current(self.clock(), check_latch=True)

    def test_equal_host_timestamp_keeps_burst_but_equal_source_time_latches_fault(self):
        anchor = self.quest.latest.header.ref
        first = self.quest.emit(query_ns=500_000_001)
        second = self.quest.emit(query_ns=500_000_002)
        self.monitor.current(self.clock(), check_latch=True)
        self.assertEqual([s.header.ref for s, _ in self.monitor.since(anchor, second.header.ref)],
                         [first.header.ref, second.header.ref])
        self.quest.emit(query_ns=500_000_002)
        with self.assertRaisesRegex(RuntimeError, "source query time"):
            self.monitor.current(self.clock(), check_latch=True)

    def test_recent_host_receipt_does_not_hide_additional_source_backlog(self):
        self.clock.advance(120_000_000)
        self.quest.emit(query_ns=510_000_000)
        with self.assertRaisesRegex(RuntimeError, "queued|backlog"):
            self.monitor.current(self.clock(), check_latch=True)
        self.clock.advance()
        self.quest.emit()
        with self.assertRaises(RuntimeError): self.monitor.current(self.clock(), check_latch=True)
        self.monitor.current(self.clock(), acknowledge=True)

    def test_focus_event_and_read_end_are_latched_without_a_pose_frame(self):
        for event in (Event("quest.session_state", self.clock(), "quest", {"details": {"state": 4}}),
                      Event("quest.disconnected", self.clock(), "quest")):
            self.monitor.try_event(event)
            with self.assertRaises(RuntimeError): self.monitor.current(self.clock(), check_latch=True)
            self.monitor.current(self.clock(), acknowledge=True)

    def test_origin_event_latches_fault_before_new_origin_frame(self):
        generation = self.monitor.generation
        self.monitor.try_event(Event("quest.reference_space_change", self.clock(), "quest", {}))
        self.assertGreater(self.monitor.generation, generation)
        with self.assertRaises(RuntimeError): self.monitor.current(self.clock(), check_latch=True)

    def test_head_tracking_requirement_is_independent_of_controller_selection(self):
        for sides in (("left",), SIDES):
            for require_head in (False, True):
                with self.subTest(sides=sides, require_head=require_head):
                    monitor = QuestInputMonitor(sides=sides, require_head=require_head)
                    self.clock.advance()
                    monitor.try_publish(self.quest.emit(invalid="head"))
                    if require_head:
                        with self.assertRaisesRegex(RuntimeError, "head tracking"):
                            monitor.current(self.clock(), check_latch=True)
                    else:
                        monitor.current(self.clock(), check_latch=True)


class RuntimeTests(unittest.TestCase):
    def setUp(self):
        self.fx = RuntimeFixture()
        self.runtime = self.fx.runtime
        self.clock_patch = patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=self.fx.clock)
        self.clock_patch.start()
        self.addCleanup(self.clock_patch.stop)
        self.addCleanup(self.runtime.close)

    def engage(self): self.runtime.engage(); self.runtime.tick()

    def test_close_skips_pause_and_releases_quest_after_disable_failure(self):
        self.engage()
        self.fx.driver.holds.clear()
        with patch.object(self.fx.driver, "close", side_effect=RuntimeError("disable unconfirmed")):
            with self.assertRaisesRegex(RuntimeError, "disable unconfirmed"):
                self.runtime.close()
        self.assertEqual(self.fx.driver.holds, [])
        self.assertTrue(self.fx.quest.closed)
        self.assertEqual(self.runtime.state, SystemState.CLOSED)
        self.runtime.pause("late worker failure")
        self.assertEqual(self.fx.driver.holds, [])

    def test_steady_tick_uses_fk_only_inside_each_servo(self):
        self.engage()
        self.fx.advance()
        with patch.object(self.fx.kine, "fk", wraps=self.fx.kine.fk) as fk:
            self.assertIsNotNone(self.runtime.tick(), self.runtime.last_error)
        self.assertEqual([call.args[0] for call in fk.call_args_list], ["left", "left", "right", "right"])

    def test_config_change_observation_is_enabled_only_with_an_external_sink(self):
        self.assertTrue(self.fx.driver.record_reported_config)
        fx = RuntimeFixture(external_sink=False)
        self.addCleanup(fx.runtime.close)
        self.assertFalse(fx.driver.record_reported_config)

    def test_capture_only_start_and_live_engagement_seed_from_exact_feedback(self):
        self.assertEqual(self.fx.driver.calls, ["start"])
        self.engage()
        self.assertEqual(self.fx.driver.calls, ["start", "configure", "engage"])
        target = self.runtime._last_target
        for side in SIDES:
            self.assertEqual(target.tool_poses[side], self.fx.executor.engagement_poses[side])
        self.assertEqual(len(self.fx.driver.commands), 1)

    def test_external_status_query_does_not_affect_following(self):
        self.engage()
        self.fx.sink.health = lambda: Health(False, self.fx.clock(), "disk full")
        self.fx.advance()
        self.assertIsNotNone(self.runtime.tick())
        self.assertEqual(self.runtime.state, SystemState.ENGAGED)

    def test_realtime_observer_rejection_pauses_before_next_arm_target(self):
        self.engage()
        before = len(self.fx.driver.commands)
        self.fx.sink.ready = False
        self.fx.clock.advance()
        self.fx.quest.emit()
        self.runtime.tick()
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertIn("observer", self.runtime.last_error)
        self.assertEqual(len(self.fx.driver.commands), before)
        self.assertEqual(len(self.fx.driver.holds), 1)

    def test_invalid_then_valid_burst_pauses_pair_without_auto_resume(self):
        self.engage()
        before = len(self.fx.driver.commands)
        self.fx.advance(quest=False)
        self.fx.quest.emit(invalid="left")
        self.fx.clock.advance(1)
        self.fx.quest.emit()
        self.runtime.tick()
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertEqual(len(self.fx.driver.commands), before)
        self.assertEqual(len(self.fx.driver.holds), 1)
        self.fx.advance()
        self.runtime.tick()
        self.assertEqual(len(self.fx.driver.commands), before)

    def test_resume_reanchors_after_robot_and_controller_moved(self):
        self.engage()
        self.runtime.pause("operator")
        self.fx.driver.q["left"] = (.35, .2, .5, 0, 0, 0, 0)
        self.fx.advance(quest=False)
        self.fx.quest.emit(positions={"right": (.5, .1, .8), "head": (.2, .3, 1.7)},
                           rotations={"head": (0., 0., math.sin(.4), math.cos(.4))})
        self.runtime.engage()
        target = self.runtime.tick()
        self.assertEqual(target.tool_poses["left"].position_m, (.35, .2, .5))
        self.assertEqual(self.fx.driver.calls.count("configure"), 1)

    def test_new_reference_pauses_and_enter_reanchors_without_calibration(self):
        self.engage()
        before = self.runtime._last_target.tool_poses
        self.fx.advance(quest=False)
        self.fx.quest.emit(origin=1, positions={"left": (.7, .5, .8), "right": (.5, -.4, .8)})
        self.runtime.tick()
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertFalse(self.runtime.status()["reference_ready"])
        count = len(self.fx.driver.commands)
        self.fx.advance()
        self.runtime.tick()
        self.assertEqual(len(self.fx.driver.commands), count)
        self.runtime.engage()
        result = self.runtime.tick()
        self.assertEqual(result.tool_poses, before)
        self.assertTrue(self.runtime.status()["reference_ready"])

    def test_engagement_records_effective_mapping_and_both_anchors(self):
        self.engage()
        event = next(e for e in self.fx.sink.events if e.kind == "teleop.engaged")
        self.assertEqual(event.details["mapping_id"], self.runtime.status()["mapping_id"])
        self.assertEqual(set(event.details["base_from_quest"]), set(SIDES))
        self.assertIn("quest_anchor", event.details)
        self.assertIn("robot_anchors", event.details)

    def test_head_translation_changes_both_arm_goals_in_headset_frame(self):
        self.engage()
        before = self.runtime._last_target.tool_poses
        self.fx.advance(positions={"head": (.1, .2, 1.8)})
        self.assertIsNotNone(self.runtime.tick(), self.runtime.last_error)
        goal = next(s.payload for s in reversed(self.fx.sink.samples) if s.header.ref.stream == "teleop.goals")
        for side, delta in (("left", (-.1, .3, -.2)), ("right", (-.1, -.3, .2))):
            for actual, start, change in zip(goal.tool_poses[side].position_m, before[side].position_m, delta):
                self.assertAlmostEqual(actual, start + change)

    def test_both_servos_succeed_before_dispatch_and_right_model_failure_holds_pair(self):
        self.engage()
        before = len(self.fx.driver.commands)
        self.fx.kine.fail_side = "right"
        self.fx.advance()
        self.runtime.tick()
        self.assertEqual(len(self.fx.driver.commands), before)
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertIn("invalid Jacobian", self.runtime.last_error)

    def test_relative_motion_has_no_arbitrary_ten_centimeter_cutoff(self):
        self.engage()
        for _ in range(200):
            self.fx.advance(positions={"right": (.2, -.2, 1.)})
            self.assertIsNotNone(self.runtime.tick(), self.runtime.last_error)
        self.assertEqual(self.runtime.state, SystemState.ENGAGED)
        self.assertGreater(abs(self.runtime._last_target.tool_poses["left"].position_m[0]-.3), .19)

    def test_stale_quest_target_is_not_renewed_by_200_hz_ticks(self):
        self.engage()
        for _ in range(20):
            self.fx.advance(quest=False)
            self.runtime.tick()
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertIn("silent", self.runtime.last_error)

    def test_mode_and_controller_error_transients_are_latched_per_feedback_packet(self):
        for kwargs in ({"wrong_mode": "right"}, {"error_side": "right"}):
            self.engage()
            self.fx.driver.emit(**kwargs)
            self.fx.driver.emit()
            self.runtime.tick()
            self.assertEqual(self.runtime.state, SystemState.PAUSED)

    def test_transient_robot_mode_fault_during_planning_prevents_group_dispatch(self):
        self.engage()
        before = len(self.fx.driver.commands)
        servos = dict(self.fx.executor._servos)
        previous = {s: (servo.q, servo.velocity) for s, servo in servos.items()}

        def mode_changes():
            self.fx.driver.emit(wrong_mode="right")
            self.fx.driver.emit()

        self.fx.kine.after_ik = mode_changes
        self.fx.advance()
        self.assertIsNone(self.runtime.tick())
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertIn("mode changed", self.runtime.last_error)
        self.assertEqual(len(self.fx.driver.commands), before)
        self.assertEqual({s: (servo.q, servo.velocity) for s, servo in servos.items()}, previous)

    def test_observer_pause_during_submission_does_not_resurrect_or_advance_servos(self):
        self.engage()
        servos = dict(self.fx.executor._servos)
        previous = {s: (servo.q, servo.velocity) for s, servo in servos.items()}
        publish = self.fx.sink.try_publish

        def reject_feedback(sample):
            return False if isinstance(sample.payload, TianjiFrame) else publish(sample)

        self.fx.advance()
        self.fx.sink.try_publish = reject_feedback
        self.assertIsNone(self.runtime.tick())
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertFalse(self.fx.driver.engaged)
        self.assertEqual(self.fx.executor._reference, {})
        self.assertEqual(self.fx.executor._servos, {})
        self.assertEqual({s: (servo.q, servo.velocity) for s, servo in servos.items()}, previous)

    def test_pause_in_first_arm_planning_keeps_both_servo_proposals_uncommitted(self):
        self.engage()
        before = len(self.fx.driver.commands)
        servos = dict(self.fx.executor._servos)
        previous = {s: (servo.q, servo.velocity) for s, servo in servos.items()}
        self.fx.kine.after_ik = lambda: self.runtime.pause("pause during planning")
        self.fx.advance()
        self.assertIsNone(self.runtime.tick())
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertEqual(len(self.fx.driver.commands), before)
        self.assertEqual({s: (servo.q, servo.velocity) for s, servo in servos.items()}, previous)

    def test_a_fault_during_blocking_engagement_prevents_first_follow_target(self):
        def lost_and_recovered():
            self.fx.advance(quest=False)
            self.fx.quest.emit(invalid="right")
            self.fx.clock.advance(1)
            self.fx.quest.emit()
        self.fx.driver.after_engage = lost_and_recovered
        with self.assertRaises(RuntimeError): self.runtime.engage()
        self.assertEqual(self.fx.driver.commands, [])
        self.assertEqual(self.runtime.state, SystemState.PAUSED)
        self.assertFalse(self.fx.driver.engaged)

    def test_90_hz_bursts_and_late_ticks_keep_following(self):
        self.engage()
        previous = self.runtime._last_target
        for i in range(1, 101):
            self.fx.advance(20_000_000 if i == 40 else 5_000_000, quest=False)
            if i % 4 == 0:
                # Two samples delivered before the next control poll.
                for j in range(2):
                    self.fx.clock.advance(1)
                    self.fx.quest.emit(positions={"left": (i*.0001+j*.00001, .2, 1.)})
            target = self.runtime.tick()
            self.assertIsNotNone(target, self.runtime.last_error)
            self.assertTrue(all(math.isfinite(x) for x in target.tool_poses["left"].position_m))
            previous = target
        self.assertEqual(self.runtime.state, SystemState.ENGAGED)

    def test_each_burst_frame_is_filtered_once_with_original_query_time(self):
        self.engage()
        initial_ns = self.fx.clock()
        self.fx.advance(20_000_000, quest=False)
        frames = [self.fx.quest.emit(query_ns=initial_ns-500_000_000+offset,
                    positions={"right": (position, -.2, 1.)})
                  for offset, position in [(5_000_000, .01), (10_000_000, .02)]]
        result = self.runtime.tick()
        self.assertIsNotNone(result, self.runtime.last_error)
        goals = [s.payload for s in self.fx.sink.samples if s.header.ref.stream == "teleop.goals"]
        self.assertEqual(len(goals), 2)
        self.assertEqual(goals[0].created_monotonic_ns, goals[1].created_monotonic_ns)
        filtered_first = .3*.8 + goals[0].tool_poses["left"].position_m[0]*.2
        filtered_second = filtered_first*.8 + goals[1].tool_poses["left"].position_m[0]*.2
        fraction = (20_000_000-self.runtime._interpolator.delay_ns-5_000_000)/5_000_000
        self.assertAlmostEqual(self.runtime._requested_target.tool_poses["left"].position_m[0],
                               filtered_first+(filtered_second-filtered_first)*fraction)
        for sample in frames:
            self.assertIn(sample.header.ref, result.source_refs)
        self.assertEqual(result.created_monotonic_ns, self.fx.clock())

    def test_fresh_pose_arriving_during_fk_is_not_misclassified_as_future(self):
        self.engage()
        fk = self.fx.kine.fk
        def delayed_fk(side, q):
            self.fx.clock.advance(100_000)
            self.fx.quest.emit()
            return fk(side, q)
        self.fx.kine.fk = delayed_fk
        self.fx.advance()
        self.assertIsNotNone(self.runtime.tick())


class CommandTimelineTests(unittest.TestCase):
    def test_pause_during_planning_does_not_advance_either_arm(self):
        fx = RuntimeFixture()
        self.addCleanup(fx.runtime.close)
        fx.runtime.engage()
        servos = dict(fx.executor._servos)
        previous = {s: (servo.q, servo.velocity) for s, servo in servos.items()}
        fx.kine.after_ik = lambda: fx.runtime.pause("paused during planning")
        fx.advance()
        self.assertIsNone(fx.runtime.tick())
        self.assertEqual(fx.runtime.state, SystemState.PAUSED)
        self.assertEqual({s: (servo.q, servo.velocity) for s, servo in servos.items()}, previous)
        self.assertEqual(fx.driver.commands, [])

    def test_engagement_processing_delay_does_not_shift_the_source_timeline(self):
        targets = []
        for anchor_age_ns in (0, 20_000_000):
            fx = RuntimeFixture()
            fx.clock.advance(anchor_age_ns)
            fx.runtime.engage()
            fx.advance(20_000_000-anchor_age_ns, positions={"left": (.01, .2, 1.)})
            result = fx.runtime.tick()
            self.assertIsNotNone(result, fx.runtime.last_error)
            targets.append(result.tool_poses)
            fx.runtime.close()
        self.assertEqual(targets[0], targets[1])

    def test_ik_compute_duration_does_not_change_the_200_hz_command_timeline(self):
        for hz in (72, 90, 120):
            outputs = []
            for total_compute_ns in (0, 1_000_000, 3_000_000):
                with self.subTest(hz=hz, total_compute_ns=total_compute_ns):
                    fx = RuntimeFixture()
                    anchor = replace(fx.quest.latest, payload=replace(fx.quest.latest.payload, refresh_hz=hz))
                    fx.runtime.input.latest = anchor
                    fx.runtime.engage()
                    self.assertEqual(fx.runtime._interpolator.delay_ns, round(1e9/hz))
                    start = fx.clock()
                    fx.kine.after_ik = lambda: fx.clock.advance(total_compute_ns//2)
                    next_frame, poses = 1, []
                    for tick in range(201):
                        elapsed_ns = tick*5_000_000
                        fx.clock.now = start+elapsed_ns
                        fx.driver.emit()
                        while round(next_frame*1e9/hz) <= elapsed_ns:
                            query_elapsed = round(next_frame*1e9/hz)
                            fx.quest.emit(query_ns=start-500_000_000+query_elapsed, refresh_hz=hz,
                                          positions={"right": (.1*query_elapsed/1e9, -.2, 1.)})
                            next_frame += 1
                        result = fx.runtime.tick()
                        self.assertIsNotNone(result, fx.runtime.last_error)
                        self.assertEqual(result.created_monotonic_ns, start+elapsed_ns)
                        poses.append(result.tool_poses["left"])
                    outputs.append(poses)
                    fx.runtime.close()
            self.assertEqual(outputs[0], outputs[1])
            self.assertEqual(outputs[0], outputs[2])

    def test_fixed_axes_map_once_and_only_engagement_configures(self):
        fx = RuntimeFixture()
        fx.runtime.engage()
        fx.runtime.tick()
        for i in range(1, 21):
            fx.advance(positions={"right": (0., -.2+i*.0001, 1.)})
            self.assertIsNotNone(fx.runtime.tick(), fx.runtime.last_error)
        pose = fx.runtime._last_target.tool_poses["left"]
        self.assertAlmostEqual(pose.position_m[0], .3)
        self.assertAlmostEqual(pose.position_m[1], .2)
        self.assertGreater(pose.position_m[2], .5)
        fx.runtime.pause("test done")
        fx.runtime.close()
        self.assertEqual(fx.driver.calls, ["start", "configure", "engage", "close"])
        self.assertTrue(fx.driver.commands)
        self.assertEqual(len(fx.driver.holds), 1)

    def test_planning_keeps_the_original_50ms_deadline_and_rejects_its_boundary(self):
        for compute_ns in (49_999_998, 50_000_000, 60_000_000):
            with self.subTest(compute_ns=compute_ns):
                fx = RuntimeFixture()
                self.addCleanup(fx.runtime.close)
                fx.runtime.engage()
                started = fx.clock()
                fx.kine.after_ik = lambda: fx.clock.advance(compute_ns // 2)
                target = fx.runtime.tick()
                if compute_ns < 50_000_000:
                    self.assertIsNotNone(target, fx.runtime.last_error)
                    command, = fx.driver.commands
                    self.assertEqual(command.created_monotonic_ns, started)
                    self.assertEqual(command.expires_monotonic_ns, started + 50_000_000)
                else:
                    self.assertIsNone(target)
                    self.assertIn("expired during IK", fx.runtime.last_error)
                    self.assertEqual(fx.driver.commands, [])
                    self.assertEqual(fx.runtime.state, SystemState.PAUSED)


class SingleSideTests(unittest.TestCase):
    def test_unselected_tracking_and_robot_faults_do_not_interrupt_selected_arm(self):
        for side in SIDES:
            with self.subTest(side=side):
                other = "right" if side == "left" else "left"
                fx = RuntimeFixture(side=side)
                self.addCleanup(fx.runtime.close)
                with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock):
                    fx.quest.emit(invalid=side)
                    fx.driver.emit(error_side=other)
                    fx.kine.fail_side = other
                    self.assertTrue(fx.runtime.health().ready)
                    fx.runtime.engage()
                    target = fx.runtime.tick()
                    self.assertEqual(set(target.tool_poses), {side})
                    self.assertEqual(set(fx.driver.commands[-1].payload.targets), {side})
                    self.assertEqual(fx.driver.profile.active_arms, (side,))
                    fx.advance(quest=False)
                    fx.quest.emit(positions={other: (.1, .2, 1.)}, invalid=side)
                    self.assertIsNotNone(fx.runtime.tick(), fx.runtime.last_error)
                    self.assertEqual(fx.runtime.state, SystemState.ENGAGED)

    def test_selected_tracking_loss_and_feedback_error_pause(self):
        for side in SIDES:
            with self.subTest(side=side):
                fx = RuntimeFixture(side=side)
                self.addCleanup(fx.runtime.close)
                with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock):
                    fx.runtime.engage()
                    fx.runtime.tick()
                    fx.advance(quest=False)
                    fx.quest.emit(invalid="right" if side == "left" else "left")
                    fx.runtime.tick()
                    self.assertEqual(fx.runtime.state, SystemState.PAUSED)
                    fx.advance()
                    fx.runtime.engage()
                    fx.driver.emit(error_side=side)
                    fx.runtime.tick()
                    self.assertEqual(fx.runtime.state, SystemState.PAUSED)

    def test_single_arm_only_checks_selected_kinematics(self):
        fx = RuntimeFixture(side="right")
        self.addCleanup(fx.runtime.close)
        fx.kine.fail_side = "left"
        fx.quest.emit(invalid="right")
        fx.runtime.engage()
        self.assertEqual(set(fx.runtime.tick().tool_poses), {"right"})
        self.assertEqual(fx.driver.calls, ["start", "configure", "engage"])
        self.assertEqual(set(fx.driver.commands[-1].payload.targets), {"right"})


class ReferenceFrameTests(unittest.TestCase):
    def next_goal(self, fx, **changes):
        fx.advance(quest=False)
        fx.quest.emit(**changes)
        self.assertIsNotNone(fx.runtime.tick(), fx.runtime.last_error)
        return next(s.payload for s in reversed(fx.sink.samples) if s.header.ref.stream == "teleop.goals")

    def test_each_controller_moves_only_the_opposite_arm_in_single_and_dual_modes(self):
        for robot_side, controller_side, delta in (("left", "right", (.1, -.3, .2)),
                                                   ("right", "left", (.1, .3, -.2))):
            for side in (robot_side, "both"):
                with self.subTest(robot_side=robot_side, side=side):
                    fx = RuntimeFixture(side=side)
                    self.addCleanup(fx.runtime.close)
                    fx.runtime.engage()
                    before = fx.runtime.tick().tool_poses
                    position = tuple(x+d for x, d in zip(fx.quest.positions[controller_side], (.1, .2, .3)))
                    goal = self.next_goal(fx, positions={controller_side: position})
                    for actual, start, change in zip(goal.tool_poses[robot_side].position_m,
                                                      before[robot_side].position_m, delta):
                        self.assertAlmostEqual(actual, start + change)
                    if side == "both":
                        self.assertEqual(goal.tool_poses[controller_side], before[controller_side])

    def test_head_tracking_loss_pauses_only_headset_mode(self):
        for side in ("both", *SIDES):
            for coordinate_frame in ("headset", "world"):
                with self.subTest(side=side, coordinate_frame=coordinate_frame):
                    fx = RuntimeFixture(side=side, coordinate_frame=coordinate_frame)
                    self.addCleanup(fx.runtime.close)
                    with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock), \
                            patch.object(fx.quest, "health", wraps=fx.quest.health) as health:
                        fx.runtime.engage()
                        self.assertIsNotNone(fx.runtime.tick())
                        before = len(fx.driver.commands)
                        fx.advance(quest=False)
                        fx.quest.emit(invalid="head")
                        target = fx.runtime.tick()
                        selected = SIDES if side == "both" else ("right" if side == "left" else "left",)
                        self.assertEqual(set(health.call_args.kwargs["sides"]), set(selected))
                        self.assertEqual(health.call_args.kwargs["require_head"], coordinate_frame == "headset")
                        if coordinate_frame == "headset":
                            self.assertIsNone(target)
                            self.assertEqual(fx.runtime.state, SystemState.PAUSED)
                            self.assertEqual(len(fx.driver.commands), before)
                        else:
                            self.assertIsNotNone(target, fx.runtime.last_error)
                            self.assertEqual(fx.runtime.state, SystemState.ENGAGED)

    def test_world_mode_ignores_head_position_and_vertical_head_orientation(self):
        fx = RuntimeFixture(coordinate_frame="world")
        self.addCleanup(fx.runtime.close)
        fx.runtime.engage()
        before = fx.runtime.tick().tool_poses
        goal = self.next_goal(fx, positions={"head": (1., -1., .5)},
                              rotations={"head": (0., math.sqrt(.5), 0., math.sqrt(.5))})
        self.assertEqual(goal.tool_poses, before)

    def test_headset_pitch_and_roll_do_not_change_targets(self):
        fx = RuntimeFixture()
        self.addCleanup(fx.runtime.close)
        fx.runtime.engage()
        before = fx.runtime.tick().tool_poses
        sx, cx, sy, cy = math.sin(.2), math.cos(.2), math.sin(.3), math.cos(.3)
        goal = self.next_goal(fx, rotations={"head": (sx*cy, cx*sy, -sx*sy, cx*cy)})
        for side in SIDES:
            for actual, expected in zip(goal.tool_poses[side].position_m, before[side].position_m):
                self.assertAlmostEqual(actual, expected)
            for actual, expected in zip(goal.tool_poses[side].orientation_xyzw, before[side].orientation_xyzw):
                self.assertAlmostEqual(actual, expected)

    def test_vertical_head_forward_in_a_burst_pauses_and_resume_reanchors(self):
        fx = RuntimeFixture()
        self.addCleanup(fx.runtime.close)
        with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock):
            fx.runtime.engage()
            before = fx.runtime.tick().tool_poses
            command_count = len(fx.driver.commands)
            fx.advance(quest=False)
            fx.quest.emit(rotations={"head": (0., math.sqrt(.5), 0., math.sqrt(.5))})
            fx.clock.advance(1)
            fx.quest.emit(positions={"head": (.2, .3, 1.7), "right": (.6, .1, .9)})
            self.assertIsNone(fx.runtime.tick())
            self.assertEqual(fx.runtime.state, SystemState.PAUSED)
            self.assertEqual(len(fx.driver.commands), command_count)
            fx.runtime.engage()
            self.assertEqual(fx.runtime.tick().tool_poses, before)

    def test_vertical_head_frame_during_ik_blocks_dispatch_only_in_headset_mode(self):
        for coordinate_frame in ("headset", "world"):
            for recovered in (False, True):
                with self.subTest(coordinate_frame=coordinate_frame, recovered=recovered):
                    fx = RuntimeFixture(coordinate_frame=coordinate_frame)
                    self.addCleanup(fx.runtime.close)
                    with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock):
                        fx.runtime.engage()
                        self.assertIsNotNone(fx.runtime.tick())
                        before = len(fx.driver.commands)
                        fx.advance()

                        def head_vertical():
                            fx.kine.after_ik = None
                            fx.clock.advance(1)
                            fx.quest.emit(rotations={"head": (0., math.sqrt(.5), 0., math.sqrt(.5))})
                            if recovered:
                                fx.clock.advance(1)
                                fx.quest.emit()

                        fx.kine.after_ik = head_vertical
                        target = fx.runtime.tick()
                        if coordinate_frame == "headset":
                            self.assertIsNone(target)
                            self.assertEqual(fx.runtime.state, SystemState.PAUSED)
                            self.assertIn("yaw", fx.runtime.last_error)
                            self.assertEqual(len(fx.driver.commands), before)
                        else:
                            self.assertIsNotNone(target, fx.runtime.last_error)
                            self.assertEqual(fx.runtime.state, SystemState.ENGAGED)
                            self.assertEqual(len(fx.driver.commands), before + 1)


if __name__ == "__main__":
    unittest.main()
