"""Hand following and lifecycle tests; no SDK manager or hardware connections."""

from dataclasses import replace
import importlib.util
import math
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch

from bimanual_teleop.devices.wuji.adapter import JOINT_LIMITS_RAD, JOINT_NAMES
from bimanual_teleop.system import SystemState
from bimanual_teleop.control.hand.follow import WujiHandRetargeter, WujiTeleop, create_wuji_teleop
from bimanual_teleop.types import (
    ControlProfile, HandSkeleton, Health, JointState, JointTarget, Sample,
    SampleHeader, SampleRef, Submission,
)


class Clock:
    def __init__(self): self.now = 1_000_000_000
    def __call__(self): return self.now
    def advance(self, seconds): self.now += round(seconds * 1e9)


class Sink:
    def __init__(self): self.events, self.samples = [], []
    def try_event(self, event): self.events.append(event); return True
    def try_publish(self, sample): self.samples.append(sample); return True


class Device:
    def __init__(self, side, clock, *, glove=False):
        self.side, self.clock, self.glove = side, clock, glove
        self.device_id = f"wuji_{side}_{'glove' if glove else 'hand'}"
        self.metadata, self.statistics = {}, {}
        self.fault, self.latest, self.sink = None, None, None
        self.sequence, self.q = 0, .1
        self.ready, self.closed = True, False
        self.enabled, self.profile, self.last_target = False, None, None
        self.commands, self.calls = [], []
        self.accept, self.engage_hook, self.disable_hook, self.close_error = True, None, None, None

    def start(self, sink=None): self.sink = sink; self.calls.append("start"); self.emit()

    def emit(self, *, valid=True, q=None):
        if q is not None: self.q = q
        self.sequence += 1
        payload = HandSkeleton(f"{self.side}_wrist", tuple(f"p{i}" for i in range(21)),
            ((self.q, 0., 0.),)*21, (1.,)*21, ()) if self.glove else JointState(JOINT_NAMES, (self.q,)*20)
        self.latest = Sample(SampleHeader(SampleRef(self.device_id, "test", self.sequence),
                                         self.clock(), valid), payload)
        if not valid: self.fault = "invalid sample"
        if self.sink: self.sink.try_publish(self.latest)
        return self.latest

    def get_latest(self): return self.latest

    def health(self, *, check_latch=False):
        ready = self.ready and self.latest is not None and self.latest.header.valid
        if check_latch and self.fault: ready = False
        return Health(ready, self.clock(), None if ready else self.fault or "unavailable")

    def clear_fault(self):
        if not self.health().ready: raise RuntimeError("not currently healthy")
        self.fault = None

    def configure(self, profile): self.profile = profile; self.calls.append("configure")

    def engage(self, *, cancelled=None):
        self.calls.append("engage")
        if self.engage_hook: self.engage_hook()
        if cancelled and cancelled(): raise RuntimeError("cancelled")
        self.last_target = JointTarget(JOINT_NAMES, self.latest.payload.position_rad)
        self.enabled = True

    def submit(self, command):
        if not self.accept: return Submission(command.command_id, False, f"{self.side} send failed")
        if self.clock() >= command.expires_monotonic_ns:
            return Submission(command.command_id, False, "expired")
        self.commands.append(command)
        self.last_target = command.payload
        return Submission(command.command_id, True)

    def request_hold(self, reason): self.calls.append("hold")

    def disable(self, reason="requested"):
        self.calls.append("disable")
        self.enabled = False
        if self.disable_hook: self.disable_hook()

    def close(self):
        self.calls.append("close")
        self.enabled, self.closed = False, True
        if self.close_error: raise self.close_error


class Mapper:
    def __init__(self): self.calls, self.resets = 0, 0; self.fail = False; self.hook = None
    def reset(self): self.resets += 1
    def solve(self, sample):
        self.calls += 1
        if self.hook: self.hook()
        if self.fail: raise ValueError("invalid retarget output")
        return JointTarget(JOINT_NAMES, (sample.payload.positions_m[0][0],)*20)


class StartupTests(unittest.TestCase):
    def test_native_failure_closes_all_devices_and_preserves_both_errors(self):
        class WujiException(Exception):
            pass

        clock, sink = Clock(), Sink()
        gloves = {s: Device(s, clock, glove=True) for s in ("left", "right")}
        hands = {s: Device(s, clock) for s in gloves}
        cause = WujiException("right hand connection timeout")
        hands["left"].close_error = WujiException("left hand cleanup failed")
        runtime = WujiTeleop(gloves, hands, {s: Mapper() for s in gloves},
            profile=ControlProfile("test", "mit", {}), sink=sink, threaded=False)
        original_start = Device.start

        def start(device, sink=None):
            if device is hands["right"]:
                raise cause
            original_start(device, sink)

        with patch.object(Device, "start", start), self.assertRaisesRegex(
                RuntimeError, "right hand connection timeout.*left hand cleanup failed") as caught:
            runtime.start()
        self.assertIs(caught.exception.__cause__, cause)
        self.assertEqual(runtime.state, SystemState.CLOSED)
        self.assertEqual(runtime.last_error, str(caught.exception))
        self.assertTrue(all(d.closed for d in (*hands.values(), *gloves.values())))
        self.assertTrue(all(not h.enabled for h in hands.values()))
        failure = next(e for e in sink.events if e.kind == "wuji.start_failed")
        self.assertEqual(failure.details["error"], str(cause))
        runtime.close()


class RuntimeTests(unittest.TestCase):
    def make(self, *, motion=True, sides=("left", "right"), threaded=False, clock=None):
        self.clock = clock or Clock()
        self.gloves = {s: Device(s, self.clock, glove=True) for s in sides}
        self.hands = {s: Device(s, self.clock) for s in sides}
        self.maps = {s: Mapper() for s in sides}
        self.sink = Sink()
        self.runtime = WujiTeleop(self.gloves, self.hands, self.maps,
            profile=ControlProfile("tested", "mit", {"kp": 5., "kd": .05, "current_limit_a": 1.5}),
            sink=self.sink, enable_motion=motion, clock_ns=self.clock, threaded=threaded)
        self.addCleanup(self.runtime.close)
        self.runtime.start()
        return self.runtime

    def refresh(self):
        for device in (*self.gloves.values(), *self.hands.values()): device.emit()

    def join_disables(self):
        for thread in tuple(self.runtime._disable_threads.values()): thread.join(1)

    def test_start_is_acquisition_only_and_preview_never_enables(self):
        runtime = self.make(motion=False)
        self.assertEqual(runtime.state, SystemState.READY)
        self.assertTrue(all(h.calls == ["start"] for h in self.hands.values()))
        runtime.engage()
        runtime._step()
        self.assertTrue(all(not h.enabled and not h.commands for h in self.hands.values()))
        self.assertEqual(runtime.mode, "preview")

    def test_realtime_observer_failure_pauses_before_hand_submission(self):
        runtime = self.make()
        runtime.engage()
        before = {side: len(hand.commands) for side, hand in self.hands.items()}
        self.sink.try_event = lambda _: False
        self.gloves["left"].emit(q=.4)
        runtime._step()
        self.assertEqual(runtime.state, SystemState.PAUSED)
        self.assertIn("observer", runtime.last_error)
        self.assertEqual({side: len(hand.commands) for side, hand in self.hands.items()}, before)

    def test_left_and_right_targets_with_exact_takeover_endpoints(self):
        runtime = self.make()
        self.gloves["left"].emit(q=.3)
        self.gloves["right"].emit(q=.5)
        runtime.engage()
        runtime._step()
        self.assertTrue(all(h.last_target.position_rad == (.1,)*20 for h in self.hands.values()))
        self.clock.advance(.375); self.refresh(); runtime._step()
        self.assertAlmostEqual(self.hands["left"].last_target.position_rad[0], .2)
        self.assertAlmostEqual(self.hands["right"].last_target.position_rad[0], .3)
        self.clock.advance(.375); self.refresh(); runtime._step()
        self.assertEqual(self.hands["left"].last_target.position_rad, (.3,)*20)
        self.assertEqual(self.hands["right"].last_target.position_rad, (.5,)*20)
        # Following after takeover has no added velocity limiter.
        self.gloves["left"].emit(q=.6); self.clock.advance(.008); runtime._step()
        self.assertEqual(self.hands["left"].last_target.position_rad, (.6,)*20)

    def test_repeated_latest_and_caller_tick_do_not_resolve_or_add_raw_samples(self):
        runtime = self.make()
        runtime.engage()
        count = len(self.sink.samples)
        for _ in range(5): runtime._step(); runtime.tick()
        self.assertEqual([m.calls for m in self.maps.values()], [1, 1])
        self.assertEqual(len(self.sink.samples), count)
        self.assertEqual(len(self.hands["left"].commands), 5)

    def test_healthy_hands_hold_while_waiting_for_arm_engagement(self):
        runtime = self.make()
        runtime.prepare_engage()
        runtime._step()
        self.assertEqual(runtime.mode, "hold")
        self.assertNotEqual(runtime.state, SystemState.ENGAGED)
        self.assertTrue(all(len(h.commands) == 1 for h in self.hands.values()))
        runtime.begin_follow()
        self.assertEqual(runtime.state, SystemState.ENGAGED)

    def test_pause_resume_starts_at_held_command_not_loaded_actual_position(self):
        runtime = self.make()
        self.gloves["left"].emit(q=.4)
        runtime.engage(); self.clock.advance(.75); self.refresh(); runtime._step()
        runtime.pause("Space")
        self.hands["left"].emit(q=.05)
        self.gloves["left"].emit(q=.7)
        runtime._step()
        self.assertEqual(self.hands["left"].last_target.position_rad, (.4,)*20)
        runtime.engage(); runtime._step()
        self.assertEqual(self.hands["left"].last_target.position_rad, (.4,)*20)
        self.assertEqual(self.hands["left"].calls.count("configure"), 1)

    def test_resume_after_disable_references_new_actual_position(self):
        runtime = self.make()
        runtime.engage(); runtime.pause("fault")
        self.hands["left"].disable()
        self.hands["left"].emit(q=.02)
        runtime.engage(); runtime._step()
        self.assertEqual(self.hands["left"].last_target.position_rad, (.02,)*20)

    def test_transient_glove_failure_pauses_despite_valid_latest_then_manual_ack(self):
        runtime = self.make()
        runtime.engage()
        self.gloves["left"].emit(valid=False)
        self.gloves["left"].emit(valid=True)
        runtime._step()
        self.assertEqual(runtime.state, SystemState.PAUSED)
        self.assertTrue(all(h.enabled for h in self.hands.values()))
        self.assertTrue(runtime.health().ready)
        runtime.engage()
        self.assertIsNone(self.gloves["left"].fault)

    def test_stale_input_does_not_extend_its_own_deadline_through_hold(self):
        runtime = self.make()
        runtime.engage(); runtime._step()
        source = self.gloves["left"].latest.header
        self.clock.advance(.25)
        for hand in self.hands.values(): hand.emit()
        runtime._step(); runtime._step()
        self.assertEqual(runtime.state, SystemState.PAUSED)
        self.assertEqual(self.gloves["left"].latest.header, source)
        self.assertTrue(all(h.enabled for h in self.hands.values()))
        self.assertEqual(len(self.hands["left"].commands), 2)

    def test_hand_fault_disables_only_failed_side_and_other_side_holds(self):
        runtime = self.make()
        runtime.engage(); runtime._step()
        self.hands["left"].fault = "joint missing"
        runtime._step(); self.join_disables()
        self.assertEqual(runtime.state, SystemState.PAUSED)
        self.assertFalse(self.hands["left"].enabled)
        self.assertTrue(self.hands["right"].enabled)
        self.assertEqual(len(self.hands["right"].commands), 2)

    def test_slow_fault_disable_does_not_block_healthy_hand_or_pause(self):
        runtime = self.make()
        entered, release = threading.Event(), threading.Event()
        self.addCleanup(release.set)
        self.hands["left"].disable_hook = lambda: (entered.set(), release.wait(1))
        runtime.engage()
        self.hands["left"].fault = "offline"
        runtime._step()
        self.assertTrue(entered.wait(1))
        runtime.pause("operator pause")
        runtime._step()
        self.assertEqual(len(self.hands["right"].commands), 2)
        release.set(); self.join_disables()

    def test_both_solutions_are_checked_before_either_side_is_submitted(self):
        runtime = self.make()
        runtime.engage()
        self.maps["right"].fail = True
        self.gloves["left"].emit(q=.2); self.gloves["right"].emit(q=.3)
        runtime._step()
        self.assertFalse(self.hands["left"].commands)
        self.assertFalse(self.hands["right"].commands)
        self.assertEqual(runtime.state, SystemState.PAUSED)

    def test_source_failure_during_other_side_solve_is_rechecked_before_submit(self):
        runtime = self.make()
        runtime.engage()
        self.maps["right"].hook = lambda: self.gloves["left"].emit(valid=False)
        self.gloves["right"].emit()
        runtime._step()
        self.assertFalse(self.hands["left"].commands)
        self.assertEqual(runtime.state, SystemState.PAUSED)

    def test_partial_submission_failure_is_not_claimed_atomic(self):
        runtime = self.make()
        runtime.engage()
        self.hands["right"].accept = False
        runtime._step(); self.join_disables()
        self.assertEqual(len(self.hands["left"].commands), 1)
        self.assertFalse(self.hands["right"].commands)
        self.assertFalse(self.hands["right"].enabled)
        self.assertEqual(runtime.state, SystemState.PAUSED)

    def test_bad_solver_is_rejected_before_any_enable(self):
        runtime = self.make()
        self.maps["right"].fail = True
        with self.assertRaises(ValueError): runtime.prepare_engage()
        self.assertTrue(all(not h.enabled and h.profile is None for h in self.hands.values()))

    def test_prepare_cancelled_during_enable_cannot_later_begin_follow(self):
        runtime = self.make()
        self.hands["right"].engage_hook = lambda: runtime.pause("Space")
        with self.assertRaises(RuntimeError): runtime.prepare_engage()
        with self.assertRaises(RuntimeError): runtime.begin_follow()
        self.assertEqual(runtime.state, SystemState.PAUSED)
        runtime._step()
        self.assertTrue(self.hands["left"].enabled)

    def test_pause_between_prepare_and_follow_invalidates_token(self):
        runtime = self.make()
        runtime.prepare_engage(); runtime.pause("Space")
        with self.assertRaises(RuntimeError): runtime.begin_follow()

    def test_single_side_and_close_all_even_after_one_failure(self):
        runtime = self.make(sides=("right",))
        runtime.engage(); runtime._step(); runtime.close()
        self.assertTrue(self.hands["right"].closed and self.gloves["right"].closed)
        self.assertEqual(runtime.state, SystemState.CLOSED)
        self.assertFalse(self.hands["right"].enabled)

    def test_close_error_is_reported_after_all_cleanup(self):
        runtime = self.make()
        self.hands["left"].close_error = RuntimeError("parameter restore failed")
        with self.assertRaisesRegex(RuntimeError, "parameter restore failed"): runtime.close()
        self.assertTrue(all(d.closed for d in (*self.hands.values(), *self.gloves.values())))

    def test_default_model_change_pauses_following(self):
        runtime = self.make()
        runtime.engage()
        runtime.session = SimpleNamespace(health=lambda: Health(False, self.clock(), "SDK user changed"),
                                          close=lambda: None)
        runtime._step()
        self.assertEqual(runtime.state, SystemState.PAUSED)
        self.assertFalse(runtime.health().ready)

    def test_worker_runs_independently_and_skips_overdue_periods(self):
        runtime = self.make(clock=time.monotonic_ns, threaded=True)
        runtime.engage()
        self.maps["left"].hook = lambda: time.sleep(.025)
        self.gloves["left"].emit()
        time.sleep(.09)
        runtime.pause("done")
        self.assertGreater(runtime.missed_periods, 0)
        self.assertGreaterEqual(len(self.hands["left"].commands), 2)
        self.assertLess(len(self.hands["left"].commands), 14)

    def test_dead_worker_cannot_be_reengaged_with_healthy_feedback(self):
        runtime = self.make()
        runtime.engage()
        runtime._step = lambda: (_ for _ in ()).throw(TypeError("unexpected worker failure"))
        runtime._run()
        self.join_disables()
        self.assertFalse(runtime.health().ready)
        self.assertTrue(runtime._stop.is_set())
        with self.assertRaisesRegex(RuntimeError, "worker stopped"): runtime.prepare_engage()
        self.assertTrue(all(not h.enabled for h in self.hands.values()))

    def test_actual_rate_counts_only_worker_cycles(self):
        runtime = self.make()
        runtime._step()
        self.clock.advance(.01)
        runtime._step()
        for _ in range(50): runtime.tick()
        self.assertEqual(runtime.status()["control_hz_actual"], 100.)
        self.assertEqual(runtime.cycles, 2)

    def test_disable_rpc_failure_needs_new_explicit_enable(self):
        runtime = self.make()
        runtime.engage()
        self.hands["left"].disable_hook = lambda: (_ for _ in ()).throw(RuntimeError("network lost"))
        self.hands["left"].fault = "bad feedback"
        runtime._step(); self.join_disables()
        self.assertIn("left", runtime._blocked_hands)
        self.assertTrue(any(e.kind == "wuji.disable_failed" for e in self.sink.events))
        self.hands["left"].disable_hook = None
        self.refresh()
        runtime.engage()
        self.assertEqual(self.hands["left"].calls.count("engage"), 2)
        self.assertNotIn("left", runtime._blocked_hands)


class RetargetTests(unittest.TestCase):
    def sample(self):
        return Sample(SampleHeader(SampleRef("skeleton", "test", 1), 1, True),
            HandSkeleton("left_wrist", tuple(str(i) for i in range(21)),
                         ((0., 0., 0.),)*21, (1.,)*21, ()))

    def test_official_side_model_float32_order_and_mechanical_limits(self):
        import numpy as np
        calls = []
        class Session:
            def step(self, points):
                self.points = points
                return np.array([-100, 100]*10)
            def reset(self): calls.append("reset")
        def for_hand(model, *, side):
            calls.append((model, side))
            return Session()
        sdk = SimpleNamespace(RetargetSession=SimpleNamespace(for_hand=for_hand),
            HandModel=SimpleNamespace(WujiHand2="Hand2"),
            Handedness=SimpleNamespace(Left="L", Right="R"))
        for side, expected in (("left", "L"), ("right", "R")):
            mapper = WujiHandRetargeter(side, sdk=sdk)
            result = mapper.solve(self.sample())
            self.assertIn(("Hand2", expected), calls)
            self.assertEqual(result.joint_names, JOINT_NAMES)
            self.assertEqual(result.position_rad,
                tuple(limit[i % 2] for i, limit in enumerate(JOINT_LIMITS_RAD)))
            self.assertEqual(mapper._session.points.dtype, np.float32)
            mapper.reset()
        self.assertEqual(calls.count("reset"), 2)

    def test_invalid_input_and_native_output_are_not_fabricated(self):
        import numpy as np
        mapper = WujiHandRetargeter("left")
        sample = self.sample()
        for positions in (((0.,)*3,)*20, ((math.nan, 0., 0.),)*21):
            with self.assertRaises(ValueError):
                mapper.solve(replace(sample, payload=replace(sample.payload, positions_m=positions)))
        for q in (np.zeros(19), np.full(20, math.nan)):
            mapper._session = SimpleNamespace(step=lambda points: q)
            with self.assertRaises(ValueError): mapper.solve(sample)

    @unittest.skipUnless(importlib.util.find_spec("wuji_sdk"), "optional Wuji SDK not installed")
    def test_native_both_side_and_reset_smoke_without_manager(self):
        sample = self.sample()
        points = [(0., 0., 0.)]
        for finger in range(5):
            points.extend(((finger-2)*.018, .025 + joint*.023, 0.) for joint in range(4))
        sample = replace(sample, payload=replace(sample.payload, positions_m=tuple(points)))
        for side in ("left", "right"):
            mapper = WujiHandRetargeter(side)
            first = mapper.solve(sample)
            mapper.reset()
            second = mapper.solve(sample)
            self.assertEqual(len(first.position_rad), 20)
            self.assertTrue(all(math.isfinite(x) for x in first.position_rad))
            self.assertEqual(first.position_rad, second.position_rad)

    def test_factory_does_not_import_or_connect_sdk(self):
        config = {"profile_id": "test", "sdk_user_name": "yuchen",
                  "parameters": {"kp": 5, "kd": .05, "current_limit_a": 1.5},
                  "devices": {s: {"glove": "192.168.1.100:50001", "hand": "192.168.1.110:7447"}
                              for s in ("left", "right")}}
        runtime = create_wuji_teleop(config, sides=("left",))
        self.assertEqual(runtime.state, SystemState.DISCONNECTED)
        self.assertIsNone(runtime.session.manager)
        self.assertEqual(runtime.session.user_name, "yuchen")
        self.assertIsNone(runtime.retargeters["left"]._session)
        for selection in ({"sdk_user_name": 123}, {"sdk_user_name": " "},
                          {"sdk_user_id": "ambiguous"}):
            with self.assertRaises(ValueError):
                create_wuji_teleop({**config, **selection}, sides=("left",))
        with self.assertRaisesRegex(ValueError, "sdk_user_name"):
            create_wuji_teleop({**config, "sdk_user_name": "", "sdk_user_id": "old"}, sides=("left",))


if __name__ == "__main__":
    unittest.main()
