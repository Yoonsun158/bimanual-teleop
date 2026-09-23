"""Lifecycle and concurrency boundaries; no SDK or camera hardware is opened."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import tempfile
import threading
import unittest
from unittest.mock import patch
import uuid

from experiments.astra_rail_grasp.fake import FakeBackend
from experiments.astra_rail_grasp.runtime import Runner


class Clock:
    def __init__(self):
        self.now = 1_000_000_000

    def __call__(self):
        return self.now


class Camera:
    def __init__(self, clock):
        self.clock = clock
        self.ready = True

    def start(self):
        pass

    def close(self):
        self.ready = False

    def health(self):
        return {"ready": self.ready}

    def snapshot(self):
        return {"ready": self.ready, "cameras": [
            {"age_s": 0., "captured_monotonic_ns": self.clock()}]}


class RuntimeEdges(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.clock = Clock()
        self.backend = FakeBackend(("right",), clock=self.clock)
        self.camera = Camera(self.clock)
        self.runner = Runner(self.backend, self.camera, self.directory.name,
                             allow_motion=True, clock=self.clock)
        # Drive ticks explicitly so edge timing is deterministic.
        self.runner._worker = lambda *args: None
        self.runner.start()

    def tearDown(self):
        if self.runner.state != "closed":
            self.runner.possible_load = False
            self.runner.action = None
            self.runner.shutdown()
        self.directory.cleanup()

    def visual(self, **args):
        self.clock.now += 1
        observation = self.runner.observe()
        return dict(observation_id=observation["observation_id"], note="synthetic visual judgement", **args)

    def engage(self):
        self.runner.engage(self.visual(onsite_ready=True))
        self.runner.tick(self.clock())

    def request(self, op, args=None, **extra):
        return dict(op=op, args=args or {}, session_id=self.runner.session_id,
                    id=uuid.uuid4().hex, expires_ns=self.clock()+1_000_000_000, **extra)

    def test_observation_session_cannot_enable_or_configure(self):
        self.runner.allow_motion = False
        self.runner.observe()
        self.runner.status()
        with self.assertRaisesRegex(ValueError, "allow-motion"):
            self.runner.engage(self.visual(onsite_ready=True))
        self.assertEqual(self.backend.calls, ["open"])

    def test_right_only_rejects_left_targets_without_touching_right(self):
        self.engage()
        start = deepcopy(self.runner.arm_targets)
        with self.assertRaisesRegex(ValueError, "selected sides"):
            self.runner.arm_step(self.visual(moves={"left": {"translation_mm": [0, 10, 0]}}), "left")
        self.assertEqual(self.runner.arm_targets, start)
        self.assertEqual(set(self.backend.hand_enabled), {"right"})
        self.assertNotIn("engage_hand:left", self.backend.calls)

    def test_expired_or_other_session_requests_never_change_targets(self):
        self.engage()
        args = self.visual(moves={"right": {"translation_mm": [0, 10, 0]}})
        request = self.request("arm-step", args)
        request["expires_ns"] = self.clock()-1
        with self.assertRaisesRegex(ValueError, "expired"):
            self.runner.handle(request)
        request["expires_ns"] = self.clock()+1_000_000_000
        request["session_id"] = "old-session"
        with self.assertRaisesRegex(ValueError, "session_id"):
            self.runner.handle(request)
        self.assertIsNone(self.runner.action)
        self.assertEqual(self.runner.travel_mm, 0)

    def test_duplicate_returns_original_result_and_different_payload_is_rejected(self):
        self.engage()
        request = self.request("arm-step", self.visual(moves={"right": {"translation_mm": [0, 10, 0]}}))
        first = self.runner.handle(request)
        self.clock.now += 2_000_000_000  # Retry after request expiry still cannot replay.
        self.assertEqual(self.runner.handle(request), first)
        self.assertEqual(self.runner.travel_mm, 10.)
        different = deepcopy(request)
        different["args"]["moves"]["right"]["translation_mm"] = [10, 0, 0]
        with self.assertRaisesRegex(ValueError, "different content"):
            self.runner.handle(different)

    def test_concurrent_duplicate_waits_for_exactly_one_operation(self):
        entered, release = threading.Event(), threading.Event()
        calls = []

        def operation(_):
            calls.append("pause")
            entered.set()
            self.assertTrue(release.wait(1))
            return {"state": "paused"}

        request = self.request("pause")
        with patch.object(self.runner, "pause", side_effect=operation), ThreadPoolExecutor(2) as pool:
            first = pool.submit(self.runner.handle, request)
            self.assertTrue(entered.wait(1))
            second = pool.submit(self.runner.handle, request)
            release.set()
            self.assertEqual(first.result(timeout=1), second.result(timeout=1))
        self.assertEqual(calls, ["pause"])

    def test_camera_loss_at_final_validation_does_not_consume_travel(self):
        self.engage()
        args = self.visual(moves={"right": {"translation_mm": [0, 10, 0]}})
        with patch.object(self.camera, "health", side_effect=[{"ready": True}, {"ready": False}]):
            with self.assertRaisesRegex(ValueError, "camera"):
                self.runner.arm_step(args, "lost-camera")
        self.assertEqual(self.runner.travel_mm, 0.)
        self.assertIsNone(self.runner.action)
        self.assertTrue(self.runner.observation["valid"])

    def test_audit_failure_does_not_set_load_or_accept_hand_action(self):
        self.engage()
        args = self.visual(side="right", purpose="grasp", joints_deg={"finger1_joint1": 1})
        with patch.object(self.runner, "log", side_effect=RuntimeError("disk failed")):
            with self.assertRaisesRegex(RuntimeError, "disk failed"):
                self.runner.hand_step(args, "audit-failed")
        self.assertFalse(self.runner.possible_load)
        self.assertEqual(self.runner.stage, "empty")
        self.assertIsNone(self.runner.action)
        self.assertTrue(self.runner.observation["valid"])

    def test_visual_marks_cannot_skip_load_lifecycle_or_reuse_observation(self):
        self.engage()
        with self.assertRaisesRegex(ValueError, "returned"):
            self.runner.mark(self.visual(what="supported"))
        self.runner.possible_load, self.runner.stage = True, "contact"
        args = self.visual(what="grasped")
        self.runner.mark(args)
        with self.assertRaisesRegex(ValueError, "observation_id"):
            self.runner.mark(dict(args, what="supported"))
        self.runner.mark(self.visual(what="supported"))
        with self.assertRaisesRegex(ValueError, "release steps"):
            self.runner.mark(self.visual(what="released"))
        with self.assertRaisesRegex(ValueError, "change at least"):
            self.runner.hand_step(self.visual(side="right", purpose="release",
                                   joints_deg={"finger1_joint1": 0}), "zero-release")
        self.assertTrue(self.runner.possible_load)

    def test_regrasp_discards_prior_release_evidence(self):
        self.engage()
        self.runner.possible_load, self.runner.stage = True, "supported"
        self.runner.release_sides = {"right"}
        self.runner.hand_step(self.visual(side="right", purpose="grasp",
                              joints_deg={"finger1_joint1": 1}), "regrasp")
        self.assertEqual(self.runner.stage, "contact")
        self.assertEqual(self.runner.release_sides, set())

    def test_possible_load_blocks_shutdown_without_closing_devices(self):
        self.engage()
        self.runner.possible_load = True
        with self.assertRaisesRegex(ValueError, "possibly loaded"):
            self.runner.shutdown()
        self.assertFalse(self.backend.closed)
        self.assertEqual(self.runner.state, "running")

    def test_pause_waits_for_inflight_motion_and_next_submission_is_hold(self):
        self.engage()
        self.runner.arm_step(self.visual(moves={"right": {"translation_mm": [0, 10, 0]}}), "step")
        self.clock.now += 1_500_000_000  # Midpoint of the 3-second, 10 mm segment.
        entered, release, paused = threading.Event(), threading.Event(), threading.Event()
        original_send, original_log = self.backend.send_arms, self.runner.log
        targets = []

        def blocked(poses, now):
            targets.append(deepcopy(poses))
            entered.set()
            if not release.wait(1):
                raise RuntimeError("test submission not released")
            original_send(poses, now)

        def log(kind, **details):
            original_log(kind, **details)
            if kind == "paused":
                paused.set()

        with patch.object(self.backend, "send_arms", side_effect=blocked), \
                patch.object(self.runner, "log", side_effect=log), ThreadPoolExecutor(2) as pool:
            send = pool.submit(self.runner._send_arms, self.clock())
            self.assertTrue(entered.wait(1))
            pause = pool.submit(self.runner.pause)
            self.assertTrue(paused.wait(1))
            self.assertFalse(pause.done())
            release.set()
            send.result(timeout=1)
            self.assertEqual(pause.result(timeout=1)["state"], "paused")
        self.runner._send_arms(self.clock())
        self.assertEqual(self.backend.poses, self.runner.arm_targets)
        self.assertNotEqual(targets[0], self.runner.arm_targets)
        self.assertIsNone(self.runner.action)


if __name__ == "__main__":
    unittest.main()
