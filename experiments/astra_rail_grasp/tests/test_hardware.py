"""Hardware adapter contract tests; all device classes are replaced by fakes."""

from dataclasses import replace
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch
from uuid import uuid4

from experiments.astra_rail_grasp import hardware as hw
from bimanual_teleop.types import Health, JointState, JointTarget, Pose, Sample, SampleHeader, SampleRef, Submission


CALLS = []


def sample(payload, stream="test"):
    return Sample(SampleHeader(SampleRef(stream, "test", 1), hw.time.monotonic_ns(), True), payload)


class FakeLease:
    def __init__(self, resources):
        self.resources = resources

    def acquire(self):
        CALLS.append("lease.acquire")

    def release(self):
        CALLS.append("lease.release")


class FakeKinematics:
    def fk(self, side, q):
        return Pose(f"tianji_{side}_base", f"tianji_{side}_flange", (.3, .2, .1), (0., 0., 0., 1.))


class FakeDriver:
    fail_start = False

    def __init__(self, ip):
        self.ip = ip
        self.fail_close = False
        self.fail_hold = False
        self.engaged = False
        self.arms = {side: SimpleNamespace(
            joints=JointState(tuple(f"j{i}" for i in range(7)), (0.,) * 7, (0.,) * 7),
            state=0, error=0, native_current_permille=(None,) * 7) for side in hw.SIDES}
        self.feedback = sample(SimpleNamespace(arms=self.arms))

    def start(self):
        CALLS.append("arms.start")
        if self.fail_start:
            raise RuntimeError("offline controller")

    def health(self, *, sides=None):
        self.checked_sides = sides
        ready = all(self.arms[side].error == 0 for side in sides)
        return Health(ready, hw.time.monotonic_ns(), None if ready else "controller fault")

    def get_latest(self):
        return self.feedback

    def request_hold(self, reason):
        CALLS.append("arms.hold")
        self.engaged = False
        if self.fail_hold:
            raise RuntimeError("hold rejected")

    def close(self):
        CALLS.append("arms.close")
        if self.fail_close:
            raise RuntimeError("servo-off not confirmed")


class FakeExecutor:
    def __init__(self, driver, kinematics):
        self.driver = driver
        self.kinematics = kinematics
        self.engagement_poses = {}
        self.commands = []
        self.reject = False

    def configure(self, profile):
        CALLS.append("arms.configure")
        self.profile = profile

    def engage(self):
        CALLS.append("arms.engage")
        self.driver.engaged = True
        self.engagement_poses = {side: self.kinematics.fk(side, (0.,) * 7)
                                 for side in self.profile.parameters["active_arms"]}

    def submit(self, target):
        self.commands.append(target)
        return Submission(target.command_id, not self.reject, "synthetic rejection" if self.reject else None)

    def request_hold(self, reason):
        self.driver.request_hold(reason)


class FakeHand:
    fail_start_side = None

    def __init__(self, side, address, **kwargs):
        self.side = side
        self.address = address
        self.device_id = f"wuji_{side}_hand"
        self.enabled = False
        self.fail_close = self.fail_disable = False
        self._enable_attempted = False
        self.commands = []
        self.feedback = sample(JointState(hw.JOINT_NAMES, (.1,) * 20, (0.,) * 20, (.02,) * 20))
        self.diagnostics = sample(hw.WujiDiagnostics(("Disabled",) * 20, (0,) * 20,
                                                    ((False, False, False),) * 20, 0, 0))

    def start(self):
        CALLS.append(f"{self.side}.start")
        if self.side == self.fail_start_side:
            raise RuntimeError("hand offline")

    def health(self, *, check_latch=False):
        return Health(True, hw.time.monotonic_ns())

    def get_latest(self):
        return self.feedback

    def get_latest_stream(self, stream):
        assert stream == "diagnostics"
        return self.diagnostics

    def configure(self, profile):
        CALLS.append(f"{self.side}.configure")
        self.profile = profile

    def engage(self):
        CALLS.append(f"{self.side}.engage")
        self.enabled = self._enable_attempted = True
        self.last_target = JointTarget(hw.JOINT_NAMES, self.feedback.payload.position_rad)
        self.diagnostics = sample(replace(self.diagnostics.payload, states=("Enabled",) * 20))

    def submit(self, target):
        self.commands.append(target)
        return Submission(target.command_id, self.enabled, None if self.enabled else "not enabled")

    def disable(self, reason):
        CALLS.append(f"{self.side}.disable")
        if self.fail_disable:
            raise RuntimeError("disable rejected")
        self.enabled = self._enable_attempted = False

    def close(self):
        CALLS.append(f"{self.side}.close")
        if self.fail_close:
            raise RuntimeError("disconnect failed")


class HardwareTests(unittest.TestCase):
    def setUp(self):
        CALLS.clear()
        FakeDriver.fail_start = False
        FakeHand.fail_start_side = None
        for name, value in (("_HardwareLease", FakeLease), ("TianjiDriver", FakeDriver),
                            ("TianjiKinematics", FakeKinematics), ("TianjiCartesianExecutor", FakeExecutor),
                            ("WujiHandDriver", FakeHand)):
            patcher = patch.object(hw, name, value)
            patcher.start()
            self.addCleanup(patcher.stop)

    def backend(self, sides=hw.SIDES):
        backend = hw.HardwareBackend(sides=sides)
        self.addCleanup(backend.close)
        return backend

    def test_construction_open_snapshot_close_never_configure_or_enable(self):
        backend = self.backend()
        self.assertEqual(CALLS, [])
        backend.open()
        state = backend.snapshot()
        self.assertTrue(state["healthy"], state["problems"])
        self.assertFalse(state["hands"]["left"]["enabled"])
        self.assertEqual(state["arms"]["right"]["pose"]["position_m"], [.3, .2, .1])
        json.dumps(state, allow_nan=False)
        backend.close()
        self.assertEqual(CALLS, ["lease.acquire", "arms.start", "left.start", "right.start",
                                 "arms.close", "left.close", "right.close", "lease.release"])

    def test_hand_partial_open_failure_retains_other_devices_and_cleans_all(self):
        FakeHand.fail_start_side = "left"
        backend = self.backend()
        backend.open()
        state = backend.snapshot()
        self.assertFalse(state["healthy"])
        self.assertTrue(any("left hand connection" in p for p in state["problems"]))
        self.assertIsNotNone(state["arms"]["right"]["pose"])
        self.assertIsNotNone(state["hands"]["right"]["position_rad"])
        with self.assertRaisesRegex(RuntimeError, "not healthy"):
            backend.configure()
        self.assertNotIn("arms.configure", CALLS)
        backend.close()
        self.assertEqual(CALLS[-4:], ["arms.close", "left.close", "right.close", "lease.release"])

    def test_arm_connection_failure_still_attempts_both_hands(self):
        FakeDriver.fail_start = True
        backend = self.backend()
        backend.open()
        self.assertIn("right.start", CALLS)
        self.assertFalse(backend.snapshot()["healthy"])
        self.assertIn("offline controller", " ".join(backend.snapshot()["problems"]))

    def test_right_only_ignores_left_controller_fault_and_does_not_open_left_hand(self):
        backend = self.backend(("right",))
        backend.open()
        backend.driver.arms["left"].state = 100
        backend.driver.arms["left"].error = 4
        state = backend.snapshot()
        self.assertTrue(state["healthy"], state["problems"])
        self.assertEqual(state["arms"]["left"]["error"], 4)
        self.assertIsNone(state["hands"]["left"]["position_rad"])
        self.assertEqual(backend.driver.checked_sides, ("right",))
        self.assertEqual(set(backend.hands), {"right"})
        self.assertTrue(any(resource.startswith("tianji:") for resource in backend._lease.resources))
        self.assertEqual(len(backend._lease.resources), 2)
        backend.configure()
        self.assertEqual(backend.arm_profile.parameters["active_arms"], ["right"])
        with self.assertRaises(ValueError):
            backend.engage_hand("left")

    def test_unknown_diagnostics_and_channels_remain_unknown(self):
        backend = self.backend()
        backend.open()
        hand = backend.hands["left"]
        hand.feedback = sample(replace(hand.feedback.payload, motor_current_a=(math.nan,) * 20))
        hand.diagnostics = None
        state = backend.snapshot()
        self.assertIsNone(state["hands"]["left"]["enabled"])
        self.assertEqual(state["hands"]["left"]["current_a"], [None] * 20)
        json.dumps(state, allow_nan=False)
        with self.assertRaisesRegex(RuntimeError, "not healthy"):
            backend.configure()

    def test_initial_joint_frame_before_diagnostics_does_not_latch_startup_fault(self):
        backend = self.backend(("right",))
        backend.open()
        hand = backend.hands["right"]
        diag, hand.diagnostics = hand.diagnostics, None
        with patch.object(hand, "health", wraps=hand.health) as health:
            self.assertFalse(backend.snapshot()["healthy"])
            health.assert_not_called()
            hand.diagnostics = diag
            self.assertTrue(backend.snapshot()["healthy"])
            health.assert_called_once_with(check_latch=True)

    def test_configure_is_explicit_and_preserves_tool_parameters(self):
        backend = self.backend()
        backend.open()
        backend.configure()
        for arm in backend.arm_profile.parameters["arms"].values():
            self.assertEqual((arm["velocity_ratio"], arm["acceleration_ratio"]), (5, 5))
            self.assertEqual(len(arm["tool_dyn10"]), 10)
        self.assertEqual(backend.hand_profile.parameters, {"kp": 3., "kd": .05, "current_limit_a": .5})
        self.assertNotIn("arms.engage", CALLS)
        self.assertNotIn("left.engage", CALLS)

    def test_selected_fault_and_existing_active_state_prevent_configuration(self):
        backend = self.backend(("right",))
        backend.open()
        backend.driver.arms["right"].error = 4
        with self.assertRaisesRegex(RuntimeError, "not healthy"):
            backend.configure()
        backend.driver.arms["right"].error = 0
        backend.driver.arms["right"].state = 3
        with self.assertRaisesRegex(RuntimeError, "IDLE"):
            backend.configure()
        self.assertNotIn("arms.configure", CALLS)

    def test_engage_seeds_and_commands_use_correct_profile_and_fresh_ttl(self):
        backend = self.backend(("right",))
        backend.open()
        backend.configure()
        seed = backend.engage_hand("right")
        poses = backend.engage_arms()
        self.assertEqual(seed, (.1,) * 20)
        with patch.object(hw.time, "monotonic_ns", return_value=1_000_000_000):
            backend.send_hand("right", seed, 1_000_000_000)
            backend.send_arms(poses, 1_000_000_000)
        hand_target = backend.hands["right"].commands[-1]
        arm_target = backend.executor.commands[-1]
        for command in (hand_target, arm_target):
            self.assertEqual(command.expires_monotonic_ns - command.created_monotonic_ns, 50_000_000)
            self.assertEqual(len(command.source_refs), 1)
        self.assertEqual(hand_target.device_id, "wuji_right_hand")
        self.assertEqual(set(arm_target.tool_poses), {"right"})
        with patch.object(hw.time, "monotonic_ns", return_value=1_000_000_000):
            for invalid in (1_000_000_001, 950_000_000, True):
                with self.assertRaises(ValueError):
                    backend.send_hand("right", seed, invalid)

    def test_command_rejections_propagate(self):
        backend = self.backend(("right",))
        backend.open()
        backend.configure()
        with self.assertRaisesRegex(RuntimeError, "not enabled"):
            backend.send_hand("right", (0.,) * 20, hw.time.monotonic_ns())
        backend.executor.reject = True
        with self.assertRaisesRegex(RuntimeError, "synthetic rejection"):
            backend.send_arms({}, hw.time.monotonic_ns())

    def test_fault_stop_attempts_every_device_and_latches_before_future_commands(self):
        backend = self.backend()
        backend.open()
        backend.configure()
        backend.driver.fail_hold = True
        backend.hands["left"].fail_disable = True
        with self.assertRaisesRegex(RuntimeError, "hold rejected.*disable rejected"):
            backend.fault_stop("tracking mismatch")
        self.assertEqual(len(backend.errors), 2)
        self.assertIn("right.disable", CALLS)
        state = backend.snapshot()
        self.assertFalse(state["healthy"])
        self.assertTrue(all(error in state["problems"] for error in backend.errors))
        with self.assertRaisesRegex(RuntimeError, "not configured"):
            backend.send_arms({}, hw.time.monotonic_ns())

    def test_close_aggregates_failures_releases_lock_and_is_idempotent(self):
        backend = self.backend()
        backend.open()
        backend.driver.fail_close = True
        backend.hands["left"].fail_close = True
        with self.assertRaisesRegex(RuntimeError, "servo-off not confirmed.*disconnect failed"):
            backend.close()
        self.assertEqual(CALLS[-4:], ["arms.close", "left.close", "right.close", "lease.release"])
        prior = list(CALLS)
        backend.close()
        self.assertEqual(CALLS, prior)


class LockTests(unittest.TestCase):
    def test_real_advisory_lock_contends_and_can_be_reacquired_after_release(self):
        resource = "test-rail-lock:" + uuid4().hex
        key = hashlib.sha256(resource.encode()).hexdigest()[:24]
        path = Path("/tmp") / f"astra-rail-hardware-{key}.lock"
        first, second = hw._HardwareLease([resource]), hw._HardwareLease([resource])
        try:
            with patch.object(hw, "_find_local_controllers", return_value=[]):
                first.acquire()
                with self.assertRaisesRegex(RuntimeError, "busy"):
                    second.acquire()
                self.assertEqual(second.fds, [])
                first.release()
                second.acquire()
        finally:
            first.release()
            second.release()
            path.unlink(missing_ok=True)

    def test_proc_scan_recognizes_sdk_and_commands_without_matching_shell_text(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            for pid, args, maps in (("999991", b"python\0/x/home_tianji.py\0", ""),
                                    ("999992", b"python\0other.py\0", "/sdk/libMarvinSDK.so"),
                                    ("999993", b"bash\0-c\0cat home_tianji.py\0", "")):
                entry = root / pid
                entry.mkdir()
                (entry / "cmdline").write_bytes(args)
                (entry / "maps").write_text(maps)
            conflicts = hw._find_local_controllers(root)
            self.assertEqual(len(conflicts), 2)
            self.assertFalse(any("999993" in conflict for conflict in conflicts))


if __name__ == "__main__":
    unittest.main()
