"""Paper-based pose following: filter equivalence, unrestricted goals and device isolation."""

from bimanual_teleop.devices.tianji.config import load_config
import importlib.util
import math
from pathlib import Path
import unittest
from unittest.mock import patch

from bimanual_teleop.devices.tianji.model import DEFAULT_LIBRARY, MotionProfile, TianjiKinematics
from bimanual_teleop.system import SystemState
from bimanual_teleop.control.arm.mapping import PoseGoalFilter, _from_matrix, matmul, rotation_matrix
from bimanual_teleop.types import ControlProfile, Pose, RobotTarget
from test_quest_tianji_runtime import RuntimeFixture, SIDES

ROOT = Path(__file__).resolve().parents[1]


class JointFollowTests(unittest.TestCase):
    @unittest.skipUnless(importlib.util.find_spec("scipy"), "SciPy required for upstream Filter comparison")
    def test_filter_matches_openteach_scipy_implementation(self):
        import numpy as np
        from scipy.spatial.transform import Rotation, Slerp
        rng = np.random.default_rng(729)
        initial = {s: Pose(f"{s}_base", f"{s}_tool", (0., 0., 0.), (0., 0., 0., 1.)) for s in SIDES}
        filter_ = PoseGoalFilter(initial)
        pos_state, ori_state = np.zeros(3), np.zeros(3)
        # Independent SciPy reference follows the authors' Filter.__call__.
        # Noncommuting rotations and rotations crossing pi exercise quaternion
        # conversion/sign differences between this port and the upstream rotvec.
        for i in range(100):
            position = rng.normal(size=3)
            rotation = Rotation.random(random_state=rng)
            pos_state = pos_state*.8 + position*.2
            ori_interp = Slerp([0, 1], Rotation.from_rotvec(np.stack([ori_state, rotation.as_rotvec()])))
            ori_state = ori_interp([.2])[0].as_rotvec()
            poses = {s: Pose(f"{s}_base", f"{s}_tool", tuple(position), tuple(rotation.as_quat())) for s in SIDES}
            goal = RobotTarget(str(i), poses, {}, (), i, i+100, "test")
            actual = filter_.update(goal)
            self.assertEqual(actual.expires_monotonic_ns, goal.expires_monotonic_ns)
            for pose in actual.tool_poses.values():
                np.testing.assert_allclose(pose.position_m, pos_state, atol=1e-12)
                delta = Rotation.from_quat(pose.orientation_xyzw) * Rotation.from_rotvec(ori_state).inv()
                self.assertLess(delta.magnitude(), 1e-7)

    def test_filter_updates_on_new_quest_frames_not_on_each_send_tick(self):
        fx = RuntimeFixture(motion=False)
        self.addCleanup(fx.runtime.close)
        fx.runtime.engage()
        fx.runtime.tick()
        fx.advance(positions={"left": (0., .3, 1.)})
        self.assertIsNotNone(fx.runtime.tick())
        filtered = dict(fx.runtime._filter._poses)
        for _ in range(5):
            fx.advance(quest=False)
            self.assertIsNotNone(fx.runtime.tick(), fx.runtime.last_error)
            self.assertEqual(fx.runtime._filter._poses, filtered)

    def test_motion_and_preview_follow_without_extra_joint_speed_cap(self):
        for motion in (True, False):
            with self.subTest(motion=motion):
                fx = RuntimeFixture(motion=motion)
                ik = fx.kine.ik

                def sensitive_ik(side, pose, reference):
                    q = list(ik(side, pose, reference))
                    q[4] = 20 * (pose.position_m[2] - .5)
                    return tuple(q)

                fx.kine.ik = sensitive_ik
                with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock):
                    fx.runtime.engage()
                    self.assertIsNotNone(fx.runtime.tick(), fx.runtime.last_error)
                    origin = fx.runtime._last_target.tool_poses
                    previous_q = dict(fx.driver.q) if motion else dict(fx.runtime._preview_q)
                    peak_step = 0.
                    for i in range(600):
                        fx.advance(20_000_000 if i == 40 else 5_000_000,
                                   positions={"left": (0., .23, 1.), "right": (0., -.17, 1.)})
                        target = fx.runtime.tick()
                        self.assertIsNotNone(target, fx.runtime.last_error)
                        current_q = fx.driver.q if motion else fx.runtime._preview_q
                        for side in SIDES:
                            peak_step = max(peak_step, max(abs(a-b) for a, b in zip(
                                current_q[side], previous_q[side])))
                        previous_q = dict(current_q)
                    for side in SIDES:
                        self.assertAlmostEqual(math.dist(origin[side].position_m,
                            target.tool_poses[side].position_m), .03, places=6)
                    self.assertGreater(peak_step, math.radians(18)*.005)
                    self.assertEqual(fx.runtime.state, SystemState.ENGAGED)
                    self.assertEqual(fx.driver.holds, [])
                fx.runtime.close()

    def test_tracking_loss_during_joint_planning_prevents_dispatch(self):
        fx = RuntimeFixture()
        self.addCleanup(fx.runtime.close)
        with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock):
            fx.runtime.engage()
            self.assertIsNotNone(fx.runtime.tick())
            before = len(fx.driver.commands)
            def loss():
                fx.quest.emit(invalid="right")
                fx.quest.emit()
            fx.kine.after_ik = loss
            fx.advance()
            self.assertIsNone(fx.runtime.tick())
            self.assertEqual(len(fx.driver.commands), before)
            self.assertEqual(fx.runtime.state, SystemState.PAUSED)

    @unittest.skipUnless(DEFAULT_LIBRARY.exists(), "Build official SDK bridge for M6 regression")
    def test_ready_pose_official_ik_follows_six_axes_with_real_profile(self):
        # Device acquisition and dispatch remain in memory. Only official FK/IK
        # and the actual motion profile are used; no robot connection is made.
        fx = RuntimeFixture()
        self.addCleanup(fx.runtime.close)
        config = load_config(ROOT / "configs/tianji_teleop.yaml")
        profile = ControlProfile(**config["profile"])
        parsed = MotionProfile.from_control_profile(profile)
        ready = config["ready_pose"]["target_deg"]
        kinematics = TianjiKinematics()
        fx.runtime.profile = fx.profile = profile
        fx.runtime.motion_profile = fx.driver.parsed = parsed
        fx.runtime.kinematics = fx.executor.kinematics = kinematics
        fx.driver.q = {side: tuple(map(math.radians, ready[side])) for side in SIDES}
        fx.driver.emit()
        with patch("bimanual_teleop.control.arm.cartesian.time.monotonic_ns", side_effect=fx.clock):
            fx.runtime.engage()
            self.assertIsNotNone(fx.runtime.tick(), fx.runtime.last_error)
            origins = dict(fx.quest.positions)
            rotations = {s: getattr(fx.quest.latest.payload, s).orientation_xyzw for s in (*SIDES, "head")}
            previous = dict(fx.driver.q)
            peak_step = 0.
            for i in range(1, 2401):
                # Fast hand inputs: 4 cm / 10 degree excursions along each of
                # six axes, including reversals, 90 Hz frames and a late tick.
                segment, phase = divmod(i-1, 400)
                angle = 2*math.pi*phase/399
                displacement = .04*math.sin(angle)
                turn = math.radians(10)*math.sin(angle)
                positions, orientations = {}, dict(rotations)
                for side in SIDES:
                    p = list(origins[side])
                    if segment < 3:
                        p[segment] += displacement
                    else:
                        q = [0., 0., 0., math.cos(turn/2)]
                        q[segment-3] = math.sin(turn/2)
                        orientations[side] = _from_matrix(matmul(rotation_matrix(tuple(q)),
                                                                rotation_matrix(rotations[side])))
                    positions[side] = tuple(p)
                fx.advance(20_000_000 if i == 550 else 5_000_000, quest=False)
                if i % 20 // 2 != (i-1) % 20 // 2 and i % 20 != 18:
                    fx.quest.emit(positions=positions, rotations=orientations)
                target = fx.runtime.tick()
                self.assertIsNotNone(target, f"cycle {i}: {fx.runtime.last_error}")
                for side in SIDES:
                    for a, b in zip(fx.driver.q[side], previous[side]):
                        peak_step = max(peak_step, abs(a-b))
                previous = dict(fx.driver.q)
            self.assertGreater(peak_step, math.radians(18)*.005)
            self.assertEqual(fx.runtime.state, SystemState.ENGAGED)
            self.assertEqual(fx.driver.holds, [])


if __name__ == "__main__":
    unittest.main()
