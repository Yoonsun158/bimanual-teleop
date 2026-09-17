"""Pinned M6 model validation and local FK/IK; no controller connection."""

from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bimanual_teleop.devices.tianji.model import (  # noqa: E402
    DEFAULT_MODEL, IKTargetError, KinematicsError, M6Model,
    MotionProfile, ProfileError, TianjiKinematics, _matrix_from_pose, _pose_from_matrix,
)
from bimanual_teleop.types import ControlProfile, Pose
from bimanual_teleop.devices.tianji.sdk import load_sdk  # noqa: E402


class TianjiModelTests(unittest.TestCase):
    def setUp(self):
        self.model = M6Model.from_file()
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)

    def profile(self):
        """Synthetic values validate schema only; these are not robot settings."""
        arm = {
            "stiffness": [1]*6, "damping": [1]*6,
            "nullspace_stiffness": 1, "nullspace_damping": 1,
            "tool_dyn10": [1, 0, 0, 0, 0.01, 0, 0, 0.01, 0, 0.01],
            "velocity_ratio": 10, "acceleration_ratio": 10,
        }
        return ControlProfile("test", "cartesian_impedance", {
            "active_arms": ["left"], "arms": {"left": arm},
        })

    def test_nominal_geometry_units_and_both_arms(self):
        self.assertEqual(self.model.arm("left").controller_type, 1017)
        self.assertEqual(self.model.arm("right").dof, 7)
        self.assertEqual(self.model.arm("left").dh_native[0], (0, 0, 174.5, 0))
        self.assertEqual(self.model.arm("left").dh_native[-1], (90, 0, 95, 90))
        self.assertAlmostEqual(self.model.arm("left").lower_rad[0], math.radians(-170))
        self.assertEqual(self.model.arm("right").gravity_m_s2, (0, -9.81, 0))
        self.assertEqual(self.model.digest, hashlib.sha256(DEFAULT_MODEL.read_bytes()).hexdigest())

    def test_modified_nominal_cannot_claim_pinned_m6_provenance(self):
        modified = Path(self.directory.name) / "changed.MvKDCfg"
        modified.write_bytes(DEFAULT_MODEL.read_bytes().replace(b"174.500000", b"174.600000"))
        with self.assertRaises(ProfileError):
            M6Model.from_file(modified)

    def test_profile_requires_sdk_parameters_without_provenance_gates(self):
        result = MotionProfile.from_control_profile(self.profile())
        self.assertEqual(result.active_arms, ("left",))
        self.assertEqual(result.arms["left"].velocity_ratio, 10)

        for field, value in [("stiffness", None), ("tool_dyn10", [-1]+[0]*9), ("velocity_ratio", True)]:
            with self.subTest(field=field):
                profile = deepcopy(self.profile())
                profile.parameters["arms"]["left"][field] = value
                with self.assertRaises(ProfileError):
                    MotionProfile.from_control_profile(profile)

    def test_incomplete_motion_parameters_are_rejected(self):
        for field in ("nullspace_damping", "tool_dyn10"):
            with self.subTest(field=field):
                profile = self.profile()
                profile.parameters["arms"]["left"][field] = None
                with self.assertRaises(ProfileError):
                    MotionProfile.from_control_profile(profile)

    def test_verified_unloaded_profile_is_accepted_for_both_arms(self):
        profile = self.profile()
        profile.parameters["active_arms"] = ["left", "right"]
        profile.parameters["arms"]["left"]["tool_dyn10"] = [0]*10
        profile.parameters["arms"]["right"] = deepcopy(profile.parameters["arms"]["left"])
        parsed = MotionProfile.from_control_profile(profile)
        for side in ("left", "right"):
            self.assertEqual(parsed.arms[side].tool_dyn10, (0.0,)*10)

    def test_zero_mass_cannot_have_nonzero_other_dynamics(self):
        for index in range(1, 10):
            with self.subTest(index=index):
                profile = self.profile()
                dynamics = [0]*10
                dynamics[index] = 0.1
                profile.parameters["arms"]["left"]["tool_dyn10"] = dynamics
                with self.assertRaisesRegex(ProfileError, "unloaded tool dynamics"):
                    MotionProfile.from_control_profile(profile)

    def test_damping_matches_native_closed_unit_interval(self):
        for value in (0, 1):
            profile = self.profile()
            arm = profile.parameters["arms"]["left"]
            arm.update(damping=[value]*6, nullspace_damping=value)
            MotionProfile.from_control_profile(profile)
        for field in ("damping", "nullspace_damping"):
            for value in (-0.001, 1.001):
                with self.subTest(field=field, value=value):
                    profile = self.profile()
                    profile.parameters["arms"]["left"][field] = [0]*5+[value] if field == "damping" else value
                    with self.assertRaises(ProfileError):
                        MotionProfile.from_control_profile(profile)


class TianjiPoseTests(unittest.TestCase):
    def test_ik_failure_contains_exact_replay_inputs_and_sdk_flags(self):
        kine = TianjiKinematics()
        def solve(side, pose, reference):
            result = load_sdk("kine").FX_InvKineSolvePara()
            result.m_OutPut_Result_Num = 1
            result.m_Output_RetJoint.data[:] = [math.degrees(q) for q in reference]
            result.m_Output_RetJoint.data[5] = -60.05
            result.m_Output_JntExdTags[5] = True
            return True, result
        kine.solve = solve
        pose = Pose("tianji_left_base", "tianji_left_flange", (.5, -.1, .3), (0., 0., 0., 1.))
        reference = tuple(map(math.radians, (30, -60, -34, -52, 30, -59.95, 4)))
        with self.assertRaises(IKTargetError) as raised:
            kine.ik("left", pose, reference)
        detail = str(raised.exception)
        self.assertIn("J6=-60.05deg", detail.splitlines()[0])
        snapshot = json.loads(detail.split("[IK诊断] ", 1)[1])
        self.assertEqual(snapshot["schema"], "tianji_ik_failure_v1")
        self.assertEqual(snapshot["model_sha256"], kine.model.digest)
        self.assertEqual(snapshot["sdk_commit"], kine.model.sdk_commit)
        self.assertEqual(snapshot["side"], "left")
        self.assertEqual(snapshot["target"]["position_m"], list(pose.position_m))
        self.assertEqual(snapshot["target"]["orientation_xyzw"], list(pose.orientation_xyzw))
        for actual, radians in zip(snapshot["reference_deg"], reference):
            self.assertAlmostEqual(actual, math.degrees(radians))
        self.assertEqual(snapshot["result_deg"][5], -60.05)
        self.assertEqual(snapshot["status"], 0)
        self.assertEqual(snapshot["limit_mask"], 32)

    def test_solver_constraint_flags_are_distinct_from_sdk_failures(self):
        pose = Pose("tianji_left_base", "tianji_left_flange", (0., 0., 0.), (0., 0., 0., 1.))
        for status, count, outside, singular, limits, recoverable in (
                (0, 0, 0, 0, 0, True),
                (-1, 0, 1, 8, 0, True),
                (-1, 0, 0, 8, 0, True),
                (0, 1, 0, 0, 32, True),
                (-1, 0, 0, 0, 0, False),
                (-1, 0, 0, 0, 0, False)):
            with self.subTest(status=status, outside=outside, singular=singular, limits=limits):
                def solve(side, pose, reference):
                    result = load_sdk("kine").FX_InvKineSolvePara()
                    result.m_OutPut_Result_Num = count
                    result.m_Output_IsOutRange = bool(outside)
                    for i in range(7):
                        result.m_Output_IsDeg[i] = bool(singular & (1 << i))
                        result.m_Output_JntExdTags[i] = bool(limits & (1 << i))
                    return status == 0, result
                kine = TianjiKinematics()
                kine.solve = solve
                with self.assertRaises(KinematicsError) as raised:
                    kine.ik("left", pose, (0.,)*7)
                self.assertEqual(isinstance(raised.exception, IKTargetError), recoverable)

    def test_nonfinite_sdk_result_is_fatal_even_with_constraint_flags(self):
        def solve(side, pose, reference):
            result = load_sdk("kine").FX_InvKineSolvePara()
            result.m_OutPut_Result_Num = 1
            result.m_Output_IsOutRange = True
            result.m_Output_RetJoint.data[0] = math.nan
            return True, result
        kine = TianjiKinematics()
        kine.solve = solve
        pose = Pose("tianji_left_base", "tianji_left_flange", (0., 0., 0.), (0., 0., 0., 1.))
        with self.assertRaisesRegex(KinematicsError, "nonfinite") as raised:
            kine.ik("left", pose, (0.,)*7)
        self.assertNotIsInstance(raised.exception, IKTargetError)
        snapshot = json.loads(str(raised.exception).split("[IK诊断] ", 1)[1])
        self.assertIsNone(snapshot["result_deg"][0])

    def test_nonzero_rotation_translation_and_half_turn(self):
        for quaternion in [(0.5, 0.5, 0.5, 0.5), (1, 0, 0, 0), (0, 1, 0, 0), (0, 0, 1, 0)]:
            pose = Pose("tianji_left_base", "tianji_left_flange", (0.1, -0.2, 0.3), quaternion)
            matrix = _matrix_from_pose(pose, "left")
            self.assertEqual((matrix[3], matrix[7], matrix[11]), (100, -200, 300))
            restored = _pose_from_matrix(matrix, "left")
            self.assertEqual(restored.position_m, pose.position_m)
            self.assertAlmostEqual(abs(sum(a*b for a, b in zip(restored.orientation_xyzw, quaternion))), 1)
        matrix = _matrix_from_pose(Pose("tianji_left_base", "tianji_left_flange", (0, 0, 0),
                                      (0.5, 0.5, 0.5, 0.5)), "left")
        self.assertEqual((matrix[0], matrix[4], matrix[8]), (0, 1, 0))  # x -> y

    def test_rejects_wrong_frames_and_nonunit_quaternion(self):
        for parent, q in [("world", (0, 0, 0, 1)), ("tianji_left_base", (1, 1, 1, 1))]:
            with self.assertRaises(KinematicsError):
                _matrix_from_pose(Pose(parent, "tianji_left_flange", (0, 0, 0), q), "left")

    def test_official_compiled_fk_ik_fk(self):
        kine = TianjiKinematics()
        reference = tuple(math.radians(value) for value in (12, -22, 31, -48, 17, 15, -12))
        for side in ("left", "right"):
            pose = kine.fk(side, reference)
            result = kine.ik(side, pose, reference)
            restored = kine.fk(side, result)
            for expected, actual in zip(pose.position_m, restored.position_m):
                self.assertAlmostEqual(expected, actual, places=6)
            self.assertAlmostEqual(abs(sum(a*b for a, b in zip(
                pose.orientation_xyzw, restored.orientation_xyzw))), 1, places=6)
            impossible = Pose(pose.parent_frame, pose.child_frame, (10, 10, 10), pose.orientation_xyzw)
            with self.assertRaises(KinematicsError):
                kine.ik(side, impossible, reference)
        singular = (0.0,)*7
        with self.assertRaisesRegex(KinematicsError, "singular"):
            kine.ik("left", kine.fk("left", singular), singular)
        coupled_limit = tuple(math.radians(value) for value in (12, -22, 31, -48, 17, 59, 89))
        with self.assertRaisesRegex(KinematicsError, "limits"):
            kine.ik("left", kine.fk("left", coupled_limit), coupled_limit)


if __name__ == "__main__":
    unittest.main()
