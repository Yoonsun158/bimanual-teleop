"""M6 model/export agreement and local FK/IK; no controller connection."""

import configparser
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bimanual_teleop.devices.tianji.model import (  # noqa: E402
    DEFAULT_LIBRARY, DEFAULT_MODEL, KinematicsError, M6Model, ModelMismatchError,
    MotionProfile, ProfileError, TianjiKinematics, _matrix_from_pose, _pose_from_matrix,
)
from bimanual_teleop.types import ControlProfile, Pose  # noqa: E402


def controller_fixture(model):
    """Represent a controller export, then mutate individual parameters in tests."""
    parser = configparser.ConfigParser()
    for index, side in enumerate(("left", "right")):
        arm, prefix = model.arm(side), f"R.A{index}"
        parser[f"{prefix}.BASIC"] = {"Dof": "7", "Type": "1017"}
        for joint, dh in enumerate(arm.dh_native):
            section = f"{prefix}.L{joint}.DH" if joint < 7 else f"{prefix}.FLANGE"
            parser[section] = dict(zip(("Alpha", "A", "D", "Theta"), map(str, dh)))
        for joint, limits in enumerate(arm.limits_native):
            parser[f"{prefix}.L{joint}.BASIC"] = dict(zip(
                ("LimitPos", "LimitNeg", "VelMax", "AccMax"), map(str, limits)))
        parser[f"{prefix}.CTRL"] = {
            f"BD67{quadrant}{coefficient}": str(value)
            for quadrant, row in zip(("PP", "NP", "NN", "PN"), arm.bd67_native)
            for coefficient, value in enumerate(row)
        }
    return parser


class TianjiModelTests(unittest.TestCase):
    def setUp(self):
        self.model = M6Model.from_file()
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.export = Path(self.directory.name) / "robot.ini"
        self.config = controller_fixture(self.model)
        self.write_export()

    def write_export(self):
        with self.export.open("w") as stream:
            self.config.write(stream)

    def profile(self):
        """Synthetic values validate schema only; these are not robot settings."""
        provenance = {"verified": True, "verified_by": "unit-test", "verified_at": "test",
                      "evidence": "synthetic fixture; not validated on hardware"}
        arm = {
            "stiffness": [1]*6, "damping": [1]*6,
            "nullspace_stiffness": 1, "nullspace_damping": 1,
            "tool_dyn10": [1, 0, 0, 0, 0.01, 0, 0, 0.01, 0, 0.01],
            "tool_load_verification": provenance,
            "max_joint_velocity_rad_s": [0.1]*7,
            "velocity_ratio": 10, "acceleration_ratio": 10,
        }
        return ControlProfile("test", "cartesian_impedance", {
            "active_arms": ["left"], "arms": {"left": arm},
            "controller_model": {
                "path": str(self.export), "sha256": hashlib.sha256(self.export.read_bytes()).hexdigest(),
                "verified_current_controller": True, "verified_by": "unit-test",
                "verified_at": "test", "evidence": "synthetic controller export",
            },
        })

    def test_nominal_geometry_units_and_both_arms(self):
        self.assertEqual(self.model.arm("left").controller_type, 1017)
        self.assertEqual(self.model.arm("right").dof, 7)
        self.assertEqual(self.model.arm("left").dh_native[0], (0, 0, 174.5, 0))
        self.assertEqual(self.model.arm("left").dh_native[-1], (90, 0, 95, 90))
        self.assertAlmostEqual(self.model.arm("left").lower_rad[0], math.radians(-170))
        self.assertEqual(self.model.arm("right").gravity_m_s2, (0, -9.81, 0))
        self.assertEqual(self.model.digest, hashlib.sha256(DEFAULT_MODEL.read_bytes()).hexdigest())

    def test_export_provenance_does_not_claim_live_verification(self):
        result = self.model.verify_controller_export(self.export)
        self.assertFalse(result.live_verified)
        self.assertEqual(result.source, "provided_export")
        downloaded = self.model.verify_controller_export(self.export, source="downloaded_controller_export")
        self.assertTrue(downloaded.connected_export_checked)
        self.assertFalse(downloaded.live_parameters_verified)

    def test_modified_nominal_cannot_claim_pinned_m6_provenance(self):
        modified = Path(self.directory.name) / "changed.MvKDCfg"
        modified.write_bytes(DEFAULT_MODEL.read_bytes().replace(b"174.500000", b"174.600000"))
        with self.assertRaises(ProfileError):
            M6Model.from_file(modified)

    def test_model_mismatches_cannot_enable_motion(self):
        for section, field in [("R.A0.L0.DH", "D"), ("R.A0.FLANGE", "Alpha"),
                               ("R.A0.L6.BASIC", "LimitPos"), ("R.A0.CTRL", "BD67PP1"),
                               ("R.A0.BASIC", "Dof")]:
            with self.subTest(section=section, field=field):
                previous = self.config[section][field]
                self.config[section][field] = str(float(previous) + 1)
                self.write_export()
                with self.assertRaises(ModelMismatchError):
                    self.model.verify_controller_export(self.export)
                self.config[section][field] = previous

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
        profile = self.profile()
        profile.parameters.pop("controller_model")
        profile.parameters["arms"]["left"].pop("tool_load_verification")
        profile.parameters["arms"]["left"].pop("max_joint_velocity_rad_s")
        result = MotionProfile.from_control_profile(profile)
        self.assertEqual(result.active_arms, ("left",))

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
            profile.parameters["arms"][side]["tool_load_verification"]["verified"] = False
            MotionProfile.from_control_profile(profile)

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

    @unittest.skipUnless(DEFAULT_LIBRARY.exists(), "Build the official Tianji bridge for native FK/IK tests")
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
