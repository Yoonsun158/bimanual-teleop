"""Check the official local Jacobian against FK; no device connection."""

import math
from pathlib import Path
import random
import sys
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from bimanual_teleop.devices.tianji.model import (  # noqa: E402
    KinematicsError, TianjiKinematics, _KINE_LOCK, _matrix_from_pose,
)


# Independent right-arm references from the seven 2026-09-15 failure snapshots.
# These are configurations, not seven consecutive points of a recorded path.
RECORDED_REFERENCES_DEG = (
    (-134.29116617910117, -91.43614369541307, 95.777610788848, -144.98952814355047,
     -82.47976097603735, 17.00631028010176, 20.0793169753757),
    (-127.23352964352469, -31.452411793809684, 92.85075934256383, -135.05833219577502,
     -32.7497901578303, 59.938637067274165, 3.4624599348217537),
    (-161.037251334445, -11.238977603441866, 120.70322124390688, -127.95959335420936,
     -4.758534466973669, 44.69760515610464, -16.103157534628078),
    (-169.70229588771608, -6.633505041634923, 129.6190492562913, -127.11301117925225,
     -0.7699396469051867, 46.468423432126286, -15.720737716537606),
    (-168.6438629920325, -6.139830330069175, 129.70271331339438, -126.58523485367108,
     -0.005147606360424106, 45.572707883821636, -16.287952590459557),
    (-169.91496899473995, -5.622241040986864, 131.14112474820774, -126.4983474592807,
     0.7892729721965566, 45.831532190343594, -16.332862444319463),
    (-169.85213081975155, -5.068419715078109, 132.5834747545535, -126.49468036743589,
     1.3942053764677669, 46.14596008307809, -15.968651356685488),
)


def finite_difference(fk, joints, step=1e-5):
    """Spatial angular derivative: vee(dR/dq R^T), with radians as input."""
    result = np.zeros((6, 7))
    _, rotation = fk(joints)
    for joint in range(7):
        positive, negative = list(joints), list(joints)
        positive[joint] += step
        negative[joint] -= step
        p_plus, r_plus = fk(positive)
        p_minus, r_minus = fk(negative)
        result[:3, joint] = (p_plus - p_minus) / (2 * step)
        skew = ((r_plus - r_minus) / (2 * step)) @ rotation.T
        result[3:, joint] = ((skew[2, 1] - skew[1, 2]) / 2,
                            (skew[0, 2] - skew[2, 0]) / 2,
                            (skew[1, 0] - skew[0, 1]) / 2)
    return result


class TianjiJacobianTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.kine = TianjiKinematics()

    def pose_reader(self, side):
        def fk(joints):
            pose = self.kine.fk(side, joints)
            matrix = np.array(_matrix_from_pose(pose, side)).reshape(4, 4)
            return np.array(pose.position_m), matrix[:3, :3]
        return fk

    def assert_jacobian(self, side, joints):
        actual = np.array(self.kine.jacobian(side, joints))
        self.assertEqual(actual.shape, (6, 7))
        expected = finite_difference(self.pose_reader(side), joints)
        # A degree/radian error is a factor of 57.3; a mistaken FK-style
        # millimetre conversion is a factor of 1000. Both must fail this test.
        np.testing.assert_allclose(actual, expected, rtol=2e-7, atol=2e-8)

    def test_random_valid_configurations_on_both_arms(self):
        rng = random.Random(20260915)
        for side in ("left", "right"):
            arm = self.kine.model.arm(side)
            valid = 0
            for _ in range(200):
                joints = tuple(rng.uniform(low + .01, high - .01)
                               for low, high in zip(arm.lower_rad, arm.upper_rad))
                try:
                    # Use SDK single/coupled-limit and singularity checks to
                    # establish the sampled configuration's validity first.
                    self.kine.ik(side, self.kine.fk(side, joints), joints)
                except KinematicsError:
                    continue
                with self.subTest(side=side, sample=valid):
                    self.assert_jacobian(side, joints)
                valid += 1
                if valid == 32:
                    break
            self.assertEqual(valid, 32, "Insufficient valid random configurations")

    def test_seven_real_near_limit_references(self):
        for index, degrees in enumerate(RECORDED_REFERENCES_DEG, 1):
            with self.subTest(snapshot=index):
                joints = tuple(math.radians(value) for value in degrees)
                self.assert_jacobian("right", joints)

    def test_zero_configuration_returns_rank_deficient_jacobian(self):
        for side in ("left", "right"):
            joints = (0.,) * 7
            self.assert_jacobian(side, joints)
            self.assertLess(np.linalg.matrix_rank(self.kine.jacobian(side, joints)), 6)

    def test_configured_tcp_offset_changes_linear_rows_in_base_coordinates(self):
        # Exercise the official SDK with a nonidentity tool: the SDK Jacobian
        # follows the configured TCP. The public Python model uses identity
        # tool and therefore exposes a flange Jacobian. Restore it afterwards.
        joints = tuple(math.radians(q) for q in (12, -22, 31, -48, 17, 15, -12))
        tool = (1., 0., 0., 30., 0., 0., -1., -20., 0., 1., 0., 50., 0., 0., 0., 1.)

        with _KINE_LOCK:
            official = self.kine._initialize("right")
            flange = np.array(self.kine.jacobian("right", joints))
            def raw_fk(values):
                matrix = np.array(official.fk([math.degrees(q) for q in values]))
                return matrix[:3, 3] / 1000, matrix[:3, :3]
            try:
                self.assertTrue(official.set_tool_kine(np.array(tool).reshape(4,4).tolist()))
                actual = np.array(official.joints2JacobMatrix([math.degrees(q) for q in joints]))
                expected = finite_difference(raw_fk, joints)
                np.testing.assert_allclose(actual, expected, rtol=2e-7, atol=2e-8)
                self.assertGreater(np.max(np.abs(actual[:3] - flange[:3])), .01)
                np.testing.assert_allclose(actual[3:], flange[3:], atol=1e-12)
            finally:
                self.assertTrue(official.remove_tool_kine())
        self.assert_jacobian("right", joints)

    def test_invalid_input_nonfinite_result_and_missing_interface_are_errors(self):
        for joints in ((0.,) * 6, (math.nan,) * 7, (math.inf,) * 7):
            with self.assertRaises(KinematicsError):
                self.kine.jacobian("left", joints)
        with self.assertRaisesRegex(KinematicsError, "Unknown arm"):
            self.kine.jacobian("both", (0.,) * 7)
        official = self.kine._initialize("left")
        with patch.object(official, "joints2JacobMatrix", return_value=False):
            with self.assertRaisesRegex(KinematicsError, "Jacobian.*failed"):
                self.kine.jacobian("left", (0.,) * 7)
            self.kine.fk("left", (0.,) * 7)
        with patch.object(official, "joints2JacobMatrix", return_value=[[math.nan]*7]*6):
            with self.assertRaisesRegex(KinematicsError, "nonfinite"):
                self.kine.jacobian("left", (0.,) * 7)


if __name__ == "__main__":
    unittest.main()
