"""Numerical evidence for the M6 command envelope's float32 reserve.

The SDK serializes degree-valued double inputs as FX_FLOAT (float32) in
Robot.cpp::OnWriteIntFloat. These tests do not load its library or send data.
"""

import ctypes
import math
import unittest

from bimanual_teleop.devices.tianji.model import M6Model


WRIST_RESERVE_DEG = 1e-5


def wire_degrees(degrees):
    # Include the same Python radians-to-degrees conversion as the driver,
    # followed by the native SDK's float32 conversion.
    return ctypes.c_float(math.degrees(math.radians(degrees))).value


def wrist_margin_degrees(j6, j7):
    return 110.5 - 1.025 * abs(j6) - abs(j7)


class TianjiCommandPrecisionTests(unittest.TestCase):
    def test_strictly_inside_double_command_can_round_outside_every_wrist_face(self):
        self.assertEqual(ctypes.sizeof(ctypes.c_float), 4)
        for sign6 in (-1, 1):
            for sign7 in (-1, 1):
                with self.subTest(sign6=sign6, sign7=sign7):
                    j6 = sign6 * 32.000276565561755
                    j7 = sign7 * 77.69971084595727
                    self.assertGreater(wrist_margin_degrees(j6, j7), 5e-6)
                    encoded = wire_degrees(j6), wire_degrees(j7)
                    self.assertLess(wrist_margin_degrees(*encoded), -9e-8)

    def test_reserve_exceeds_the_global_half_ulp_error_bound(self):
        # |J6| <= 60: float32 half-ULP <= 2^-19 degrees.
        # |J7| <= 90: float32 half-ULP <= 2^-18 degrees.
        # Each wrist facet therefore changes by at most this weighted sum.
        maximum_rounding_error = 1.025 * 2 ** -19 + 2 ** -18
        self.assertAlmostEqual(maximum_rounding_error, 5.769729614257812e-6)
        self.assertGreater(WRIST_RESERVE_DEG, maximum_rounding_error)
        # In radians this is about 1e-7, not a degree-sized safety margin.
        self.assertLess(math.radians(maximum_rounding_error), 1.008e-7)

    def test_reserved_wrist_faces_survive_rounding_near_float32_cell_boundaries(self):
        # Cover float32 exponent transitions and the 64-degree J7 transition.
        # 160 adjacent J6 cells cover every relative rounding phase of the
        # rational slope 1.025 = 41/40 in these exponent regions.
        count = 0
        for base in (20., 31.99, 32., 40., 45.36, 49., 59.99):
            _, exponent = math.frexp(base)
            ulp6 = 2. ** (exponent - 24)
            base = wire_degrees(base)
            for cell in range(160):
                for offset in (-.500001, -.5, -.499999, 0., .499999, .5, .500001):
                    j6 = base + (cell + offset) * ulp6
                    j7 = 110.5 - 1.025 * j6 - WRIST_RESERVE_DEG
                    if not (0 <= j6 <= 60 and 0 <= j7 <= 90):
                        continue
                    for sign6 in (-1, 1):
                        for sign7 in (-1, 1):
                            encoded = wire_degrees(sign6 * j6), wire_degrees(sign7 * j7)
                            self.assertGreaterEqual(wrist_margin_degrees(*encoded), 0.,
                                                    f"rounding violated the reserved face: {j6}, {j7}")
                            count += 1
        self.assertGreater(count, 30_000)

    def test_individual_integer_joint_limits_are_exact_float32_endpoints(self):
        model = M6Model.from_file()
        for side in ("left", "right"):
            for positive, negative, *_ in model.arm(side).limits_native:
                self.assertEqual(wire_degrees(positive), positive)
                self.assertEqual(wire_degrees(negative), negative)
                for value in (negative, math.nextafter(negative, positive),
                              math.nextafter(positive, negative), positive):
                    encoded = wire_degrees(value)
                    self.assertGreaterEqual(encoded, negative)
                    self.assertLessEqual(encoded, positive)


if __name__ == "__main__":
    unittest.main()
