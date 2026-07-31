#!/usr/bin/env python3

import importlib.util
import math
import os
import unittest


SCRIPT = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "scripts",
    "one_key_takeoff_hover_land.py",
)
SPEC = importlib.util.spec_from_file_location("one_key_takeoff_hover_land", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


class SafetyMathTest(unittest.TestCase):
    def test_quaternion_sign_equivalence(self):
        quaternion = MODULE.yaw_to_quaternion(math.radians(-49.669))
        negated = tuple(-value for value in quaternion)
        self.assertAlmostEqual(MODULE.quaternion_angle_deg(quaternion, negated), 0.0, places=7)

    def test_quaternion_difference(self):
        first = MODULE.yaw_to_quaternion(0.0)
        second = MODULE.yaw_to_quaternion(math.radians(5.0))
        self.assertAlmostEqual(MODULE.quaternion_angle_deg(first, second), 5.0, places=6)

    def test_ramp_toward_never_overshoots(self):
        self.assertEqual(MODULE.ramp_toward(0.0, 0.5, 0.1), 0.1)
        self.assertEqual(MODULE.ramp_toward(0.49, 0.5, 0.1), 0.5)
        self.assertEqual(MODULE.ramp_toward(0.5, 0.0, 0.2), 0.3)

    def test_invalid_quaternion_is_rejected(self):
        self.assertIsNone(MODULE.normalize_quaternion((0.0, 0.0, 0.0, 0.0)))
        self.assertTrue(math.isinf(MODULE.quaternion_angle_deg((0, 0, 0, 0), (0, 0, 0, 1))))

    def test_maximum_upward_excursion(self):
        sample = MODULE.PoseSample
        samples = [
            sample(0.0, 0.0, (0.0, 0.0, 0.10), (0.0, 0.0, 0.0, 1.0)),
            sample(1.0, 1.0, (0.0, 0.0, 0.05), (0.0, 0.0, 0.0, 1.0)),
            sample(2.0, 2.0, (0.0, 0.0, 0.36), (0.0, 0.0, 0.0, 1.0)),
            sample(3.0, 3.0, (0.0, 0.0, 0.20), (0.0, 0.0, 0.0, 1.0)),
        ]
        dz, start, end = MODULE.maximum_upward_excursion(samples)
        self.assertAlmostEqual(dz, 0.31)
        self.assertEqual(start, 1.0)
        self.assertEqual(end, 2.0)


if __name__ == "__main__":
    unittest.main()
