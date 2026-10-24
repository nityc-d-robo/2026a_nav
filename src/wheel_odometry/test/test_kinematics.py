import math
import unittest

import numpy as np

from wheel_odometry.kinematics import OmniOdometry


class KinematicsTest(unittest.TestCase):
    def make_core(self):
        return OmniOdometry(
            [0.2, -0.1, -0.1], [0., math.sqrt(3) * 0.1, -math.sqrt(3) * 0.1],
            [math.pi / 2, 7 * math.pi / 6, 11 * math.pi / 6], [0.05] * 3, [1.] * 3)

    def test_translation_and_yaw(self):
        for delta, distances in [
            ([1., 0., 0.], [0., -math.sqrt(3) / 2, math.sqrt(3) / 2]),
            ([0., 1., 0.], [1., -0.5, -0.5]),
            ([0., 0., 1.], [0.2, 0.2, 0.2]),
        ]:
            core = self.make_core()
            core.update([100.] * 3, 1_000_000_000)
            pose, velocity = core.update(100 + np.array(distances) / 0.05, 2_000_000_000)
            np.testing.assert_allclose(pose, delta, atol=1e-12)
            np.testing.assert_allclose(velocity, delta, atol=1e-12)

    def test_curved_motion(self):
        core = self.make_core()
        core.update([0.] * 3, 1_000_000_000)
        wheel_distance = np.array([0.2, 0.2 - math.sqrt(3) / 2, 0.2 + math.sqrt(3) / 2])
        pose, _ = core.update(wheel_distance / 0.05, 2_000_000_000)
        np.testing.assert_allclose(pose, [math.sin(1), 1 - math.cos(1), 1], atol=1e-12)

    def test_invalid_samples_leave_baseline_intact(self):
        core = self.make_core()
        core.update([0.] * 3, 1_000_000_000)
        for angles, stamp in [([0.] * 3, 1_000_000_000), ([float('nan')] * 3, 2_000_000_000)]:
            with self.assertRaises(ValueError):
                core.update(angles, stamp)
        self.assertEqual(core.previous_stamp, 1_000_000_000)

    def test_velocity_ramp_and_gap(self):
        core = self.make_core()
        forward = np.array([0., -math.sqrt(3) / 2, math.sqrt(3) / 2]) / 0.05
        core.update_velocity([0.] * 3, 1_000_000_000)
        pose, velocity = core.update_velocity(forward, 1_100_000_000)
        np.testing.assert_allclose(pose, [0.05, 0., 0.], atol=1e-12)
        np.testing.assert_allclose(velocity, [1., 0., 0.], atol=1e-12)
        pose, _ = core.update_velocity(forward, 2_000_000_000)
        np.testing.assert_allclose(pose, [0.05, 0., 0.], atol=1e-12)
        pose, _ = core.update_velocity(forward, 2_100_000_000)
        np.testing.assert_allclose(pose, [0.15, 0., 0.], atol=1e-12)

    def test_unobservable_geometry(self):
        with self.assertRaises(ValueError):
            OmniOdometry([0.] * 4, [0.] * 4, [0.] * 4, [0.05] * 4, [1.] * 4)

    def test_four_wheel_x_layout(self):
        d = 0.503 / math.sqrt(2)
        for wheels, expected in [
            ([-1., -1., 1., 1.], [1., 0., 0.]),
            ([1., -1., -1., 1.], [0., 1., 0.]),
        ]:
            core = OmniOdometry(
                [d, -d, -d, d], [d, d, -d, -d],
                [3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4, math.pi / 4],
                [0.05] * 4, [1.] * 4)
            angular = np.array(wheels) / (math.sqrt(2) * 0.05)
            core.update_velocity(angular, 1_000_000_000)
            pose, velocity = core.update_velocity(angular, 1_100_000_000)
            np.testing.assert_allclose(velocity, expected, atol=1e-12)
            np.testing.assert_allclose(pose, np.array(expected) * 0.1, atol=1e-12)

    def test_real_geometry_corner_origin(self):
        d = 0.503 / math.sqrt(2)
        core = OmniOdometry(
            [0.43 + d, 0.43 - d, 0.43 - d, 0.43 + d],
            [0.43 + d, 0.43 + d, 0.43 - d, 0.43 - d],
            [3 * math.pi / 4, 5 * math.pi / 4, 7 * math.pi / 4, math.pi / 4],
            [0.127] * 4, [1.] * 4)
        # Pure counterclockwise rotation about the robot center: the corner moves too.
        speeds = [0.503 / 0.127] * 4
        core.update_velocity(speeds, 1_000_000_000)
        pose, velocity = core.update_velocity(speeds, 1_100_000_000)
        np.testing.assert_allclose(velocity, [0.43, -0.43, 1.], atol=1e-12)
        np.testing.assert_allclose(pose, [
            0.43 * (math.sin(0.1) + 1 - math.cos(0.1)),
            0.43 * (1 - math.cos(0.1) - math.sin(0.1)), 0.1], atol=1e-12)


if __name__ == '__main__':
    unittest.main()
