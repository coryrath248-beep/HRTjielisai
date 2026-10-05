import math
import unittest

import numpy as np

from footroom_debug import FootRoomShadow, ManualPathReference, Pose2D, gravity_from_rpy


class PathReferenceTests(unittest.TestCase):
    def test_manual_nudge_resets_origin_on_release(self):
        path = ManualPathReference()
        straight = (1.2, 0.0, 0.0)
        _, error, mode = path.update(straight, Pose2D(0, 0, 0))
        self.assertEqual((error, mode), ((0.0, 0.0), "straight"))
        _, error, _ = path.update(straight, Pose2D(2.0, 0.4, 0.1))
        self.assertAlmostEqual(error[0], 0.4)
        self.assertAlmostEqual(error[1], 0.1)
        command, error, mode = path.update((1.2, 0, 0.12), Pose2D(2, 0.4, 0.1))
        self.assertEqual((error, mode), ((0.0, 0.0), "manual_turn"))
        self.assertAlmostEqual(command[2], 0.12)
        _, error, mode = path.update(straight, Pose2D(2.3, 0.5, 0.2))
        self.assertEqual((error, mode), ((0.0, 0.0), "straight"))

    def test_stick_noise_does_not_interrupt_straight_reference(self):
        path = ManualPathReference()
        path.update((1.0, 0, 0), Pose2D(0, 0, 0))
        command, error, mode = path.update((1.0, 0, 0.03), Pose2D(1, 0.2, 0))
        self.assertEqual(mode, "straight")
        self.assertEqual(command[2], 0.0)
        self.assertAlmostEqual(error[0], 0.2)

    def test_straight_requires_odometry(self):
        with self.assertRaisesRegex(ValueError, "odometry"):
            ManualPathReference().update((1.0, 0, 0), None)


class ActorContractTests(unittest.TestCase):
    def test_gravity_and_actor_layout(self):
        np.testing.assert_allclose(gravity_from_rpy((0, 0, 0)), (0, 0, -1))
        shadow = FootRoomShadow()
        result = shadow.step(1.0, shadow.default_q, np.zeros(23), (0, 0, 0),
                             (0, 0, 0), (1.6, 0, 0), Pose2D(0, 0, 0))
        obs = result["observation"]
        self.assertEqual(len(obs), 80)
        self.assertEqual(len(result["action"]), 21)
        self.assertEqual(len(result["targets_urdf_rad"]), 23)
        self.assertEqual(obs[:3], [0.0, -0.0, -1.0])
        self.assertAlmostEqual(obs[6], 1.6)
        self.assertEqual(obs[78:80], [0.0, 0.0])
        self.assertTrue(all(math.isfinite(v) for v in result["targets_urdf_rad"]))
        self.assertEqual(result["mode"], "straight")

    def test_turning_has_zero_path_features_and_keeps_forward_command(self):
        shadow = FootRoomShadow()
        result = shadow.step(1.0, shadow.default_q, np.zeros(23), (0, 0, 0),
                             (0, 0, 0), (0.8, 0, 0.2), None)
        self.assertEqual(result["mode"], "manual_turn")
        self.assertEqual(result["observation"][78:80], [0.0, 0.0])
        self.assertAlmostEqual(result["command"][0], 0.8)
        self.assertAlmostEqual(result["command"][2], 0.2)


if __name__ == "__main__":
    unittest.main()
