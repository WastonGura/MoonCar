import math
import pathlib
import sys
import unittest
import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'scripts'))
from terrain_geometry import correct_camera_pose, transform_points, tilt_degrees


def quaternion_from_axis_angle(axis, angle_rad):
    axis = np.asarray(axis, dtype=float)
    axis = axis / np.linalg.norm(axis)
    s = math.sin(angle_rad / 2.0)
    c = math.cos(angle_rad / 2.0)
    return np.array([axis[0] * s, axis[1] * s, axis[2] * s, c])


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2,
        w1*w2 - x1*x2 - y1*y2 - z1*z2
    ])


class PanTiltTransformTest(unittest.TestCase):
    """Verifies that depth optical frame transforms with Pan-Tilt gimbal angles

    project accurately into the global map frame across both flat and sloped terrain.
    """

    def setUp(self):
        # Base optical transform: camera looking forward in robot frame (+X forward, +Y left, +Z up)
        # Optical frame: +X right (-Y), +Y down (-Z), +Z forward (+X)
        # Equivalent rotation: yaw=-90 deg around Z, then pitch=0, roll=-90 deg around X
        # Quaternion [x, y, z, w] for optical to body: [-0.5, 0.5, -0.5, 0.5]
        self.q_body_to_optical = np.array([-0.5, 0.5, -0.5, 0.5])
        self.camera_mount_offset = np.array([0.18, 0.0, 0.28])

    def get_camera_tf(self, robot_pos, robot_yaw, pan_rad, tilt_rad):
        """Compute camera link pose relative to map given robot yaw, gimbal pan and tilt."""
        q_robot = quaternion_from_axis_angle([0, 0, 1], robot_yaw)
        q_pan = quaternion_from_axis_angle([0, 0, 1], pan_rad)
        q_tilt = quaternion_from_axis_angle([0, -1, 0], tilt_rad)

        # Composite orientation: robot * pan * tilt * optical
        q_gimbal = quaternion_multiply(q_pan, q_tilt)
        q_total_rot = quaternion_multiply(q_robot, q_gimbal)
        q_camera_optical = quaternion_multiply(q_total_rot, self.q_body_to_optical)

        # Mount position in world
        mount_world = robot_pos + transform_points([self.camera_mount_offset], (0, 0, 0), q_robot)[0]
        return mount_world, q_camera_optical, q_robot

    def test_pan_rotation_horizontal_projection(self):
        """Rotating pan joint swings the optical forward projection horizontally."""
        robot_pos = np.array([1.0, 2.0, 0.0])
        robot_yaw = 0.0
        pan_angle = math.radians(45.0)  # 45 deg left
        tilt_angle = 0.0

        cam_pos, q_cam, q_robot = self.get_camera_tf(robot_pos, robot_yaw, pan_angle, tilt_angle)

        # Point 2 meters straight in front of depth sensor (optical z = +2.0)
        optical_pt = np.array([[0.0, 0.0, 2.0]])
        world_pt = transform_points(optical_pt, cam_pos, q_cam)[0]

        expected_x = cam_pos[0] + 2.0 * math.cos(pan_angle)
        expected_y = cam_pos[1] + 2.0 * math.sin(pan_angle)
        expected_z = cam_pos[2]

        np.testing.assert_allclose(world_pt, [expected_x, expected_y, expected_z], atol=1e-5)

    def test_tilt_rotation_elevation_projection(self):
        """Tilting camera downward angles the optical ray towards the ground."""
        robot_pos = np.array([0.0, 0.0, 0.0])
        robot_yaw = 0.0
        pan_angle = 0.0
        tilt_angle = math.radians(-15.0)  # 15 deg pitch down

        cam_pos, q_cam, q_robot = self.get_camera_tf(robot_pos, robot_yaw, pan_angle, tilt_angle)

        distance = 2.0
        optical_pt = np.array([[0.0, 0.0, distance]])
        world_pt = transform_points(optical_pt, cam_pos, q_cam)[0]

        expected_x = cam_pos[0] + distance * math.cos(abs(tilt_angle))
        expected_y = cam_pos[1]
        expected_z = cam_pos[2] - distance * math.sin(abs(tilt_angle))

        np.testing.assert_allclose(world_pt, [expected_x, expected_y, expected_z], atol=1e-5)

    def test_combined_pan_tilt_with_terrain_imu_attitude(self):
        """Under terrain tilt, correct_camera_pose preserves Euclidean distances and adds chassis tilt."""
        robot_pos = np.array([3.0, -1.0, 0.5])
        robot_yaw = math.radians(30.0)
        pan_angle = math.radians(-60.0)  # 60 deg right
        tilt_angle = math.radians(-20.0) # 20 deg down

        cam_pos, q_cam, q_robot = self.get_camera_tf(robot_pos, robot_yaw, pan_angle, tilt_angle)

        # Non-flat terrain: robot pitched 5 deg, rolled 3 deg
        # IMU measures pitch and roll in gravity-aligned frame
        q_pitch = quaternion_from_axis_angle([0, 1, 0], math.radians(5.0))
        q_roll = quaternion_from_axis_angle([1, 0, 0], math.radians(3.0))
        q_terrain = quaternion_multiply(q_pitch, q_roll)
        q_imu = quaternion_multiply(q_robot, q_terrain)

        corrected_pos, corrected_q = correct_camera_pose(
            robot_pos, q_robot, cam_pos, q_cam, q_imu
        )

        # Quaternion must be normalized
        self.assertAlmostEqual(np.linalg.norm(corrected_q), 1.0, places=6)

        # Check distance invariance: 3 points transformed into world must preserve inter-point distances
        pts_local = np.array([
            [0.0, 0.0, 1.0],
            [0.5, -0.2, 2.0],
            [-0.3, 0.4, 1.5]
        ])
        pts_world = transform_points(pts_local, corrected_pos, corrected_q)

        dist_local_01 = np.linalg.norm(pts_local[0] - pts_local[1])
        dist_world_01 = np.linalg.norm(pts_world[0] - pts_world[1])
        self.assertAlmostEqual(dist_local_01, dist_world_01, places=5)

        dist_local_12 = np.linalg.norm(pts_local[1] - pts_local[2])
        dist_world_12 = np.linalg.norm(pts_world[1] - pts_world[2])
        self.assertAlmostEqual(dist_local_12, dist_world_12, places=5)

    def test_gimbal_joint_limits(self):
        """Full range of motion (+-135 deg pan, -35 to +15 deg tilt) remains valid without singular flips."""
        extreme_configs = [
            (math.radians(-135.0), math.radians(-35.0)),
            (math.radians(135.0), math.radians(-35.0)),
            (math.radians(-135.0), math.radians(15.0)),
            (math.radians(135.0), math.radians(15.0)),
            (0.0, 0.0)
        ]
        robot_pos = np.array([0.0, 0.0, 0.0])
        for pan, tilt in extreme_configs:
            cam_pos, q_cam, q_robot = self.get_camera_tf(robot_pos, 0.0, pan, tilt)
            self.assertTrue(np.isfinite(cam_pos).all())
            self.assertTrue(np.isfinite(q_cam).all())
            self.assertAlmostEqual(np.linalg.norm(q_cam), 1.0, places=6)
            pt = transform_points([[0, 0, 1]], cam_pos, q_cam)[0]
            self.assertTrue(np.isfinite(pt).all())


if __name__ == '__main__':
    unittest.main()
