"""Check retargeting geometry and compositing without robot assets or rendering."""
import unittest
from xml.etree import ElementTree

import numpy as np
from scipy.spatial.transform import Rotation
import trimesh

from grasp_retargeting.mesh_geometry import (
    homogeneous,
    inverse_points,
    make_collision_grid,
    mano_joints,
    sample_collision_grid,
    transform_points,
)
from grasp_retargeting.mesh_preview import compose_depth
from grasp_retargeting.native_hands import position_control_matrix
from grasp_retargeting.native_retarget import pinch_pose


class RetargetingGeometryTests(unittest.TestCase):
    def test_native_control_map_preserves_coupled_joint_motion(self):
        xml = ElementTree.fromstring("""
            <mujoco>
              <tendon><fixed name="distal_pair">
                <joint joint="middle" coef="1"/><joint joint="distal" coef="1"/>
              </fixed></tendon>
              <actuator>
                <position name="flex" tendon="distal_pair"/>
                <position name="spread" joint="knuckle"/>
              </actuator>
            </mujoco>
        """)
        matrix = position_control_matrix(xml, ["distal", "knuckle", "middle"], ["spread", "flex"])
        np.testing.assert_array_equal(matrix, [[0, 1, 0], [1, 0, 1]])
        np.testing.assert_allclose(matrix @ [0.25, 0.1, 0.5], [0.1, 0.75])

    def test_pinch_pose_aligns_both_tips_and_wrist(self):
        tips = np.array([[-0.04, 0, 0.15], [0.04, 0, 0.15]])
        rotation = Rotation.from_euler("xyz", [0.3, -0.2, 0.8]).as_matrix()
        wrist = np.array([0.1, 0.2, 0.6])
        target = tips @ rotation.T + wrist

        pose = pinch_pose(tips, target, wrist)

        self.assertEqual(pose.shape, (4, 4))
        np.testing.assert_allclose(pose[:3, :3], rotation, atol=1e-10)
        np.testing.assert_allclose(pose[:3, 3], wrist, atol=1e-10)
        np.testing.assert_allclose(inverse_points(target, pose), tips, atol=1e-10)

    def test_camera_transform_promotes_three_rows_and_rejects_reflection(self):
        camera = homogeneous([[0, -1, 0, 0.2], [1, 0, 0, 0], [0, 0, 1, 0.8]])
        point = np.array([[0.01, 0.02, 0.03]])
        np.testing.assert_allclose(transform_points(point, camera), [[0.18, 0.01, 0.83]])
        with self.assertRaises(ValueError):
            homogeneous(np.diag([1, 1, -1, 1]))

    def test_mano_joint_order_exposes_five_distinct_fingertips(self):
        vertices = np.arange(778 * 3, dtype=float).reshape(778, 3)
        regressor = np.eye(16, 778)
        joints = mano_joints(vertices, regressor)
        self.assertEqual(joints.shape, (21, 3))
        np.testing.assert_array_equal(joints[0], vertices[0])
        # The public 21-joint layout ends each thumb-to-little-finger chain with its tip.
        np.testing.assert_array_equal(joints[[4, 8, 12, 16, 20]], vertices[[744, 320, 443, 554, 671]])

    def test_collision_grid_distinguishes_penetration_and_clearance_in_metres(self):
        obj = trimesh.creation.box(extents=[0.06, 0.08, 0.10])
        field, origin, spacing, _ = make_collision_grid(obj, resolution=25)
        points = [[0.01, 0, 0], [0.03, 0, 0], [0.04, 0, 0], [1, 0, 0]]
        distances = sample_collision_grid(points, field, origin, spacing)
        np.testing.assert_allclose(distances[:3], [-0.02, 0, 0.01], atol=1e-6)
        self.assertGreater(distances[3], 0)


class CompositingTests(unittest.TestCase):
    def test_depth_orders_robot_object_and_background_and_masks_human_pixels(self):
        source = np.full((1, 5, 3), 200, dtype=np.uint8)
        background = np.full_like(source, 10)
        robot = np.full_like(source, 80)
        object_render = np.full_like(source, 120)
        result, source_visible, robot_front = compose_depth(
            source, background, robot, np.array([[0.4, 0.6, 0.5, 0.0, 0.0]]),
            object_render, np.array([[0.5, 0.5, 0.5, 0.5, 0.0]]),
            np.array([[0, 0, 0.4, 0, 0]]), np.array([[True, True, True, False, False]]),
        )
        np.testing.assert_array_equal(result[0, :, 0], [80, 200, 120, 120, 10])
        np.testing.assert_array_equal(source_visible, [[True, True, False, False, False]])
        np.testing.assert_array_equal(robot_front, [[True, False, False, False, False]])


if __name__ == "__main__":
    unittest.main()
