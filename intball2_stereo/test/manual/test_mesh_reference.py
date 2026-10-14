import unittest
import numpy as np

from mesh_reference import TriangleRaycaster, render


def plane(z):
    return np.array([[[-10, -10, z], [10, -10, z], [10, 10, z]],
                     [[-10, -10, z], [10, 10, z], [-10, 10, z]]], dtype=float)


class MeshReferenceTests(unittest.TestCase):
    def test_nonunit_rays_return_optical_axis_depth_and_nearest_face(self):
        caster = TriangleRaycaster(np.concatenate([plane(3), plane(2)]), leaf_size=1)
        depth, faces = caster.raycast([0, 0, 0], [[0, 0, 1], [1, 0, 1], [20, 0, 1]], 4)
        np.testing.assert_allclose(depth[:2], [2, 2])
        self.assertTrue(np.isnan(depth[2]))
        self.assertTrue(np.all(faces[:2] >= 2))
        self.assertEqual(faces[2], -1)

    def test_backface_sensitivity_and_parallel_box_miss(self):
        caster = TriangleRaycaster(plane(2), leaf_size=1)
        depth, _ = caster.raycast([0, 0, 0], [[0, 0, 1]], 3, front_faces_only=True)
        self.assertTrue(np.isnan(depth[0]))
        caster = TriangleRaycaster(plane(2)[:, ::-1], leaf_size=1)
        depth, _ = caster.raycast([0, 0, 0], [[0, 0, 1]], 3, front_faces_only=True)
        self.assertEqual(depth[0], 2)
        depth, _ = caster.raycast([20, 0, 0], [[0, 0, 1]], 3)
        self.assertTrue(np.isnan(depth[0]))

    def test_near_surface_occludes_far_surface_before_depth_range_filter(self):
        caster = TriangleRaycaster(np.concatenate([plane(0.1), plane(2)]))
        pose = {'parent': 'iss_body', 'world_to_optical': np.eye(4)}
        calibration = {'k': np.eye(3), 'width': 1, 'height': 1}
        depth, _ = render(caster, calibration, pose, 0.25, 3)
        self.assertTrue(np.isnan(depth[0, 0]))

    def test_rigid_transform_preserves_optical_depth(self):
        caster = TriangleRaycaster(plane(2))
        transform = np.eye(4)
        transform[:3, 3] = [1, 0, 1]
        pose = {'parent': 'iss_body', 'world_to_optical': transform}
        calibration = {'k': np.eye(3), 'width': 1, 'height': 1}
        depth, _ = render(caster, calibration, pose, 0.25, 3)
        self.assertEqual(depth[0, 0], 1)

    def test_virtual_reference_euclidean_range_is_distinct_from_z_cutoff(self):
        caster = TriangleRaycaster(plane(2))
        pose = {'parent': 'iss_body', 'world_to_optical': np.eye(4)}
        calibration = {'k': np.eye(3), 'width': 3, 'height': 1}
        z_depth, _ = render(caster, calibration, pose, 0.25, 3)
        range_depth, _ = render(caster, calibration, pose, 0.25, 3, max_euclidean_range=3)
        np.testing.assert_allclose(z_depth, [[2, 2, 2]])
        np.testing.assert_allclose(range_depth[0, :2], [2, 2])
        self.assertTrue(np.isnan(range_depth[0, 2]))


if __name__ == '__main__':
    unittest.main()
