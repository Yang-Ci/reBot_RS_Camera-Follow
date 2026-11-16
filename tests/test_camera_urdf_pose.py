"""Verify camera selection and optical-frame transforms without hardware."""
from pathlib import Path
import tempfile
import unittest

import numpy as np

from urdf_camera_pose import d435i_nominal_tcp_camera, gemini2_nominal_tcp_camera, _fixed_chain
import xml.etree.ElementTree as ET


class CameraUrdfPoseTests(unittest.TestCase):
    def test_color_and_infrared_use_their_own_optical_centers(self):
        arm = '''<robot name="arm"><link name="base"/><link name="wrist"/><link name="gripper_end"/>
          <joint name="joint6" type="revolute"><parent link="base"/><child link="wrist"/></joint>
          <joint name="end" type="fixed"><parent link="wrist"/><child link="gripper_end"/>
          <origin xyz="0 0 0.05"/></joint></robot>'''
        camera = '''<robot name="camera"><link name="gripper_end"/><link name="camera_link"/>
          <link name="camera_color_optical_frame"/><link name="camera_infra1_optical_frame"/>
          <joint name="mount" type="fixed"><parent link="gripper_end"/><child link="camera_link"/>
          <origin xyz="0.1 0 0"/></joint>
          <joint name="color" type="fixed"><parent link="camera_link"/><child link="camera_color_optical_frame"/>
          <origin xyz="0 0.015 0"/></joint>
          <joint name="infra" type="fixed"><parent link="camera_link"/><child link="camera_infra1_optical_frame"/>
          <origin xyz="0 0 0"/></joint></robot>'''
        with tempfile.TemporaryDirectory() as directory:
            arm_path, camera_path = Path(directory) / 'arm.urdf', Path(directory) / 'camera.urdf'
            arm_path.write_text(arm); camera_path.write_text(camera)
            color = d435i_nominal_tcp_camera(arm_path, [0, 0, .02], camera_path, has_color=True)
            infrared = d435i_nominal_tcp_camera(arm_path, [0, 0, .02], camera_path, has_color=False)
            gemini = gemini2_nominal_tcp_camera(arm_path, [0, 0, .02], camera_path)
        np.testing.assert_allclose(color[:3, 3], [.1, .015, .03])
        np.testing.assert_allclose(infrared[:3, 3], [.1, 0, .03])
        np.testing.assert_allclose(gemini, color)

    def test_shared_level_mount_and_optical_directions(self):
        models = Path(__file__).resolve().parents[1] / 'models/wrist_camera'
        frames = []
        for name in ('gemini2_level.urdf', 'd435i_level.urdf', 'gemini336_level.urdf'):
            root = ET.parse(models / name).getroot()
            mesh = root.find("link[@name='camera_mount_link']/visual/geometry/mesh")
            self.assertTrue(mesh.get('filename').endswith('Gemini2_V1_Level.stl'))
            frames.append(_fixed_chain(root, 'gripper_end', 'camera_color_optical_frame'))
        for frame in frames:
            np.testing.assert_allclose(frame[:3, :3].T @ frame[:3, :3], np.eye(3), atol=1e-8)
        for frame in frames[1:]:
            self.assertGreater(np.dot(frames[0][:3, 2], frame[:3, 2]), .999)
        # The different RGB centers are retained even when the mount is shared.
        self.assertGreater(np.linalg.norm(frames[0][:3, 3] - frames[1][:3, 3]), .01)

    def test_336_camera_rotates_about_cad_pivot(self):
        from scipy.spatial.transform import Rotation
        models = Path(__file__).resolve().parents[1] / 'models/wrist_camera'
        original = ET.parse(models / 'gemini336.urdf').getroot()
        level = ET.parse(models / 'gemini336_level.urdf').getroot()
        before = _fixed_chain(original, 'camera_mount_link', 'camera_color_optical_frame')
        after = _fixed_chain(level, 'camera_mount_link', 'camera_color_optical_frame')
        rotation = Rotation.from_euler('x', -14.999820349448367, degrees=True).as_matrix()
        pivot = np.array([.0002, -.00472, .02352])
        np.testing.assert_allclose(after[:3, 3], pivot + rotation @ (before[:3, 3] - pivot), atol=1e-9)
        np.testing.assert_allclose(after[:3, :3], rotation @ before[:3, :3], atol=1e-9)

    def test_missing_optical_chain_fails(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / 'invalid.urdf'
            path.write_text('<robot><joint name="joint6"><child link="gripper_end"/></joint></robot>')
            with self.assertRaises(ValueError):
                d435i_nominal_tcp_camera(path, [0, 0, 0], path)


if __name__ == '__main__':
    unittest.main()
