import copy
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import PyKDL

from simulation_validation.model import (
    default_frames_file, joint_positions, load_yaml, origin, robot_description,
)


PACKAGE = Path(__file__).resolve().parents[1]


class ModelTests(unittest.TestCase):
    def setUp(self):
        self.config = load_yaml(default_frames_file())

    def test_base_rotation_survives_rpy_conversion(self):
        for side in ('left', 'right'):
            transform = self.config['fixed_transforms'][f'base_T_{side}_base']
            xyz, rpy = origin(transform)
            rotation = PyKDL.Rotation.RPY(*rpy)
            self.assertEqual(xyz, transform['translation_m'])
            for i in range(3):
                for j in range(3):
                    self.assertAlmostEqual(rotation[i, j], transform['rotation_matrix'][i][j])

    def test_reflection_is_not_a_rigid_rotation(self):
        with self.assertRaises(ValueError):
            origin({'translation_m': [0, 0, 0],
                    'rotation_matrix': [[-1, 0, 0], [0, 1, 0], [0, 0, 1]]})

    def test_unmeasured_transform_is_not_replaced_with_identity(self):
        with self.assertRaises(ValueError):
            origin(None)

    def test_init1_joint_names_and_degree_conversion(self):
        pose = load_yaml(PACKAGE / 'config/init1.yaml')
        names, positions = joint_positions(pose)
        self.assertEqual(names, [f'{side}_joint{i}' for side in ('left', 'right')
                                 for i in range(1, 8)])
        expected = pose['left_joint_angles_deg'] + pose['right_joint_angles_deg']
        for value, degrees in zip(positions, expected):
            self.assertAlmostEqual(math.degrees(value), degrees)
        with self.assertRaises(ValueError):
            joint_positions(dict(pose, left_joint_angles_deg=[0] * 6))

    def test_scene_has_two_arms_tcp_and_no_hardware_plugins(self):
        root = ET.fromstring(robot_description(self.config))
        links = {link.attrib['name'] for link in root.findall('link')}
        self.assertTrue(set(self.config['frames'].values()) - {self.config['frames']['camera']} <= links)
        moving = [j for j in root.findall('joint') if j.attrib['type'] != 'fixed']
        self.assertEqual(len(moving), 14)
        self.assertIsNone(root.find('ros2_control'))
        self.assertIsNone(root.find('gazebo'))
        for side in ('left', 'right'):
            joint = root.find(f"joint[@name='{side}_tcp_offset']")
            self.assertEqual(joint.find('parent').attrib['link'], self.config['frames'][f'{side}_flange'])
            self.assertEqual(joint.find('child').attrib['link'], self.config['frames'][f'{side}_tcp'])

    def test_camera_is_omitted_until_measured(self):
        config = copy.deepcopy(self.config)
        config['camera_mount']['parent_T_camera'] = None
        root = ET.fromstring(robot_description(config))
        self.assertNotIn(config['frames']['camera'], [link.attrib['name'] for link in root.findall('link')])
        config['camera_mount'] = {
            'parent_frame': 'base',
            'parent_T_camera': {'translation_m': [0.1, 0.2, 0.3], 'rpy_rad': [0, 0, 0]},
        }
        root = ET.fromstring(robot_description(config))
        joint = root.find("joint[@name='camera_mount']")
        self.assertEqual(joint.find('parent').attrib['link'], config['frames']['base'])
        self.assertEqual(joint.find('child').attrib['link'], config['frames']['camera'])


if __name__ == '__main__':
    unittest.main()
