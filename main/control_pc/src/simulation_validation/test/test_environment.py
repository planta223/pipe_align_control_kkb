import copy
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import PyKDL as kdl

from simulation_validation.environment import placement
from simulation_validation.model import default_frames_file, load_yaml, robot_description


class EnvironmentTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        directory = Path(default_frames_file()).parent
        cls.config = load_yaml(directory / 'fixed_frames.yaml')
        cls.environment = load_yaml(directory / 'environment_models.yaml')

    def scene(self, environment):
        return ET.fromstring(robot_description(self.config, environment=environment))

    def test_world_placement_preserves_table_and_blind_and_adds_right_wall(self):
        table, relative_blind, relative_right = placement(self.environment)
        self.assertLess((table.p - kdl.Vector(0.1233, 0, -0.36786954783562276)).Norm(), 1e-9)
        self.assertLess((table.M.UnitZ() - kdl.Vector(0, 0, 1)).Norm(), 1e-9)
        blind = table * relative_blind
        self.assertAlmostEqual(blind.p[0] - (table.p[0] + 0.76), 0.03)
        self.assertAlmostEqual(blind.p[2], -1.06801)
        self.assertLess(abs(kdl.dot(table.M.UnitZ(), blind.M.UnitX())), 1e-9)
        right = table * relative_right
        self.assertAlmostEqual(right.p[0], table.p[0] + 0.76/2)
        self.assertAlmostEqual((table.p[1] - 1.52/2) - right.p[1], 0.10)
        self.assertAlmostEqual(right.p[2], -1.06801)
        self.assertLess((right.M.UnitX() - kdl.Vector(0, -1, 0)).Norm(), 1e-9)
        self.assertLess(abs(kdl.dot(right.M.UnitX(), blind.M.UnitX())), 1e-9)
        # 벽 길이 양 끝은 책상 가까운 끝·먼 끝과 일치한다.
        for direction, expected_x in ((-1, table.p[0]), (1, table.p[0] + 0.76)):
            edge = right * kdl.Vector(0, direction * 0.76/2, 0)
            self.assertAlmostEqual(edge[0], expected_x)

    def test_geometry_matches_top_surface_and_wall_bottom(self):
        root = self.scene(self.environment)
        table = root.find("link[@name='environment_table']/visual")
        blind = root.find("link[@name='environment_blind']/visual")
        right = root.find("link[@name='environment_right_wall']/visual")
        self.assertEqual(list(map(float, table.find('geometry/box').get('size').split())), [0.76, 1.52, 0.04])
        self.assertEqual(list(map(float, table.find('origin').get('xyz').split())), [0.38, 0, -0.02])
        self.assertEqual(list(map(float, blind.find('geometry/box').get('size').split())), [0.01, 1.52, 2.0])
        self.assertEqual(list(map(float, blind.find('origin').get('xyz').split())), [0.005, 0, 1.0])
        self.assertEqual(list(map(float, right.find('geometry/box').get('size').split())), [0.01, 0.76, 2.0])
        self.assertEqual(list(map(float, right.find('origin').get('xyz').split())), [0.005, 0, 1.0])
        joint = root.find("joint[@name='environment_blind_mount']")
        self.assertEqual(joint.find('parent').get('link'), 'environment_table')
        for link in root.findall('link'):
            if link.get('name').startswith('environment_'):
                self.assertIsNone(link.find('collision'))

    def test_yaml_changes_move_resize_and_rotate_models(self):
        environment = copy.deepcopy(self.environment)
        environment['table'].update(size_m=[1.0, 1.8, 0.06],
                                    position_m=[0.3233, -0.2, -0.397869547836],
                                    rpy_rad=[0, 0, math.pi/2])
        environment['blind'].update(gap_from_table_m=0.05, width_m=1.8, height_m=2.2,
                                    rgba=[0.2, 0.3, 0.4, 0.5])
        environment['right_wall'].update(gap_from_table_m=0.12, length_m=1.0)
        original, _, _ = placement(self.environment)
        table, relative_blind, relative_right = placement(environment)
        blind = table * relative_blind
        self.assertAlmostEqual(table.p[0] - original.p[0], 0.2)
        self.assertAlmostEqual(table.p[1] - original.p[1], -0.2)
        self.assertAlmostEqual(table.p[2] - original.p[2], -0.03)
        self.assertAlmostEqual(blind.p[0], table.p[0])
        self.assertAlmostEqual(blind.p[1] - table.p[1], 1.05)
        self.assertAlmostEqual(blind.p[2], -1.06801)
        right = table * relative_right
        self.assertAlmostEqual(right.p[0] - table.p[0], 1.02)
        self.assertAlmostEqual(right.p[1] - table.p[1], 0.5)
        self.assertAlmostEqual(right.p[2], -1.06801)
        root = self.scene(environment)
        visual = root.find("link[@name='environment_blind']/visual")
        self.assertEqual(list(map(float, visual.find('geometry/box').get('size').split())), [0.01, 1.8, 2.2])
        self.assertEqual(list(map(float, visual.find('material/color').get('rgba').split())), [0.2, 0.3, 0.4, 0.5])

    def test_floor_height_and_wall_offsets_are_configurable(self):
        environment = copy.deepcopy(self.environment)
        environment['floor_z_m'] = -2.0
        environment['right_wall'].update(offset_m=[0.1, -0.2, 0.3], rpy_rad=[0, 0, 0.1])
        table, blind_relative, right_relative = placement(environment)
        right = table * right_relative
        self.assertAlmostEqual((table * blind_relative).p[2], -2.0)
        self.assertAlmostEqual(right.p[2], -1.7)
        self.assertAlmostEqual(right.p[0], table.p[0] + 0.38 + 0.1)
        self.assertAlmostEqual(right.p[1], table.p[1] - 0.76 - 0.10 - 0.2)
        self.assertAlmostEqual(right.M.GetRPY()[2], -math.pi/2 + 0.1)

    def test_hidden_table_still_anchors_visible_blind(self):
        environment = copy.deepcopy(self.environment)
        environment['table']['enabled'] = False
        root = self.scene(environment)
        self.assertIsNone(root.find("link[@name='environment_table']/visual"))
        self.assertIsNotNone(root.find("link[@name='environment_blind']/visual"))
        self.assertIsNotNone(root.find("link[@name='environment_right_wall']/visual"))

    def test_environment_placement_does_not_depend_on_robot_mount_values(self):
        changed = copy.deepcopy(self.config)
        for side in ('left', 'right'):
            changed['fixed_transforms'][f'base_T_{side}_base']['translation_m'] = [10, 20, 30]
        roots = [self.scene(self.environment),
                 ET.fromstring(robot_description(changed, environment=self.environment))]
        joints = [{j.get('name'): ET.tostring(j) for j in root.findall('joint')
                   if j.get('name').startswith('environment_')} for root in roots]
        self.assertEqual(joints[0], joints[1])

    def test_environment_does_not_change_robot_joints(self):
        original = ET.fromstring(robot_description(self.config))
        with_environment = self.scene(self.environment)
        original_joints = {j.get('name'): ET.tostring(j) for j in original.findall('joint')}
        robot_joints = {j.get('name'): ET.tostring(j) for j in with_environment.findall('joint')
                        if not j.get('name').startswith('environment_')}
        self.assertEqual(original_joints, robot_joints)
        self.assertEqual(len([j for j in with_environment.findall('joint') if j.get('type') != 'fixed']), 14)

    def test_invalid_geometry_or_frame_is_rejected(self):
        for section, key, value in [('table', 'size_m', [0, 1, 1]),
                                    ('table', 'position_m', [math.nan, 0, 0]),
                                    ('table', 'enabled', 'false'),
                                    ('blind', 'gap_from_table_m', -0.01),
                                    ('blind', 'height_m', 0),
                                    ('blind', 'rgba', [1.1, 0, 0, 1]),
                                    ('right_wall', 'gap_from_table_m', -0.1),
                                    ('right_wall', 'length_m', 0)]:
            with self.subTest(section=section, key=key):
                environment = copy.deepcopy(self.environment)
                environment[section][key] = value
                with self.assertRaises(ValueError):
                    self.scene(environment)
        for key, value in (('frame_id', 'missing'), ('floor_z_m', math.inf)):
            environment = copy.deepcopy(self.environment)
            environment[key] = value
            with self.assertRaises(ValueError):
                self.scene(environment)


if __name__ == '__main__':
    unittest.main()
