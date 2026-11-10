import copy
import math
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

import PyKDL as kdl

from simulation_validation.kinematics import (
    ArmKinematics, SIDES, dynamic_pose_names, frame_from_matrix, interpolate, make_plan, named_pose,
)
from simulation_validation.model import default_frames_file, joint_positions, load_yaml, origin, robot_description


PACKAGE = Path(__file__).resolve().parents[1]


class KinematicsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.config = load_yaml(default_frames_file())
        # 저장된 v1·v2 목표는 보존된 legacy 장착값에서 생성됐다.
        cls.legacy_config = load_yaml(Path(default_frames_file()).parent / 'legacy/fixed_frames.yaml')
        cls.coordinates = load_yaml(Path(default_frames_file()).with_name('arm_poses.yaml'))
        cls.description = robot_description(cls.config)
        cls.arms = {side: ArmKinematics(cls.description, cls.config['frames'][f'{side}_base'],
                                        cls.config['frames'][f'{side}_flange']) for side in SIDES}
        _, seed = joint_positions(load_yaml(PACKAGE / 'config/init1.yaml'))
        cls.seeds = dict(left=seed[:7], right=seed[7:])

    def assertFrameClose(self, actual, expected):
        error = kdl.diff(actual, expected)
        self.assertLess(error.vel.Norm(), 1e-6)
        self.assertLess(error.rot.Norm(), 1e-6)

    def test_dynamic_requires_both_endpoints_even_with_sequence(self):
        for start, target in (('', ''), ('init1', ''), ('', 'goal'), (' ', 'goal')):
            for sequence in ('', 'init1,goal'):
                with self.subTest(start=start, target=target, sequence=sequence):
                    with self.assertRaisesRegex(ValueError, 'start_pose와 target_pose'):
                        dynamic_pose_names(start, target, sequence)
        self.assertEqual(dynamic_pose_names('init1', 'goal'), ['init1', 'goal'])

    def test_dynamic_sequence_matches_explicit_endpoints(self):
        self.assertEqual(dynamic_pose_names('init1', 'init1', 'init1,goal,init1'),
                         ['init1', 'goal', 'init1'])
        for sequence in ('goal,init1', 'init1,other', 'init1,,goal', 'init1'):
            with self.subTest(sequence=sequence):
                with self.assertRaises(ValueError):
                    dynamic_pose_names('init1', 'goal', sequence)

    def test_invalid_or_unknown_pose_is_rejected(self):
        identity = [[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
        for row, col, value in ((3, 3, 0), (0, 0, -1), (0, 3, math.nan), (0, 0, 2)):
            with self.subTest(value=value):
                matrix = copy.deepcopy(identity)
                matrix[row][col] = value
                with self.assertRaises(ValueError):
                    frame_from_matrix(matrix)
        with self.assertRaises(ValueError):
            frame_from_matrix(identity[:3])
        with self.assertRaisesRegex(ValueError, '등록되지 않은 자세'):
            named_pose(self.coordinates, 'missing')

    def test_kdl_chain_matches_direct_urdf_transform_composition(self):
        # KDL 변환 시 회전축·관절 원점·고정 링크를 올바르게 반영하는지 검사한다.
        for model1300 in (False, True):
            description = robot_description(self.config, model1300, model1300)
            joints = {j.find('child').get('link'): j for j in ET.fromstring(description).findall('joint')}
            for side in SIDES:
                base, tip = (self.config['frames'][f'{side}_{key}'] for key in ('base', 'flange'))
                arm = ArmKinematics(description, base, tip)
                for values in (self.seeds[side], [(lo+hi)/2 for lo,hi in zip(arm.lower, arm.upper)]):
                    positions = dict(zip(arm.names, values))
                    transforms, child = [], tip
                    while child != base:
                        joint = joints[child]
                        element = joint.find('origin')
                        xyz = map(float, element.get('xyz', '0 0 0').split())
                        rpy = map(float, element.get('rpy', '0 0 0').split())
                        frame = kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))
                        if joint.get('type') == 'revolute':
                            axis = kdl.Vector(*map(float, joint.find('axis').get('xyz').split()))
                            frame = frame * kdl.Frame(kdl.Rotation.Rot(axis, positions[joint.get('name')]))
                        transforms.append(frame)
                        child = joint.find('parent').get('link')
                    expected = kdl.Frame()
                    for frame in reversed(transforms):
                        expected = expected * frame
                    self.assertFrameClose(arm.fk(values), expected)

    def test_legacy_saved_path_holds_flange_origins_and_finishes_facing_down(self):
        poses = [named_pose(self.coordinates, name) for name in ('init1', 'init1_floor_parallel_v1')]
        plan = make_plan(self.arms, poses, self.seeds, 100)
        self.assertEqual(len(plan), 101)
        for step, values in enumerate(plan):
            t = step/100
            t = t*t*(3-2*t)
            for index, side in enumerate(SIDES):
                q = values[index*7:index*7+7]
                self.assertTrue(all(lo <= v <= hi for v,lo,hi in zip(q, self.arms[side].lower, self.arms[side].upper)))
                actual = self.arms[side].fk(q)
                self.assertFrameClose(actual, interpolate(poses[0][side], poses[1][side], t))
                self.assertLess((actual.p - poses[0][side].p).Norm(), 1e-6)
                if step == 100:
                    mount = self.legacy_config['fixed_transforms'][f'base_T_{side}_base']
                    world_rotation = kdl.Rotation(*sum(mount['rotation_matrix'], []))
                    normal = world_rotation * actual.M.UnitZ()
                    self.assertLess((normal - kdl.Vector(0, 0, -1)).Norm(), 1e-6)

    def test_legacy_v2_path_matches_heights_preserves_left_xy_and_right_pose(self):
        current = named_pose(self.coordinates, 'init1')
        v1 = named_pose(self.coordinates, 'init1_floor_parallel_v1')
        target = named_pose(self.coordinates, 'init1_floor_parallel_v2')
        mounts = {}
        for side in SIDES:
            xyz, rpy = origin(self.legacy_config['fixed_transforms'][f'base_T_{side}_base'])
            mounts[side] = kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))
        self.assertFrameClose(target['right'], v1['right'])
        self.assertLess(kdl.diff(kdl.Frame(v1['left'].M), kdl.Frame(target['left'].M)).rot.Norm(), 1e-9)
        plan = make_plan(self.arms, [current, target], self.seeds, 100)
        start = {side: mounts[side] * current[side] for side in SIDES}
        for step, values in enumerate(plan):
            for index, side in enumerate(SIDES):
                q = values[index*7:index*7+7]
                actual = self.arms[side].fk(q)
                t = step / 100
                t = t*t*(3 - 2*t)
                self.assertFrameClose(actual, interpolate(current[side], target[side], t))
                world = mounts[side] * actual
                self.assertAlmostEqual(world.p[0], start[side].p[0], delta=1e-6)
                self.assertAlmostEqual(world.p[1], start[side].p[1], delta=1e-6)
                if side == 'right':
                    self.assertAlmostEqual(world.p[2], start[side].p[2], delta=1e-6)
                if step == 100:
                    self.assertAlmostEqual(world.p[2], start['right'].p[2], delta=1e-6)
                    self.assertLess((world.M.UnitZ() - kdl.Vector(0, 0, -1)).Norm(), 1e-6)

    def test_unreachable_goal_and_discontinuous_path_are_rejected(self):
        current = named_pose(self.coordinates, 'init1')
        impossible = kdl.Frame(current['left'].M, kdl.Vector(100, 100, 100))
        with self.assertRaisesRegex(ValueError, 'IK 실패'):
            self.arms['left'].solve(impossible, self.seeds['left'])
        target = named_pose(self.coordinates, 'init1_floor_parallel_v1')
        with self.assertRaisesRegex(ValueError, '인접 IK 결과'):
            make_plan(self.arms, [current, target], self.seeds, 1, max_joint_step=1e-9)

    def test_waypoint_sequence_can_return_to_start(self):
        current = named_pose(self.coordinates, 'init1')
        target = named_pose(self.coordinates, 'init1_floor_parallel_v1')
        plan = make_plan(self.arms, [current, target, current], self.seeds, 20)
        self.assertEqual(len(plan), 41)
        for index, side in enumerate(SIDES):
            self.assertFrameClose(self.arms[side].fk(plan[-1][index*7:index*7+7]), current[side])

    def test_rotation_interpolation_takes_short_arc(self):
        start = kdl.Frame(kdl.Rotation.RotZ(math.radians(179)))
        target = kdl.Frame(kdl.Rotation.RotZ(math.radians(-179)))
        middle = interpolate(start, target, 0.5)
        self.assertAlmostEqual(abs(middle.M.GetRPY()[2]), math.pi)
        self.assertFrameClose(interpolate(start, start, 0.5), start)


if __name__ == '__main__':
    unittest.main()
