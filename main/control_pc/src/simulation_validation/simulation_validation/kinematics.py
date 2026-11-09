"""공식 URDF를 읽어 실물 접속 없이 flange FK·IK를 계산한다."""

import math
import xml.etree.ElementTree as ET

import PyKDL as kdl

from .model import origin, vector


SIDES = ('left', 'right')


def frame_from_matrix(matrix):
    if not isinstance(matrix, list) or len(matrix) != 4:
        raise ValueError('자세 행렬은 4×4여야 합니다.')
    rows = [vector(row, 4, '자세 행렬') for row in matrix]
    if any(abs(a - b) > 1e-9 for a, b in zip(rows[3], [0, 0, 0, 1])):
        raise ValueError('자세 행렬의 마지막 행은 [0, 0, 0, 1]이어야 합니다.')
    xyz, rpy = origin({'translation_m': [rows[i][3] for i in range(3)],
                       'rotation_matrix': [row[:3] for row in rows[:3]]})
    return kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))


def named_pose(config, name):
    poses = config.get('poses', {})
    if name not in poses:
        raise ValueError(f'등록되지 않은 자세: {name}. 사용 가능: {", ".join(poses)}')
    value = poses[name]
    if not isinstance(value, dict) or set(value) != set(SIDES):
        raise ValueError(f'{name}: left와 right 자세가 필요합니다.')
    return {side: frame_from_matrix(value[side]['base_T_flange']) for side in SIDES}


def interpolate(start, target, fraction):
    if not 0 <= fraction <= 1:
        raise ValueError('보간 비율은 0~1이어야 합니다.')
    angle, axis = (start.M.Inverse() * target.M).GetRotAngle()
    rotation = start.M * kdl.Rotation.Rot(axis, angle * fraction)
    return kdl.Frame(rotation, start.p * (1 - fraction) + target.p * fraction)


def joints_array(values):
    result = kdl.JntArray(len(values))
    for i, value in enumerate(values):
        result[i] = value
    return result


class ArmKinematics:
    def __init__(self, description, base, flange):
        root = ET.fromstring(description)
        by_child = {joint.find('child').get('link'): joint for joint in root.findall('joint')}
        path, current = [], flange
        while current != base:
            if current not in by_child or current in [child for child, _ in path]:
                raise ValueError(f'URDF에서 {base} → {flange} 체인을 찾을 수 없습니다.')
            joint = by_child[current]
            path.append((current, joint))
            current = joint.find('parent').get('link')
        self.chain = kdl.Chain()
        self.names, lower, upper = [], [], []
        for child, joint in reversed(path):
            element = joint.find('origin')
            xyz = [float(v) for v in element.get('xyz', '0 0 0').split()] if element is not None else [0]*3
            rpy = [float(v) for v in element.get('rpy', '0 0 0').split()] if element is not None else [0]*3
            frame = kdl.Frame(kdl.Rotation.RPY(*rpy), kdl.Vector(*xyz))
            kind = joint.get('type')
            if kind == 'fixed':
                k_joint = kdl.Joint(joint.get('name'), kdl.Joint.Fixed)
            elif kind == 'revolute':
                axis_element = joint.find('axis')
                axis = [float(v) for v in axis_element.get('xyz').split()] if axis_element is not None else [1, 0, 0]
                k_joint = kdl.Joint(joint.get('name'), frame.p,
                                    frame.M * kdl.Vector(*axis), kdl.Joint.RotAxis)
                limit = joint.find('limit')
                lower.append(float(limit.get('lower')))
                upper.append(float(limit.get('upper')))
                self.names.append(joint.get('name'))
            else:
                raise ValueError(f'지원하지 않는 관절 형식: {kind}')
            self.chain.addSegment(kdl.Segment(child, k_joint, frame))
        if len(self.names) != 7:
            raise ValueError('팔별 회전 관절 7개가 필요합니다.')
        self.lower, self.upper = lower, upper
        self.fk_solver = kdl.ChainFkSolverPos_recursive(self.chain)
        self.velocity_solver = kdl.ChainIkSolverVel_pinv(self.chain, 1e-8)
        self.ik_solver = kdl.ChainIkSolverPos_NR_JL(
            self.chain, joints_array(lower), joints_array(upper),
            self.fk_solver, self.velocity_solver, 500, 1e-7)

    def fk(self, values):
        result = kdl.Frame()
        if self.fk_solver.JntToCart(joints_array(values), result) < 0:
            raise ValueError('FK 계산 실패')
        return result

    def solve(self, target, seed):
        if len(seed) != 7 or any(not math.isfinite(v) for v in seed):
            raise ValueError('IK 초기 관절각 7개가 필요합니다.')
        result = kdl.JntArray(7)
        code = self.ik_solver.CartToJnt(joints_array(seed), target, result)
        values = [result[i] for i in range(7)]
        if code < 0:
            raise ValueError(f'로컬 IK 실패: {self.ik_solver.strError(code)}')
        if any(not math.isfinite(v) or v < lo - 1e-8 or v > hi + 1e-8
               for v, lo, hi in zip(values, self.lower, self.upper)):
            raise ValueError('IK 결과가 관절 제한을 벗어났습니다.')
        residual = kdl.diff(self.fk(values), target)
        if residual.vel.Norm() > 1e-6 or residual.rot.Norm() > 1e-6:
            raise ValueError('IK 결과가 목표 flange 자세와 일치하지 않습니다.')
        return values


def make_plan(arms, poses, seeds, steps, max_joint_step=math.radians(15)):
    """전체 경로를 먼저 검사하고 통과한 관절각만 반환한다."""
    if not poses or steps < 1:
        raise ValueError('하나 이상의 자세와 양수 steps가 필요합니다.')
    current = {side: arms[side].solve(poses[0][side], seeds[side]) for side in SIDES}
    result = [sum((current[side] for side in SIDES), [])]
    for start, target in zip(poses, poses[1:]):
        for step in range(1, steps + 1):
            t = step / steps
            t = t * t * (3 - 2 * t)
            next_values = {}
            for side in SIDES:
                frame = interpolate(start[side], target[side], t)
                values = arms[side].solve(frame, current[side])
                if max(abs(a - b) for a, b in zip(values, current[side])) > max_joint_step:
                    raise ValueError(f'{side}: 인접 IK 결과가 {math.degrees(max_joint_step):g}° 이상 바뀝니다.')
                next_values[side] = values
            current = next_values
            result.append(sum((current[side] for side in SIDES), []))
    return result
