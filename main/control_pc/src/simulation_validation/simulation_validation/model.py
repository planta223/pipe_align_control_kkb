"""공식 모델에 장착 변환과 TCP를 연결한다. 로봇에는 접속하지 않는다."""

import math
from pathlib import Path
import xml.etree.ElementTree as ET

import PyKDL
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory


XACRO = 'http://www.ros.org/wiki/xacro'
ET.register_namespace('xacro', XACRO)


def default_frames_file():
    share = Path(get_package_share_directory('simulation_validation'))
    for parent in share.parents:
        candidate = parent / 'config/coordinate_frames.yaml'
        if candidate.is_file():
            return str(candidate)
    raise FileNotFoundError('coordinate_frames.yaml 위치를 frames_file 인자로 지정하세요.')


def load_yaml(path):
    value = yaml.safe_load(Path(path).read_text())
    if not isinstance(value, dict):
        raise ValueError(f'YAML mapping이 필요합니다: {path}')
    return value


def vector(value, length, name):
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f'{name}: 수치 {length}개가 필요합니다.')
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f'{name}: 유한한 수치가 필요합니다.')
    return value


def origin(transform):
    if not isinstance(transform, dict):
        raise ValueError('미확인 변환을 원점으로 대신할 수 없습니다.')
    xyz = vector(transform.get('translation_m'), 3, 'translation_m')
    if ('rpy_rad' in transform) == ('rotation_matrix' in transform):
        raise ValueError('rpy_rad 또는 rotation_matrix 중 하나를 지정하세요.')
    if 'rpy_rad' in transform:
        return xyz, vector(transform['rpy_rad'], 3, 'rpy_rad')
    rows = transform['rotation_matrix']
    if not isinstance(rows, list) or len(rows) != 3:
        raise ValueError('rotation_matrix는 3×3이어야 합니다.')
    rows = [vector(row, 3, 'rotation_matrix') for row in rows]
    for i in range(3):
        for j in range(3):
            dot = sum(rows[k][i] * rows[k][j] for k in range(3))
            if not math.isclose(dot, float(i == j), abs_tol=1e-6):
                raise ValueError('rotation_matrix가 직교행렬이 아닙니다.')
    rotation = PyKDL.Rotation(*sum(rows, []))
    if not math.isclose(PyKDL.dot(rotation.UnitX(), rotation.UnitY() * rotation.UnitZ()),
                        1.0, abs_tol=1e-6):
        raise ValueError('rotation_matrix의 determinant는 +1이어야 합니다.')
    return xyz, rotation.GetRPY()


def add_fixed_joint(root, name, parent, child, transform):
    xyz, rpy = origin(transform)
    joint = ET.SubElement(root, 'joint', name=name, type='fixed')
    ET.SubElement(joint, 'parent', link=parent)
    ET.SubElement(joint, 'child', link=child)
    ET.SubElement(joint, 'origin', xyz=' '.join(map(str, xyz)), rpy=' '.join(map(str, rpy)))


def joint_positions(pose):
    names, positions = [], []
    for side in ('left', 'right'):
        angles = vector(pose.get(f'{side}_joint_angles_deg'), 7, f'{side} J1~J7')
        names.extend(f'{side}_joint{i}' for i in range(1, 8))
        positions.extend(math.radians(angle) for angle in angles)
    return names, positions


def robot_description(config, model1300_left=False, model1300_right=False):
    frames = config['frames']
    required = {'base', 'camera', 'left_base', 'left_flange', 'left_tcp',
                'right_base', 'right_flange', 'right_tcp'}
    if not required.issubset(frames):
        raise ValueError('8개 기준 좌표계가 필요합니다.')
    if any(not isinstance(frames[k], str) or not frames[k].strip() for k in required):
        raise ValueError('좌표계 이름을 지정하세요.')
    if len({frames[k] for k in required}) != 8:
        raise ValueError('좌표계 이름은 서로 달라야 합니다.')
    fixed = config['fixed_transforms']
    root = ET.Element('robot', name='torso_dual_xarm_init1')
    ET.SubElement(root, f'{{{XACRO}}}include',
                  filename='$(find xarm_description)/urdf/xarm_device_macro.xacro')
    ET.SubElement(root, 'link', name=frames['base'])
    identity = {'translation_m': [0, 0, 0], 'rpy_rad': [0, 0, 0]}
    for side, model1300 in (('left', model1300_left), ('right', model1300_right)):
        base, flange, tcp = (frames[f'{side}_{part}'] for part in ('base', 'flange', 'tcp'))
        for link in (base, flange, tcp):
            ET.SubElement(root, 'link', name=link)
        add_fixed_joint(root, f'{side}_mount', frames['base'], base,
                        fixed[f'base_T_{side}_base'])
        ET.SubElement(root, f'{{{XACRO}}}xarm_device', {
            'prefix': f'{side}_', 'dof': '7', 'robot_type': 'xarm',
            'attach_to': base, 'create_attach_link': 'false',
            'load_gazebo_plugin': 'false', 'model1300': str(model1300).lower(),
        })
        add_fixed_joint(root, f'{side}_flange_alias', f'{side}_link_eef', flange, identity)
        add_fixed_joint(root, f'{side}_tcp_offset', flange, tcp,
                        fixed[f'{side}_flange_T_{side}_tcp'])
    camera = config.get('camera_mount', {})
    if camera.get('parent_T_camera') is not None:
        parent = frames.get(camera.get('parent_frame'), camera.get('parent_frame'))
        if parent not in set(frames.values()) - {frames['camera']}:
            raise ValueError('camera_mount.parent_frame에 알려진 장착 좌표계를 지정하세요.')
        ET.SubElement(root, 'link', name=frames['camera'])
        add_fixed_joint(root, 'camera_mount', parent, frames['camera'], camera['parent_T_camera'])
    doc = xacro.parse(ET.tostring(root, encoding='unicode'))
    xacro.process_doc(doc)
    expanded = ET.fromstring(doc.toxml())
    # 생성된 시각화 URDF에는 기구 모델만 남긴다.
    for element in list(expanded):
        if element.tag in ('ros2_control', 'gazebo', 'transmission'):
            expanded.remove(element)
    links = [link.attrib['name'] for link in expanded.findall('link')]
    if len(links) != len(set(links)):
        raise ValueError('공식 모델과 기준 좌표계의 링크명이 중복됩니다.')
    moving = {joint.attrib['name'] for joint in expanded.findall('joint')
              if joint.attrib['type'] != 'fixed'}
    expected = {f'{side}_joint{i}' for side in ('left', 'right') for i in range(1, 8)}
    if moving != expected:
        raise ValueError('공식 모델의 양팔 J1~J7 구성을 확인하세요.')
    return ET.tostring(expanded, encoding='unicode')
