"""책상·블라인드·오른쪽 벽을 world에 고정해 표시한다."""

import math
import xml.etree.ElementTree as ET

import PyKDL as kdl

from .model import add_fixed_joint, vector


def number(value, name, minimum=None, strict=False):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f'{name}: 유한한 수치가 필요합니다.')
    if minimum is not None and (value < minimum or (strict and value == minimum)):
        raise ValueError(f'{name}: {minimum}보다 {"큰" if strict else "크거나 같은"} 값이 필요합니다.')
    return value


def wall_placement(world_table, anchor, floor_z, settings, yaw):
    bottom = world_table * kdl.Vector(*anchor)
    bottom[2] = floor_z
    bottom += world_table.M * kdl.Vector(*vector(settings['offset_m'], 3, '벽 offset_m'))
    rotation = world_table.M * kdl.Rotation.RotZ(yaw) * kdl.Rotation.RPY(
        *vector(settings['rpy_rad'], 3, '벽 rpy_rad'))
    return world_table.Inverse() * kdl.Frame(rotation, bottom)


def placement(environment):
    floor_z = number(environment['floor_z_m'], 'floor_z_m')
    table, blind, right = (environment[key] for key in ('table', 'blind', 'right_wall'))
    size = vector(table['size_m'], 3, 'table.size_m')
    for value in size:
        number(value, 'table.size_m', minimum=0, strict=True)
    rotation = kdl.Rotation.RPY(*vector(table['rpy_rad'], 3, 'table.rpy_rad'))
    # table 링크 원점은 가까운 모서리 윗면 중앙이다.
    world_table = kdl.Frame(rotation, kdl.Vector(*vector(table['position_m'], 3, 'table.position_m')))
    for name, settings, span in (('blind', blind, 'width_m'), ('right_wall', right, 'length_m')):
        number(settings['gap_from_table_m'], f'{name}.gap_from_table_m', minimum=0)
        for key in (span, 'height_m', 'thickness_m'):
            number(settings[key], f'{name}.{key}', minimum=0, strict=True)
    table_blind = wall_placement(world_table, [size[0] + blind['gap_from_table_m'], 0, 0],
                                 floor_z, blind, 0)
    table_right = wall_placement(world_table, [size[0]/2, -size[1]/2 - right['gap_from_table_m'], 0],
                                 floor_z, right, -math.pi/2)
    return world_table, table_blind, table_right


def add_box(root, name, parent, transform, size, visual_offset, settings):
    if type(settings['enabled']) is not bool:
        raise ValueError(f'{name}.enabled는 true 또는 false여야 합니다.')
    rgba = vector(settings['rgba'], 4, f'{name}.rgba')
    if any(value < 0 or value > 1 for value in rgba):
        raise ValueError(f'{name}.rgba는 0~1이어야 합니다.')
    # 숨긴 책상도 두 벽의 기준 링크로 남긴다.
    link = ET.SubElement(root, 'link', name=name)
    add_fixed_joint(root, f'{name}_mount', parent, name, {
        'translation_m': [transform.p[i] for i in range(3)],
        'rpy_rad': list(transform.M.GetRPY()),
    })
    if not settings['enabled']:
        return
    visual = ET.SubElement(link, 'visual')
    ET.SubElement(visual, 'origin', xyz=' '.join(map(str, visual_offset)), rpy='0 0 0')
    geometry = ET.SubElement(visual, 'geometry')
    ET.SubElement(geometry, 'box', size=' '.join(map(str, size)))
    material = ET.SubElement(visual, 'material', name=f'{name}_material')
    ET.SubElement(material, 'color', rgba=' '.join(map(str, rgba)))


def add_environment(root, config, environment):
    if environment['frame_id'] != config['frames']['base']:
        raise ValueError('환경 frame_id는 로봇의 공통 기준 좌표계와 같아야 합니다.')
    world_table, table_blind, table_right = placement(environment)
    table, blind, right = (environment[key] for key in ('table', 'blind', 'right_wall'))
    depth, width, thickness = table['size_m']
    add_box(root, 'environment_table', config['frames']['base'], world_table,
            [depth, width, thickness], [depth / 2, 0, -thickness / 2], table)
    add_box(root, 'environment_blind', 'environment_table', table_blind,
            [blind['thickness_m'], blind['width_m'], blind['height_m']],
            [blind['thickness_m'] / 2, 0, blind['height_m'] / 2], blind)
    add_box(root, 'environment_right_wall', 'environment_table', table_right,
            [right['thickness_m'], right['length_m'], right['height_m']],
            [right['thickness_m'] / 2, 0, right['height_m'] / 2], right)
