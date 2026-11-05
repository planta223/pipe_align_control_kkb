import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from simulation_validation.model import default_frames_file, load_yaml, robot_description


def start(context):
    share = Path(get_package_share_directory('simulation_validation'))
    config = load_yaml(LaunchConfiguration('frames_file').perform(context))
    models = []
    for side in ('left', 'right'):
        value = LaunchConfiguration(f'model1300_{side}').perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'model1300_{side}는 true 또는 false여야 합니다.')
        models.append(value == 'true')
    description = robot_description(config, *models)
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    # VS Code Snap의 GTK 경로가 시스템 RViz 라이브러리와 충돌하지 않게 한다.
    rviz_env = {key: '' for key in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR')
                if '/snap/' in os.environ.get(key, '')}
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', namespace='simulation_validation',
             parameters=[{'robot_description': ParameterValue(description, value_type=str)}],
             remappings=tf_remaps, output='screen'),
        Node(package='simulation_validation', executable='init1_joint_states',
             namespace='simulation_validation',
             parameters=[{'pose_file': LaunchConfiguration('pose_file')}], output='screen'),
        Node(package='rviz2', executable='rviz2', namespace='simulation_validation',
             arguments=['-d', str(share / 'rviz/init1.rviz'), '-f', config['frames']['base']],
             remappings=tf_remaps, condition=IfCondition(LaunchConfiguration('rviz')),
             additional_env=rviz_env, output='screen'),
    ]
    if config.get('camera_mount', {}).get('parent_T_camera') is None:
        nodes.insert(0, LogInfo(msg='카메라 장착 변환이 미확인이므로 카메라 표시는 생략합니다.'))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('simulation_validation'))
    return LaunchDescription([
        DeclareLaunchArgument('frames_file', default_value=default_frames_file()),
        DeclareLaunchArgument('pose_file', default_value=str(share / 'config/init1.yaml')),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('model1300_left', default_value='false'),
        DeclareLaunchArgument('model1300_right', default_value='false'),
        OpaqueFunction(function=start),
    ])
