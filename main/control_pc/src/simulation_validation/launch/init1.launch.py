import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from simulation_validation.model import default_frames_file, load_yaml, robot_description
from simulation_validation.kinematics import dynamic_pose_names


MODES = ['static_rviz', 'dynamic_rviz', 'live_feedback', 'live_control']


def start(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in MODES:
        raise ValueError(f'mode는 {", ".join(MODES)} 중 하나여야 합니다.')
    if mode == 'dynamic_rviz':
        dynamic_pose_names(*(LaunchConfiguration(key).perform(context)
                             for key in ('start_pose', 'target_pose', 'pose_sequence')))
    share = Path(get_package_share_directory('simulation_validation'))
    frames_file = LaunchConfiguration('frames_file').perform(context)
    config = load_yaml(frames_file)
    models = []
    for side in ('left', 'right'):
        value = LaunchConfiguration(f'model1300_{side}').perform(context).lower()
        if value not in ('true', 'false'):
            raise ValueError(f'model1300_{side}는 true 또는 false여야 합니다.')
        models.append(value == 'true')
    if mode == 'live_control':
        # 실물 제어기는 공통 설정만 읽으므로 화면도 같은 파일과 모델을 사용한다.
        common = Path(default_frames_file()).parent
        if (Path(frames_file).resolve() != (common / 'fixed_frames.yaml').resolve()
                or Path(LaunchConfiguration('coordinates_file').perform(context)).resolve()
                != (common / 'arm_poses.yaml').resolve()):
            raise ValueError('live_control은 공통 fixed_frames.yaml·arm_poses.yaml만 사용합니다.')
        control = load_yaml(common / 'control.yaml')
        models = [control['robots'][side]['model1300'] for side in ('left', 'right')]
        if any(type(value) is not bool for value in models):
            raise ValueError('control.yaml의 model1300은 bool이어야 합니다.')
    show_environment = LaunchConfiguration('show_environment').perform(context).lower()
    if show_environment not in ('true', 'false'):
        raise ValueError('show_environment는 true 또는 false여야 합니다.')
    environment = None
    if show_environment == 'true':
        environment = load_yaml(LaunchConfiguration('environment_file').perform(context))
    description = robot_description(config, *models, environment=environment)
    tf_remaps = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    # VS Code Snap의 GTK 경로가 시스템 RViz 라이브러리와 충돌하지 않게 한다.
    rviz_env = {key: '' for key in ('GTK_PATH', 'GTK_EXE_PREFIX', 'GIO_MODULE_DIR')
                if '/snap/' in os.environ.get(key, '')}
    if mode == 'live_feedback':
        state_parameters = {f'{side}_ip': LaunchConfiguration(f'{side}_ip') for side in ('left', 'right')}
    elif mode != 'live_control':
        state_parameters = {
            key: ParameterValue(LaunchConfiguration(key), value_type=str)
            for key in ('mode', 'frames_file', 'coordinates_file', 'seed_file', 'pose_name',
                        'start_pose', 'target_pose', 'pose_sequence')}
        state_parameters['robot_description'] = ParameterValue(description, value_type=str)
        state_parameters['duration'] = ParameterValue(LaunchConfiguration('duration'), value_type=float)
    shutdown = [EmitEvent(event=Shutdown(reason='실측 시각화 프로세스가 종료되었습니다.'))]
    nodes = [
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             name='robot_state_publisher', namespace='simulation_validation',
             parameters=[{'robot_description': ParameterValue(description, value_type=str)}],
             remappings=tf_remaps, output='screen',
             on_exit=shutdown if mode == 'live_control' else None),
        Node(package='rviz2', executable='rviz2', namespace='simulation_validation',
             arguments=['-d', str(share / 'rviz/init1.rviz'), '-f', config['frames']['base']],
             remappings=tf_remaps, condition=IfCondition(LaunchConfiguration('rviz')),
             additional_env=rviz_env, output='screen',
             on_exit=shutdown if mode == 'live_control' else None),
    ]
    if mode == 'live_control':
        nodes.insert(0, LogInfo(msg='live_control 시각화입니다. 통합 실행은 ros2 run simulation_validation live_control을 사용하세요.'))
    else:
        nodes.insert(1, Node(package='simulation_validation',
                            executable='live_joint_states' if mode == 'live_feedback' else 'rviz_joint_states',
                            namespace='simulation_validation',
                            parameters=[state_parameters], output='screen',
                            on_exit=[EmitEvent(event=Shutdown(reason='관절각 발행기가 종료되었습니다.'))]))
    if config.get('camera_mount', {}).get('parent_T_camera') is None:
        nodes.insert(0, LogInfo(msg='카메라 장착 변환이 미확인이므로 카메라 표시는 생략합니다.'))
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory('simulation_validation'))
    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', str(
            Path(default_frames_file()).parent.parent / '_log/simulation_validation')),
        DeclareLaunchArgument('mode', default_value='static_rviz', choices=MODES),
        DeclareLaunchArgument('left_ip', default_value='192.168.2.31'),
        DeclareLaunchArgument('right_ip', default_value='192.168.2.32'),
        DeclareLaunchArgument('frames_file', default_value=default_frames_file()),
        DeclareLaunchArgument('coordinates_file', default_value=str(
            Path(default_frames_file()).with_name('arm_poses.yaml'))),
        DeclareLaunchArgument('environment_file', default_value=str(
            share / 'config/environment_models.yaml')),
        DeclareLaunchArgument('show_environment', default_value='true'),
        DeclareLaunchArgument('seed_file', default_value=str(share / 'config/init1.yaml')),
        DeclareLaunchArgument('pose_name', default_value='init1'),
        DeclareLaunchArgument('start_pose', default_value='', description='dynamic_rviz 필수: 시작 자세 이름'),
        DeclareLaunchArgument('target_pose', default_value='', description='dynamic_rviz 필수: 목표 자세 이름'),
        DeclareLaunchArgument('pose_sequence', default_value=''),
        DeclareLaunchArgument('duration', default_value='5.0'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('model1300_left', default_value='false'),
        DeclareLaunchArgument('model1300_right', default_value='false'),
        OpaqueFunction(function=start),
    ])
