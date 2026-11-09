from glob import glob
from setuptools import setup

setup(
    name='simulation_validation',
    version='0.1.0',
    packages=['simulation_validation'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/simulation_validation']),
        ('share/simulation_validation', ['package.xml', 'README.md']),
        ('share/simulation_validation/launch', glob('launch/*.launch.py')),
        ('share/simulation_validation/config', glob('config/*.yaml')),
        ('share/simulation_validation/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='planta223',
    maintainer_email='planta223@naver.com',
    description='Local pose playback and live feedback using the official xArm description.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'rviz_joint_states = simulation_validation.rviz_joint_states:main',
        'live_joint_states = simulation_validation.live_joint_states:main',
    ]},
)
