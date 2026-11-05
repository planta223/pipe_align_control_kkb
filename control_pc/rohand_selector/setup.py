from setuptools import setup

setup(
    name='rohand_selector',
    version='0.1.0',
    packages=['rohand_selector'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/rohand_selector']),
        ('share/rohand_selector', ['package.xml']),
        ('share/rohand_selector/config', ['config/selector.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='planta223',
    maintainer_email='planta223@naver.com',
    description='Keyboard hand selection without changing the existing pedal controller.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['rohand_selector = rohand_selector.selector:main']},
)
