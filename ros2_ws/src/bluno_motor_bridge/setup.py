from glob import glob

from setuptools import find_packages, setup


package_name = 'bluno_motor_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Albin Bajrami',
    maintainer_email='AlbinEV@users.noreply.github.com',
    description='Fail-safe ROS 2 serial bridge for two single-actuator ROSAIA arms.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'bluno_motor_bridge_node = bluno_motor_bridge.bridge_node:main',
            'encoder_velocity_controller = '
            'bluno_motor_bridge.velocity_controller_node:main',
        ],
    },
)
