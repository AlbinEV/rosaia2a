from glob import glob
from setuptools import find_packages, setup


package_name = 'aruco_shape_tracker'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='ROSAIA maintainers',
    maintainer_email='maintainer@example.com',
    description='Base-relative ArUco shape-state extraction.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_shape_tracker = aruco_shape_tracker.tracker_node:main',
        ],
    },
)
