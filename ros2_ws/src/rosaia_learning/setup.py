from glob import glob

from setuptools import find_packages, setup


package_name = 'rosaia_learning'

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
    description='MLP kinematics training and Jacobian inference.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'audit_sessions = rosaia_learning.audit:main',
            'generate_excitation = rosaia_learning.excitation:main',
            'play_velocity_profile = rosaia_learning.profile_player:main',
            'jacobian_publisher = rosaia_learning.jacobian_node:main',
            'train_mlp = rosaia_learning.train:main',
        ],
    },
)
