from glob import glob

from setuptools import find_packages, setup


package_name = 'rosaia_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=('test',)),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='ROSAIA maintainers',
    maintainer_email='maintainer@example.com',
    description='Live multimodal shape dashboard and goal projection.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'dashboard = rosaia_dashboard.dashboard_node:main',
        ],
    },
)
