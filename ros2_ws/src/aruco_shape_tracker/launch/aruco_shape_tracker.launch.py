from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from pathlib import Path


def generate_launch_description():
    share = Path(get_package_share_directory('aruco_shape_tracker'))
    return LaunchDescription([
        Node(
            package='aruco_shape_tracker',
            executable='aruco_shape_tracker_cpp',
            name='aruco_shape_tracker',
            output='screen',
            parameters=[str(share / 'config' / 'two_arms.yaml')],
        )
    ])
