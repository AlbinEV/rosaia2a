"""Launch learned Jacobian inference from a configurable YAML file."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create the MLP Jacobian publisher."""
    default = Path(get_package_share_directory('rosaia_learning')) / 'config'
    argument = DeclareLaunchArgument(
        'config', default_value=str(default / 'models.yaml')
    )
    node = Node(
        package='rosaia_learning',
        executable='jacobian_publisher',
        name='jacobian_publisher',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )
    return LaunchDescription([argument, node])
