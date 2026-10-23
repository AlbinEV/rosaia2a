from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('rosaia_dashboard'))
    return LaunchDescription([
        DeclareLaunchArgument('arm_1_model', default_value=''),
        DeclareLaunchArgument('arm_2_model', default_value=''),
        DeclareLaunchArgument('enable_goal_publish', default_value='false'),
        DeclareLaunchArgument('enable_motor_execute', default_value='false'),
        Node(
            package='rosaia_dashboard',
            executable='dashboard',
            name='rosaia_dashboard',
            output='screen',
            parameters=[
                str(share / 'config' / 'dashboard.yaml'),
                {
                    'arm_1_model_directory': LaunchConfiguration(
                        'arm_1_model'
                    ),
                    'arm_2_model_directory': LaunchConfiguration(
                        'arm_2_model'
                    ),
                    'enable_goal_publish': ParameterValue(
                        LaunchConfiguration('enable_goal_publish'),
                        value_type=bool,
                    ),
                    'enable_motor_execute': ParameterValue(
                        LaunchConfiguration('enable_motor_execute'),
                        value_type=bool,
                    ),
                },
            ],
        )
    ])
