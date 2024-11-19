"""
    Author: carlos
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition

def generate_launch_description():
    """
    launch method
    """

    robot_description_launch_path = PathJoinSubstitution(
        [FindPackageShare('robot_description'), 'launch', 'display.launch.py' ]
    )

    ekf_config_path = PathJoinSubstitution(
        [FindPackageShare("rosbot_navigation"), "config", "ekf.yaml"]
    )



    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_description_launch_path)
        ),
        Node(
            package='rosbot_navigation',
            executable='odometry_robot',
            name='odometry_robot'
        ),
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[
                ekf_config_path
            ],
            # remappings=[("raw_odom", "odom")]
        ),
    ])
