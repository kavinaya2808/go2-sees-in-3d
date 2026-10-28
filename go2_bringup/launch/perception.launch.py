"""Perception stack for the Go2. Start the simulator first, then this launch file."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    params = PathJoinSubstitution(
        [FindPackageShare('go2_bringup'), 'config', 'perception.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='go2_perception',
            executable='cluster_node',
            name='cluster_node',
            parameters=[params, {'use_sim_time': use_sim_time}],
            output='screen',
        ),
    ])
