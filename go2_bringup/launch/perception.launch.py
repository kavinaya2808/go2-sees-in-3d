"""Go2 perception + fusion stack. Start the simulator first, then this launch file.

    ros2 launch go2_bringup perception.launch.py               # clustering + detector + fusion
    ros2 launch go2_bringup perception.launch.py fusion:=false # LiDAR clustering only
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    fusion = LaunchConfiguration('fusion')
    params = PathJoinSubstitution(
        [FindPackageShare('go2_bringup'), 'config', 'perception.yaml'])
    common = [params, {'use_sim_time': use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('fusion', default_value='true',
                              description='Also run the 2D detector and the fusion node'),
        Node(
            package='go2_perception',
            executable='cluster_node',
            name='cluster_node',
            parameters=common,
            output='screen',
        ),
        Node(
            package='go2_detector',
            executable='detector_node',
            name='detector_node',
            parameters=common,
            output='screen',
            condition=IfCondition(fusion),
        ),
        Node(
            package='go2_fusion',
            executable='fusion_node',
            name='fusion_node',
            parameters=common,
            output='screen',
            condition=IfCondition(fusion),
        ),
    ])
