from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('run_id', default_value='default'),
        DeclareLaunchArgument('output_root', default_value='data/lyragraph_runs'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='lyragraph_keyframe_logger',
            executable='keyframe_logger_node',
            output='screen',
            parameters=[{
                'run_id': LaunchConfiguration('run_id'),
                'output_root': LaunchConfiguration('output_root'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
    ])
