from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('graph_file', default_value='data/lyragraph_graphs/default/graph.json'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='lyragraph_store',
            executable='graph_store_node',
            output='screen',
            parameters=[{
                'graph_file': LaunchConfiguration('graph_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),
        Node(
            package='lyragraph_nav_bridge',
            executable='nav_bridge_node',
            output='screen',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
    ])
