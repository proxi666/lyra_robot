from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lyra_perception',
            executable='rgbd_probe_node',
            output='screen',
        ),
    ])
