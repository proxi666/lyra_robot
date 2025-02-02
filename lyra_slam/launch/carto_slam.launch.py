from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():

    ## ***** Launch arguments *****
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value = 'True')

    ## ***** File paths ******
    # pkg_share = FindPackageShare('cartographer_ros').find('cartographer_ros')
    # urdf_dir = os.path.join(pkg_share, 'urdf')
    # urdf_file = os.path.join(urdf_dir, 'backpack_2d.urdf')
    # with open(urdf_file, 'r') as infp:
    #     robot_desc = infp.read()

    ## ***** Nodes *****
    # robot_state_publisher_node = Node(
    #     package = 'robot_state_publisher',
    #     executable = 'robot_state_publisher',
    #     parameters=[
    #         {'robot_description': robot_desc},
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}],
    #     output = 'screen'
    #     )

    cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_node',
        parameters = [{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        arguments = [
            '-configuration_directory', os.path.join(get_package_share_directory('lyra_slam'), 'config'),
            '-configuration_basename', 'carto_slam.lua'],
        remappings = [
            ('scan', '/scan')],
        output = 'screen'
        )

    cartographer_occupancy_grid_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_occupancy_grid_node',
        parameters = [
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            {'resolution': 0.05},
            # {'occupancy_grid_topic': "/carto/map"},
            ],
        remappings = [
            ('/map', '/carto/map')],
        )

    return LaunchDescription([
        use_sim_time_arg,
        # Nodes
        cartographer_node,
        cartographer_occupancy_grid_node,
    ])