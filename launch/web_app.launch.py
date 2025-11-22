from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('web_test')
    
    # Declare arguments
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'params.yaml']),
        description='Path to parameter file'
    )
    
    # Map viewer node with parameters
    map_viewer_node = Node(
        package='web_test',
        executable='map_viewer_node',
        name='map_viewer_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen'
    )
    
    # Web server node
    # web_server_node = Node(
    #     package='web_test',
    #     executable='web_server_node',
    #     name='web_server_node',
    #     output='screen'
    # )
    
    return LaunchDescription([
        params_file_arg,
        map_viewer_node,
        # web_server_node
    ])
