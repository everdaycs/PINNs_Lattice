import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Helper to find our package
    pkg_share = get_package_share_directory('p_lattice_planner')
    
    # Use our config file
    config = os.path.join(pkg_share, 'benchmark_config.yaml')
    
    # Use one of our local maps (ensure it exists in maps/ folder of the package)
    map_file = os.path.join(pkg_share, 'maps', '100by100_20.yaml')
    
    lifecycle_nodes = ['map_server', 'planner_server']

    return LaunchDescription([
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'yaml_filename': map_file},
                        {'topic_name': "map"}]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[config]),

        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            output = 'screen',
            arguments = ["0", "0", "0", "0", "0", "0", "base_link", "map"]),

        Node(
            package = 'tf2_ros',
            executable = 'static_transform_publisher',
            output = 'screen',
            arguments = ["0", "0", "0", "0", "0", "0", "base_link", "odom"]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': lifecycle_nodes}]),
        
        # Disable Rviz to save resources for benchmarking unless needed
        # IncludeLaunchDescription(...)
    ])
