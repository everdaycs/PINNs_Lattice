import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_nav2_dir = get_package_share_directory('nav2_bringup')
    pkg_saye_bringup = get_package_share_directory('saye_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='True')
    autostart = LaunchConfiguration('autostart', default='True')
    use_rviz = LaunchConfiguration('use_rviz', default='True')

    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_dir, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'map': os.path.join(pkg_saye_bringup, 'maps', 'map.yaml'),
            'params_file': os.path.join(pkg_saye_bringup, 'config', 'nav2_params.yaml'),
        }.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_saye_bringup, 'rviz', 'navigation.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    ld = LaunchDescription()
    
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='True', description='Whether to launch RViz'))
    ld.add_action(nav2_launch_cmd)
    ld.add_action(rviz_node)

    return ld
