import os
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    map_file = '/home/jalanth/Documents/MAR_miniproject/disaster_map.yaml'

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'yaml_filename': map_file,
            'use_sim_time': True,
        }]
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'autostart': True,
            'node_names': ['map_server'],
        }]
    )

    return LaunchDescription([
        map_server,
        lifecycle_manager,
    ])
