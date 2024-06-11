#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ads_launch'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='simulation_path_planning',
            executable='simulation_path_planning',
            name='simulation_path_planning',
            parameters= [config]
        ),
        Node(
            package='simulation_perception',
            executable='simulation_perception',
            name='simulation_perception',
            parameters= [config]
        ),
        Node(
            package='simulation_control',
            executable='simulation_control',
            name='simulation_control',
            parameters= [config]
        )
    ])