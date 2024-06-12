#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('ads_launch'),
        'config',
        'realworld_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='realworld_path_planning',
            executable='realworld_path_planning',
            name='realworld_path_planning',
            parameters= [config]
        ),
        Node(
            package='realworld_perception',
            executable='realworld_perception',
            name='realworld_perception',
            parameters= [config]
        ),
        Node(
            package='realworld_control',
            executable='realworld_control',
            name='realworld_control',
            parameters= [config]
        ),
        Node(
            package='realworld_localization',
            executable='realworld_localization',
            name='realworld_localization',
            parameters= [config]
        )
    ])