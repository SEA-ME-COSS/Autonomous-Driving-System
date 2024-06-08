#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('launch_files'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='hybrid_a_star',
            executable='hybrid_a_star',
            name='hybrid_a_star',
            parameters= [config]
        ),
        Node(
            package='object_detection',
            executable='object_detection',
            name='object_detection',
            parameters= [config]
        ),
        Node(
            package='stanley',
            executable='stanley',
            name='stanley',
            parameters= [config]
        )
    ])