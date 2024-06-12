import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('realworld_path_planning')

    params_file_path = os.path.join(package_dir, 'config', 'realworld_path_planning_params.yaml')

    realworld_path_planning_node = Node(
        package='realworld_path_planning',
        executable='realworld_path_planning',
        name='realworld_path_planning',
        output='screen',
        parameters=[params_file_path]
    )

    return LaunchDescription([
        realworld_path_planning_node
    ])
