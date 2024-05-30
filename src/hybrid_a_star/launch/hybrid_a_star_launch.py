import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('hybrid_a_star')

    params_file_path = os.path.join(package_dir, 'config', 'hybrid_a_star_params.yaml')

    hybrid_a_star_node = Node(
        package='hybrid_a_star',
        executable='hybrid_a_star',
        name='hybrid_a_star',
        output='screen',
        parameters=[params_file_path]
    )

    return LaunchDescription([
        hybrid_a_star_node
    ])
