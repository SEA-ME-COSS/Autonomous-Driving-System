import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_dir = get_package_share_directory('simulation_path_planning')

    params_file_path = os.path.join(package_dir, 'config', 'simulation_path_planning_params.yaml')

    simulation_path_planning_node = Node(
        package='simulation_path_planning',
        executable='simulation_path_planning',
        name='simulation_path_planning',
        output='screen',
        parameters=[params_file_path]
    )

    return LaunchDescription([
        simulation_path_planning_node
    ])
