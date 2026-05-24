import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('dvl_a50_serial_python'), 'config', 'dvl_a50_serial_python.yaml')

    dvl_node = Node(
        package='dvl_a50_serial_python',
        executable='dvl_python_node',
        name='dvl_a50_serial',
        namespace='dvl',
        output='screen',
        parameters=[config_file]
    )

    return LaunchDescription([
        dvl_node
    ])
