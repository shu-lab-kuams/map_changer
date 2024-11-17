from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_file_path = os.path.join(
        get_package_share_directory('map_changer'),
        'config',
        'map_changer.yaml',
    )

    return LaunchDescription([
        Node(
            package = 'map_changer',
            executable = 'map_change_node',
            name = 'map_change_node',
            output = 'screen',
            parameters = [config_file_path]
        ),
    ])