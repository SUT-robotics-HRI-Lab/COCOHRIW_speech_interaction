from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config_path = os.path.join(
        get_package_share_directory('workspace_objects'),
        'config',
        'objects.yaml'
    )

    return LaunchDescription([
        Node(
            package='workspace_objects',
            executable='workspace_objects_node',
            name='workspace_objects_node',
            parameters=[{'config_file': config_path}],
            output='screen'
        )
    ])