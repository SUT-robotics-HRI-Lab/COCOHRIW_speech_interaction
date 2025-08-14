#launch task_executor node

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='task_executor',
            executable='task_executor_node',
            name='task_executor_node',
        )
    ])
