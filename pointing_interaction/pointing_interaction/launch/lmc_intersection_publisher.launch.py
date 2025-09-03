from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pointing_interaction",
            executable="lmc_intersection_publisher_node",
            name="lmc_intersection_publisher_node",
            output="screen",
            parameters=[
                {"target_frame": "world"},
                {"publish_marker": True},
                {"refresh_rate": 2.0},  # refresh twice per second
            ],
            arguments=[
                "--ros-args", "--log-level", "INFO"
            ],
        ),
    ])
