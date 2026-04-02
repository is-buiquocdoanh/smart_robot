from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        Node(
            package="collision_detect",
            executable="collision_detect_node",
            output="screen"
        )

    ])