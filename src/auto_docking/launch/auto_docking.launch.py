from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # run the installed console script directly (requires workspace sourced so the script is on PATH)
    docking_manager_node = Node(
        executable='docking_manager',
        name='docking_manager',
        output='screen',
        parameters=[
            {'battery_topic': '/battery_state'},
            {'battery_threshold': 0.20},
            # set these to the known charging pose in your map
            {'charging_pose_frame': 'map'},
            {'charging_pose_x': 0.0},
            {'charging_pose_y': 0.0},
            {'charging_pose_yaw': 0.0},
            {'docking_method': 'line_follow'},
            # topic where the line follower publishes Twist (default node uses /cmd_vel_mag)
            {'line_cmd_topic': '/cmd_vel_mag'},
        ]
    )

    return LaunchDescription([
        docking_manager_node
    ])
