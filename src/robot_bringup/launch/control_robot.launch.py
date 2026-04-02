from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # driver
    driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_driver'),
                'launch',
                'robot_driver.launch.py'
            )
        )
    )
    
    # robot joy
    robot_joy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robot_joy'),
                'launch',
                'joystick.launch.py'
            )
        )
    )
    
    # collision detect
    collision_detect_node = Node(
        package='collision_detect',
        executable='collision_detect_node',
        output='screen'
    )

    # battery node
    battery_node = Node(
        package='battery_pkg',
        executable='battery_node',
        output='screen'
    )

    return LaunchDescription([
        driver_launch,
        collision_detect_node,
        battery_node,
        robot_joy
    ])

 # save map
    #ros2 run nav2_map_server map_saver_cli -f map