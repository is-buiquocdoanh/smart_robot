from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # usb camera
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_camera',
        parameters=[
            {"video_device": "/dev/video0"},
            {"image_size": [1080, 720]},  # có thể sửa nếu muốn
            {"pixel_format": "YUYV"},
            {"frame_rate": 60},   # có thể sửa nếu muốn
        ]
    )
    
    # convert compressed image to raw image
    convert_compressed_image_node = Node(
        package='image_transport',
        executable='republish',  # <--- Đổi từ 'image_transport' thành 'republish'
        name='convert_compressed_image',
        arguments=[
            'compressed',  # input transport type
            'raw'          # output transport type
        ],
        remappings=[
            ('in/compressed', '/image_raw/compressed'),
            ('out', '/yolo_image_raw')
        ]
    )
    
    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        parameters=[{
            'port': 8080,          # Cổng để truy cập trên trình duyệt
            'address': '0.0.0.0',  # Cho phép tất cả các IP trong mạng truy cập
            'type': 'ros_compressed' # Ưu tiên dùng chuẩn nén để mượt hơn
        }],
        output='screen'
    )

    # yolov8
    yolov8_node = Node(
        package='robot_recognition',
        executable='yolov8_ros2_pt',
        name='yolov8',
        output='screen',
        parameters=[{'use_sim_time': False}]
    )

    # rviz launch
    # rviz_launch = Node (
    #         package= 'rviz2',
    #         executable= 'rviz2',
    #         name= 'rviz2',
    #         output= 'screen',
    #         arguments= ['-d', rviz_config],
    #         parameters= [{'use_sim_time': False}]
    #     )

    return LaunchDescription([
        camera_node,
        convert_compressed_image_node,
        web_video_server_node
        # rviz_launch
    ])
