from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ocam_ros2',
            executable='ocam_ros2',
            name='ocam_ros2',
            output='screen',
            parameters=[{
                'resolution': 2,         # 0: 1280x960, 1: 1280x720, 2: 640x480, 3: 640x360
                'frame_rate': 15.0,
                'exposure': 345,
                'gain': 120,
                'wb_blue': 250,
                'wb_red': 150,
                'auto_exposure': False,
                'show_image': True,
            }]
        ),
    ])
