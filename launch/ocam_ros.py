from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ocam',
            executable='ocam',
            name='ocam',
            output='screen',
            parameters=[{
                'resolution': 2,         # 0: 1280x960, 1: 1280x720, 2: 640x480, 3: 640x360
                'frame_rate': 15,
                'exposure': 30,
                'gain': 110,
                'wb_blue': 250,
                'wb_red': 90,
                'auto_exposure': False,
                'show_image': True,
            }]
        ),
    ])
