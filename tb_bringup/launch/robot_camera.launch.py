"""A lancer SUR LE ROBOT (Raspberry Pi) : webcam USB -> /image_raw, /image_raw/compressed, /camera_info.

  ros2 launch tb_bringup robot_camera.launch.py
  ros2 launch tb_bringup robot_camera.launch.py video_device:=/dev/video2
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory('tb_bringup'), 'config', 'usb_cam.yaml')
    video_device = LaunchConfiguration('video_device')

    return LaunchDescription([
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='Peripherique de la webcam (v4l2-ctl --list-devices)'),
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='webcam',
            parameters=[params, {'video_device': video_device}],
            output='screen',
        ),
    ])
