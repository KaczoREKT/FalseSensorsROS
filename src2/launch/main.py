from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='fake_sensors',
            executable='fake_lidar',
            name='fake_lidar',
            parameters=['config/lidar_params.yaml']
        ),
        Node(
            package='fake_sensors',
            executable='fake_camera',
            name='fake_camera',
            parameters=['config/camera_params.yaml']
        ),
    ])
