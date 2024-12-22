from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_calibration',
            executable='imu_publisher',
            name='imu_publisher',
            output='screen',
        )
    ])
