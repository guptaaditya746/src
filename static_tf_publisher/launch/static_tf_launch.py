from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Static transform for IMU
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='imu_tf_publisher',
            arguments=['0', '0', '0.85', '0', '0', '0', 'base_link', 'imu_link']
        ),
        # Static transform for Lidar
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_tf_publisher',
            arguments=['0', '0.22', '0.85', '0', '0', '0', 'base_link', 'laser']
        ),
        # Static transform for Odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_tf_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),
    ])
