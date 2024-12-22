from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    pkg_dir = os.path.join(os.getcwd(), 'src', 'my_robot_description')
    ekf_config = os.path.join(pkg_dir, 'ekf.yaml')

    return LaunchDescription([
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config]
        )
    ])
