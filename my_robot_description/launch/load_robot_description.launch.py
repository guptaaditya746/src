from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Get package path
    package_path = os.path.join(os.getcwd(), 'src', 'my_robot_description')

    return LaunchDescription([
        # Robot State Publisher: Loads the URDF and publishes TFs
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': open(package_path + '/urdf/robot.urdf.xacro').read()}]
        ),

        # Static Transform Publisher for map -> odom (temporary)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            name='map_to_odom'
        )
        

    ])

