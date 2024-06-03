from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ldlidar_stl_ros2',
            executable='ldlidar_stl_ros2_node',
            name='ldlidar_stl_ros2_node',
            output='screen',
            parameters=['/absolute/path/to/ldlidar_config.yaml'],
            arguments=['--ros-args', '--log-level', 'debug']
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_laser']
        )
    ])
