from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ldlidar_stl_ros2',
            executable='lidar_listener',
            name='lidar_listener',
            output='screen',
            emulate_tty=True,  # To see logs in a single, continuous stream
        ),
    ])
