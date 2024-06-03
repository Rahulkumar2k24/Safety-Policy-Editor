from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    config = get_package_share_directory('ldlidar_stl_ros2') + '/config/collision_monitor_params.yaml'

    return LaunchDescription([
        Node(
            package='ldlidar_stl_ros2',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            parameters=[config]
        )
    ])
