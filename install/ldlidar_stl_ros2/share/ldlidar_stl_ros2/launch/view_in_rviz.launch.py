from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ldlidar_stl_ros2_share = get_package_share_directory('ldlidar_stl_ros2')

    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', ldlidar_stl_ros2_share + '/rviz2/collision_monitor.rviz']
        )
    ])
