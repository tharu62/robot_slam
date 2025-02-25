from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_slam',
            executable='talker',
            name='talker',
            output='screen',
            parameters=[{'use_sim_time': True}]
        )
    ])