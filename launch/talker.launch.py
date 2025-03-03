from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    talker_node = Node(
        package='robot_slam',
        executable='talker_lidar',
        name='talker_lidar',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    ld.add_action(talker_node)
    
    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'description',
            'urdf_package_path': PathJoinSubstitution(['urdf', 'robot.urdf'])}.items()
    ))
    
    return ld