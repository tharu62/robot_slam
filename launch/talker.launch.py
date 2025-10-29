from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
        
        ld = LaunchDescription()
                
        # These parameters are maintained for backwards compatibility
        gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                description='Flag to enable joint_state_publisher_gui')
        ld.add_action(gui_arg)
        rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=None,
                description='Absolute path to rviz config file')
        ld.add_action(rviz_arg)

        # This parameter has changed its meaning slightly from previous versions
        ld.add_action(DeclareLaunchArgument(name='model', default_value=None,
                description='Path to robot urdf file relative to robot_slam package'))

        ld.add_action(IncludeLaunchDescription(
                PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
                launch_arguments={
                'urdf_package': 'robot_slam',
                'urdf_package_path': LaunchConfiguration('model'),
                'rviz_config': LaunchConfiguration('rvizconfig'),
                'jsp_gui': LaunchConfiguration('gui')}.items()
        ))
        
        return ld
