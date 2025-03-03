from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ld = LaunchDescription()

    urdf_project_path = FindPackageShare('robot_slam')
    # urdf_tutorial_path = FindPackageShare('urdf_tutorial')
    default_model_path = PathJoinSubstitution([urdf_project_path, 'description', 'robot.urdf.xacro'])
    default_rviz_config_path = PathJoinSubstitution([urdf_project_path, 'rviz', 'urdf.rviz'])

    talker_node = Node(
        package='robot_slam',
        executable='talker_lidar',
        name='talker_lidar',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # These parameters are maintained for backwards compatibility
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
            description='Flag to enable joint_state_publisher_gui')
    ld.add_action(gui_arg)
    rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
            description='Absolute path to rviz config file')
    ld.add_action(rviz_arg)

    # This parameter has changed its meaning slightly from previous versions
    ld.add_action(DeclareLaunchArgument(name='model', default_value=str(default_model_path),
            description='Path to robot urdf file relative to robot_slam package'))

    ld.add_action(IncludeLaunchDescription(
        PathJoinSubstitution([FindPackageShare('urdf_launch'), 'launch', 'display.launch.py']),
        launch_arguments={
            'urdf_package': 'robot_slam',
            'urdf_package_path': LaunchConfiguration('model'),
            'rviz_config': LaunchConfiguration('rvizconfig'),
            'jsp_gui': LaunchConfiguration('gui')}.items()
    ))

    ld.add_action(talker_node)

    # cmd per lanciare launch file con rviz:
    # ros2 launch robot_slam talker.launch.py model:=/home/utonto/ros2_ws/src/robot_slam/description/robot.urdf.xacro
    # ros2 run joint_state_publisher_gui joint_state_publisher_gui
    
    return ld