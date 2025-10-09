from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
        
        ld = LaunchDescription()

        talker_lidar = Node(
                package='robot_slam',
                executable='talker_lidar',
                name='talker_lidar',
                output='screen',
                parameters=[{'use_sim_time': True}]
        )

        talker_odom = Node(
                package='robot_slam',
                executable='talker_odom',
                # name='talker_odom',
                output='screen',
                parameters=[{'use_sim_time': True}]   
        )

        odom_tf2 =  Node(
                package='robot_slam',
                executable='odom_tf2',
                name='odom_tf2',
                output='screen',
                parameters=[{'use_sim_time': True}]   
        )

        # lidar_tf2 =  Node(
        #         package='robot_slam',
        #         executable='lidar_tf2',
        #         name='lidar_tf2',
        #         output='screen',
        #         parameters=[{'use_sim_time': True}]   
        # )

        listener_keys =  Node(
                package='robot_slam',
                executable='listener_keys',
                name='listener_keys',
                output='screen',
                parameters=[{'use_sim_time': True}]  
        )
                
        # These parameters are maintained for backwards compatibility
        gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                description='Flag to enable joint_state_publisher_gui')
        ld.add_action(gui_arg)
        #rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=None,
                description='Absolute path to rviz config file')
        ld.add_action(rviz_arg)

        # This parameter has changed its meaning slightly from previous versions
        #ld.add_action(DeclareLaunchArgument(name='model', default_value=str(default_model_path),
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
        
        # ld.add_action(talker_lidar)
        ld.add_action(odom_tf2)
        ld.add_action(talker_odom)
        #ld.add_action(listener_keys)
        
        # cmd per lanciare launch file con rviz manualmente:
        # ros2 launch robot_slam talker.launch.py model:=/home/utonto/ros2_ws/src/robot_slam/description/robot.urdf.xacro rvizconfig:=/mnt/c/Users/yehan/Documenti/robot_slam/rviz/urdf.rviz
        # ros2 run joint_state_publisher_gui joint_state_publisher_gui

        # cmd per lanciare launch file con rviz manualmente (nuovo):
        # ros2 launch robot_slam talker.launch.py model:=/home/deshan/Ros/src/robot_slam/description/robot.urdf.xacro rvizconfig:=/home/deshan/Ros/src/robot_slam/rviz/urdf.rviz
        
        return ld
