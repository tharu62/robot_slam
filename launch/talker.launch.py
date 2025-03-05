import os
from launch import LaunchDescription
from launch_ros.actions import Node

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# automagically find the path of the robot.urdf.xacro and urdf.rviz files
robot_slam_dir = os.path.dirname(os.path.abspath(__file__)+'/../')  # path to the robot_slam package
model_path_dir = robot_slam_dir + '/description/robot.urdf.xacro'
rviz_config_path_dir = robot_slam_dir + '/rviz/urdf.rviz'


def generate_launch_description():
        ld = LaunchDescription()

        urdf_project_path = FindPackageShare('robot_slam')
        default_model_path = PathJoinSubstitution([urdf_project_path, 'description', 'robot.urdf.xacro'])
        default_rviz_config_path = PathJoinSubstitution([urdf_project_path, 'rviz', 'urdf.rviz'])

        # Talk Lidar Node (avvia il publisher del lidar)
        talker_node = Node(
                package='robot_slam',
                executable='talker_lidar',
                name='talker_lidar',
                output='screen',
                parameters=[{'use_sim_time': True}]
        )

        talker_node_odom = Node(
                package='robot_slam',
                executable='talker_odom',
                name='talker_odom',
                output='screen',
                parameters=[{'use_sim_time': True}]   
        )

        # These parameters are maintained for backwards compatibility
        gui_arg = DeclareLaunchArgument(name='gui', default_value='true', choices=['true', 'false'],
                description='Flag to enable joint_state_publisher_gui')
        ld.add_action(gui_arg)
        #rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=default_rviz_config_path,
        rviz_arg = DeclareLaunchArgument(name='rvizconfig', default_value=rviz_config_path_dir,
                description='Absolute path to rviz config file')
        ld.add_action(rviz_arg)

        # This parameter has changed its meaning slightly from previous versions
        #ld.add_action(DeclareLaunchArgument(name='model', default_value=str(default_model_path),
        ld.add_action(DeclareLaunchArgument(name='model', default_value=model_path_dir,
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
        # ld.add_action(talker_node_odom)
        
        # cmd per lanciare launch file con rviz:
        # ros2 launch robot_slam talker.launch.py model:=/home/utonto/ros2_ws/src/robot_slam/description/robot.urdf.xacro rvizconfig:=/mnt/c/Users/yehan/Documenti/robot_slam/rviz/urdf.rviz
        # ros2 run joint_state_publisher_gui joint_state_publisher_gui
        
        return ld


if __name__ == '__main__':
        print(robot_slam_dir)
        print(model_path_dir)
        print(rviz_config_path_dir)