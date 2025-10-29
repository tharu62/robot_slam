import launch
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='slam_core',
            executable='sub',
            name='sub'),
        launch_ros.actions.Node(
            package='slam_core',
            executable='pub',
            name='pub'),
        launch_ros.actions.Node(
            package='slam_core',
            executable='pub2',
            name='pub2'),    
    ])