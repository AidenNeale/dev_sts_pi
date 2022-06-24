import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            'node_prefix',
            default_value=[launch.substitutions.EnvironmentVariable('ROBOT_ID'), '_'],
            description='Prefix for node names',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='camera',
            namespace=[launch.substitutions.LaunchConfiguration('node_prefix')],
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='motors',
            namespace=[launch.substitutions.LaunchConfiguration('node_prefix')],
        ),
    ])