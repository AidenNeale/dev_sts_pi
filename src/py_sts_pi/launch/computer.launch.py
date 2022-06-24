import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        # launch.actions.DeclareLaunchArgument(
        #     'node_prefix',
        #     default_value=[launch.substitutions.EnvironmentVariable('USER'), '_'],
        #     description='Prefix for node names'),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='aruco',
            namespace='test1',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='display',
            namespace='test1',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='joystick',
            namespace='test1',
        ),
        launch_ros.actions.Node(
            package='joy', 
            executable='joy_node',
            namespace='test1',
        ),
            # name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
        # launch_ros.actions.Node(
        #     package='py_sts_pi', 
        #     executable='talker',
            # name=[launch.substitutions.LaunchConfiguration('node_prefix'), 'talker']),
    ])