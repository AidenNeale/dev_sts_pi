import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='py_sts_pi',
            executable='aruco',
            namespace='driver_1',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='display',
            namespace='driver_1',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='joystick',
            namespace='driver_1',
        ),
        launch_ros.actions.Node(
            package='joy', 
            executable='joy_node',
            namespace='driver_1',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi',
            executable='aruco',
            namespace='driver_2',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='display',
            namespace='driver_2',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='joystick',
            namespace='driver_2',
        ),
        launch_ros.actions.Node(
            package='joy', 
            executable='joy_node',
            namespace='driver_2',
        ),
    ])