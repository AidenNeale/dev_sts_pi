import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    return launch.LaunchDescription([
        #The following nodes control the human controlled robots
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
        #The following nodes control the autonomous following robots
        launch_ros.actions.Node(
            package='py_sts_pi',
            executable='aruco',
            namespace='follower_1',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='display',
            namespace='follower_1',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='follow_tag',
            namespace='follower_1',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi',
            executable='aruco',
            namespace='follower_2',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='display',
            namespace='follower_2',
        ),
        launch_ros.actions.Node(
            package='py_sts_pi', 
            executable='follow_tag',
            namespace='follower_2',
        ),
    ])