from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='my_ik_solver',
            namespace='my_ik_solver_publisher',
            executable='my_inverse_publisher',
            name='my_publisher'
            ),
    ])