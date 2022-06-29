from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="adma_ros2_driver",
            executable="adma_ros2_driver",
            name="adma_driver",
            output="screen",
            emulate_tty=True,
            parameters=[
                {"dest_ip": "10.0.0.2"},
                {"dest_port": 1040},
                {"performance_check": False} 
            ]
        )
    ])