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
                {"destination_ip":          "192.168.88.100"},  # "ADMA Webinterface -> Data -> Ethernet data output -> Destination IP"
                {"destination_port":        1040},              # "ADMA Webinterface -> Data -> Ethernet data output -> Destination Port"
                {"performance_check":       False},
                {"adma_data_topic":         "adma/data"},
                {"adma_navsatfix_topic":    "adma/fix"},
                {"adma_imu_topic":          "adma/imu"},
                {"adma_heading_topic":      "adma/heading"},
                {"adma_velocity_topic":     "adma/velocity"},
                {"gnss_frame":              "gnss_link"},
                {"imu_frame":               "imu_link"},
            ]
        )
    ])