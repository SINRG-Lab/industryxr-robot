from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="tcp_endpoint"
    )

    robot_sdk = Node(
        package="sinrg_robot_sdk",
        executable="robot_controller_manager",
        name="robot_controller"
    )

    return LaunchDescription([
        tcp_endpoint,
        robot_sdk
    ])
