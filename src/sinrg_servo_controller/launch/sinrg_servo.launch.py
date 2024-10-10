from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    
    servo_controller_pub = Node(
        package= "sinrg_servo_controller",
        executable= "sinrg_servo_publisher",
        name= "sinrg_servo_publisher"
    )

    servo_controller_sub = Node(
        package = "sinrg_servo_controller",
        executable= "sinrg_servo_subscriber",
        name= "sinrg_servo_subscriber"
    )

    return LaunchDescription([
        servo_controller_pub,
        servo_controller_sub
    ])
