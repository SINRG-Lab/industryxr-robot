import rclpy

# from config.ConfigUtil import ConfigUtil
from sinrg_servo_controller.servo_controller import ServoController

from rclpy.node import Node
from std_msgs.msg import String, Bool

class ServoControllerSubNode(Node):

    def __init__(self):
        # configUtil = ConfigUtil()
        # nodeName = configUtil.getValue("servo", "pub")
        # super().__init__(nodeName)
        super().__init__("servo_subscriber")

        self.servoController = ServoController()

        self.resetArmSub = self.create_subscription(Bool, "/servo/arm/reset", self.resetArmCbk, 10)

    def resetArmCbk(self, msg):
        self.servoController.resetArm()
        self.get_logger().info("Arm position reset successful")   


def main():
    rclpy.init()
    servo_subscriber = ServoControllerSubNode()
    rclpy.spin(servo_subscriber)

if __name__ == "__main__":
    main()