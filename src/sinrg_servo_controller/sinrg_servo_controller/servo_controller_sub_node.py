import rclpy

# from config.ConfigUtil import ConfigUtil
from sinrg_servo_controller.servo_id_enum import SERVO_ENUM
from sinrg_servo_controller.servo_controller import ServoController

from rclpy.node import Node
from std_msgs.msg import String, Bool, Float32



class ServoControllerSubNode(Node):

    def __init__(self):
        # configUtil = ConfigUtil()
        # nodeName = configUtil.getValue("servo", "pub")
        # super().__init__(nodeName)
        super().__init__("servo_subscriber")

        self.servoController = ServoController()

        self.get_logger().info("Initializing Servo Controller...")

        self.resetArmSub = self.create_subscription(Bool, "/arm/servo/reset/all", self.resetArmCbk, 1)
        self.get_logger().info("reset servo subscriber initialized")

        self.setBaseSub = self.create_subscription(Float32, "/arm/servo/base", self.setBaseCbk, 1)
        # self.get_logger().info("servo base topic successfully subscribed")

        self.setLowerArmSub = self.create_subscription(Float32, "/arm/servo/joint/lower", self.setJointLowerCbk, 1)

        self.setMiddleArmSub = self.create_subscription(Float32, "/arm/servo/joint/middle", self.setJointMiddleCbk, 1)

        self.setUpperArmSub = self.create_subscription(Float32, "/arm/servo/joint/upper", self.setJointUpperCbk, 1)

        self.setGripperBaseSub = self.create_subscription(Float32, "/arm/servo/gripper/base", self.setGripperBaseCbk, 1)

        self.setGripperMainSub = self.create_subscription(Float32, "/arm/servo/gripper/main", self.setGripperMainCbk, 1)

        self.get_logger().info("Servo Motors Subscribers initialized")


    def resetArmCbk(self, msg):
        self.servoController.resetArm()
        # self.get_logger().info("Arm position reset successful")   

    def setBaseCbk(self, msg):
        data = msg.data

        posVal = self.servoController.degToPulse(data)
        self.get_logger().info(f"Data is {data} Value is {posVal}")
        self.servoController.setPos(servoID= SERVO_ENUM.BASE_SERVO.value, pos=posVal, servoSpeed=0.5)

    def setJointLowerCbk(self, msg):
        data = msg.data

        posVal = self.servoController.degToPulse(data)
        self.servoController.setPos(servoID= SERVO_ENUM.LOWER_ARM.value, pos=posVal, servoSpeed=0.5)


    def setJointMiddleCbk(self, msg):
        data = msg.data

        posVal = self.servoController.degToPulse(data)
        self.servoController.setPos(servoID= SERVO_ENUM.MIDDLE_ARM.value, pos=posVal, servoSpeed=0.5)

    def setJointUpperCbk(self, msg):
        data = msg.data

        posVal = self.servoController.degToPulse(data)
        self.servoController.setPos(servoID= SERVO_ENUM.UPPER_ARM.value, pos=posVal, servoSpeed=0.5)

    def setGripperBaseCbk(self, msg):
        data = msg.data

        posVal = self.servoController.degToPulse(data)
        self.servoController.setPos(servoID= SERVO_ENUM.GRIPPER_BASE.value, pos=posVal, servoSpeed=0.5)

    def setGripperMainCbk(self, msg):
        data = msg.data

        posVal = self.servoController.degToPulse(data)
        self.servoController.setPos(servoID= SERVO_ENUM.GRIPPER_MAIN.value, pos=posVal, servoSpeed=0.5)
    

    
def main():
    rclpy.init()
    servo_subscriber = ServoControllerSubNode()

    try:
        rclpy.spin(servo_subscriber)
    except KeyboardInterrupt:
        servo_subscriber.get_logger().info("Node interrupted by user keyboard")
    finally:
        servo_subscriber.get_logger().info("Shutting Down ROS Subscriber Node")
        servo_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()