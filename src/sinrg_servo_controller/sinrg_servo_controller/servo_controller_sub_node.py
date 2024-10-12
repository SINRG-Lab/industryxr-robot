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

        self.resetArmSub = self.create_subscription(Bool, "/arm/servo/reset/all", self.resetArmCbk, 10)
        self.get_logger().info("reset servo topic successfully subscribed")

        self.setBaseSub = self.create_subscription(Float32, "/arm/servo/base", self.setBaseCbk, 10)
        self.get_logger().info("servo base topic successfully subscribed")

        self.setLowerArmSub = self.create_subscription(Float32, "/arm/servo/joint/lower", self.setJointLowerCbk, 10)

        self.setMiddleArmSub = self.create_subscription(Float32, "/arm/servo/joint/middle", self.setJointMiddleCbk, 10)

        self.setUpperArmSub = self.create_subscription(Float32, "/arm/servo/joint/upper", self.setJointUpperCbk, 10)

        self.setGripperBaseSub = self.create_subscription(Float32, "/arm/servo/gripper/base", self.setGripperBaseCbk, 10)

        self.setGripperLeftSub = self.create_subscription(Float32, "/arm/servo/gripper/left", self.setGripperLeftCbk, 10)
        
        self.setGripperRightSub = self.create_subscription(Float32, "/arm/servo/gripper/right", self.setGripperRightCbk, 10)

        self.get_logger().info("Servo Controller initialized")


    def resetArmCbk(self, msg):
        self.servoController.resetArm()
        # self.get_logger().info("Arm position reset successful")   

    def setBaseCbk(self, msg):
        data = msg.data

        posVal = self.servoController.degToPulse(data)
        self.get_logger().info(f"Data is {data} Value is {posVal}")
        self.servoController.setPos(servoID= 1, pos=posVal, servoSpeed=0.5)

        # self.get_logger().info("Base position updated, Value= " + data)

        # self.get_logger().info(str(data))

    def setJointLowerCbk(self, msg):
        
        # self.get_logger().info("Gripper Base Position Updated")
        # self.get_logger().info(str(msg))
        pass

    def setJointMiddleCbk(self, msg):
        self.get_logger().info("Gripper Base Position Updated")
        # self.get_logger().info(str(msg))
        pass

    def setJointUpperCbk(self, msg):
        # self.get_logger().info("Gripper Base Position Updated")
        # self.get_logger().info(str(msg))
        pass

    def setGripperBaseCbk(self, msg):
        # self.get_logger().info("Gripper Base Position Updated")
        pass

    def setGripperLeftCbk(self, msg):
        # self.get_logger().info("Gripper Base Position Updated")
        pass

    def setGripperRightCbk(self, msg):
        # self.get_logger().info("Gripper Base Position Updated")
        pass
    

    
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