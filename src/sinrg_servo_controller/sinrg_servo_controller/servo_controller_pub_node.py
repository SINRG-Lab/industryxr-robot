import rclpy
import logging
from rclpy.node import Node

from std_msgs.msg import String, Float32, Int32

from sinrg_servo_controller.servo_id_enum import SERVO_ENUM

# from config.ConfigUtil import ConfigUtil
from sinrg_interfaces.msg import ServoPosition

from sinrg_servo_controller.servo_controller import ServoController

class ServoControllerPubNode(Node):

    def __init__(self):
        # configUtil = ConfigUtil()
        # nodeName = configUtil.getValue("ros_node_name", "SERVO_CONTROLLER_PUB")
        # self.pollRate = configUtil.getValue("pollRate", "DEFAULT_POLL_RATE")
        self.nodeName = "servo_publisher"

        self.baseName = "Base Servo"
        self.lowerArmName = "Lower Arm"
        self.middleArmName = "Middle Arm"
        self.upperArmName = "Upper Arm"
        self.gripperBaseName = "Gripper Base"
        self.gripperMainName = "Gripper Main"
        
        self.pollRate = 0.2
        self.baseTopic = "/unity/robot/servo/base"
        self.lowerArmTopic = "/unity/robot/servo/joint/lower"
        self.middleArmTopic = "/unity/robot/servo/joint/middle"
        self.upperArmTopic = "/unity/robot/servo/joint/upper"
        self.gripperBaseTopic = "/unity/robot/servo/gripper/base"
        self.gripperMainTopic = "/unity/robot/servo/gripper/main"

        super().__init__(self.nodeName)

        self.servoController = ServoController()

        # self.sbus_pub = self.create_publisher(ServoPosition, )

        # self.tempPub = self.create_publisher(Int64, "/servo/sensor/temp", 10)
        # self.get_logger().info("Publishing Servo Temp")
        # self.timer = self.create_timer(self.pollRate, self.tempPublisher) 

        # self.tempPub = self.createPublisher(Int64, "/servo/sensor/temp", self.pollRate, \
        #                                     self.tempPublisher)

        # self.basePub = self.createPublisher(self.baseName, Int32, self.baseTopic, self.pollRate, \
        #                                     self.basePublisher)

        # self.lowerArmPub = self.createPublisher(self.lowerArmName, Int32, self.lowerArmTopic, self.pollRate, \
        #                                     self.lowerArmPublisher)

        # self.middleArmPub = self.createPublisher(self.middleArmName, Int32, self.middleArmTopic, self.pollRate, \
        #                                     self.middleArmPublisher)

        # self.upperArmPub = self.createPublisher(self.upperArmName, Int32, self.upperArmTopic, self.pollRate, \
        #                                     self.upperArmPublisher)

        # self.gripperBasePub = self.createPublisher(self.gripperBaseName, Int32, self.gripperBaseTopic, self.pollRate, \
        #                                     self.gripperBasePublisher) 
        
        # self.gripperMainPub = self.createPublisher(self.gripperMainName, Int32, self.gripperMainTopic, self.pollRate, \
        #                                     self.gripperMainPublisher) 

        
    def basePublisher(self):
        data = self.servoController.getPos(SERVO_ENUM.BASE_SERVO.value)[0]
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos
        print(msg.data)

        self.basePub.publish(msg)

    def lowerArmPublisher(self):
        data = self.servoController.getPos(SERVO_ENUM.LOWER_ARM.value)[0]
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.lowerArmPub.publish(msg)

    def middleArmPublisher(self):
        data = self.servoController.getPos(SERVO_ENUM.MIDDLE_ARM.value)[0]
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.middleArmPub.publish(msg)

    def upperArmPublisher(self):
        data = self.servoController.getPos(SERVO_ENUM.UPPER_ARM.value)[0]
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.upperArmPub.publish(msg)

    def gripperBasePublisher(self):
        data = self.servoController.getPos(SERVO_ENUM.GRIPPER_BASE.value)[0]
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.gripperBasePub.publish(msg)

    def gripperMainPublisher(self):
        data = self.servoController.getPos(SERVO_ENUM.GRIPPER_MAIN.value)[0]
        servoPos = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = servoPos

        self.gripperMainPub.publish(msg)

    # def tempPublisher(self):

    #     tempMsg = Int64()
    #     tempVal = self.servoController.getServoTemp(10)[0]
    #     tempMsg.data = tempVal
    #     self.tempPub.publish(tempMsg)

    def createPublisher(self, publisherName, msgType, msgTopic: String, pollRate: int, clbFunc, queueSize=1):
        dataPub = self.create_publisher(msgType, msgTopic, queueSize)
        self.get_logger().info(f"Publisher - {publisherName} Created")
        self.timer = self.create_timer(pollRate, clbFunc) 

        return dataPub

def main():
    rclpy.init()
    servoPublisher = ServoControllerPubNode()

    try:
        rclpy.spin(servoPublisher)
    except KeyboardInterrupt:
        servoPublisher.get_logger().info("Node interrupted by user keyboard")
    finally:
        servoPublisher.get_logger().info("Shutting Down ROS Publisher Node")
        servoPublisher.destroy_node()
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()

        


