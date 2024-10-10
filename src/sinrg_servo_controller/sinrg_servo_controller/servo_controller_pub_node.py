import rclpy
import logging
from rclpy.node import Node

from std_msgs.msg import String, Float64, Int64

# from config.ConfigUtil import ConfigUtil

from sinrg_servo_controller.servo_controller import ServoController

class ServoControllerPubNode(Node):

    def __init__(self):
        # configUtil = ConfigUtil()
        # nodeName = configUtil.getValue("ros_node_name", "SERVO_CONTROLLER_PUB")
        # self.pollRate = configUtil.getValue("pollRate", "DEFAULT_POLL_RATE")
        nodeName = "servo_publisher"
        
        self.pollRate = 1

        super().__init__(nodeName)

        self.servoController = ServoController()

        self.tempPub = self.create_publisher(Int64, "/servo/sensor/temp", 10)
        self.get_logger().info("Publishing Servo Temp")
        self.timer = self.create_timer(self.pollRate, self.tempPublisher)       

    def tempPublisher(self):

        tempMsg = Int64()
        tempVal = self.servoController.getServoTemp(10)[0]
        tempMsg.data = tempVal
        self.tempPub.publish(tempMsg)



def main():
    rclpy.init()
    servoPublisher = ServoControllerPubNode()
    rclpy.spin(servoPublisher)

def _shutdown():
    logging.info('Shutting Down ROS Publisher Node ${}')
    rclpy.shutdown()

if __name__ == '__main__':
    main()

        


