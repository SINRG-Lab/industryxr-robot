import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32, Int32

from sinrg_robot_sdk.servo_id_enum import SERVO_ENUM

from datetime import datetime, timezone

from sinrg_robot_sdk.robot_controller_sdk import Board

from sinrg_robot_sdk.servo_controller import ServoController
# from sinrg_robot_sdk.servo_controller_sub_node import ServoControllerSubNode

class RobotController(Node):

    def __init__(self):
        super().__init__("robot_controller_node")

        self.board = None
        self.isBoardActive = None
        
        self.uartAddress = "/dev/ttyACM0"
        self._initializeBoard()
        
        self.get_logger().info("Initializing Servo Controller...")
        self.servoController = ServoController(setFunc = self.board.bus_servo_set_position, \
                                               getFunc= self.board.bus_servo_read_position)

        # Subscribers
        self.get_logger().info("Initializing Servo Subscribers...")
        self.resetArmSub = self.create_subscription(Bool, "/arm/servo/reset/all", self.servoController.resetArmCbk, 1)
        self.setBaseSub = self.create_subscription(Float32, "/arm/servo/base", self.servoController.setBaseCbk, 1)
        self.setLowerArmSub = self.create_subscription(Float32, "/arm/servo/joint/lower", self.servoController.setJointLowerCbk, 1)
        self.setMiddleArmSub = self.create_subscription(Float32, "/arm/servo/joint/middle", self.servoController.setJointMiddleCbk, 1)
        self.setUpperArmSub = self.create_subscription(Float32, "/arm/servo/joint/upper", self.servoController.setJointUpperCbk, 1)
        self.setGripperBaseSub = self.create_subscription(Float32, "/arm/servo/gripper/base", self.servoController.setGripperBaseCbk, 1)
        self.setGripperMainSub = self.create_subscription(Float32, "/arm/servo/gripper/main", self.servoController.setGripperMainCbk, 1)
        self.get_logger().info("Servo Subscribers initialized.")

        # Publishers
        self.get_logger().info("Initializing Servo Publishers.")
        self.baseName = "Base Servo"
        self.lowerArmName = "Lower Arm"
        self.middleArmName = "Middle Arm"
        self.upperArmName = "Upper Arm"
        self.gripperBaseName = "Gripper Base"
        self.gripperMainName = "Gripper Main"
        
        self.pollRate = 0.5

        self.baseTopic = "/unity/robot/servo/base"
        self.lowerArmTopic = "/unity/robot/servo/joint/lower"
        self.middleArmTopic = "/unity/robot/servo/joint/middle"
        self.upperArmTopic = "/unity/robot/servo/joint/upper"
        self.gripperBaseTopic = "/unity/robot/servo/gripper/base"
        self.gripperMainTopic = "/unity/robot/servo/gripper/main"

        self.basePub = self.createPublisher(self.baseName, Int32, self.baseTopic, self.pollRate, \
                                            self.basePublisher)
        self.lowerArmPub = self.createPublisher(self.lowerArmName, Int32, self.lowerArmTopic, self.pollRate, \
                                            self.lowerArmPublisher)
        self.middleArmPub = self.createPublisher(self.middleArmName, Int32, self.middleArmTopic, self.pollRate, \
                                            self.middleArmPublisher)
        self.upperArmPub = self.createPublisher(self.upperArmName, Int32, self.upperArmTopic, self.pollRate, \
                                            self.upperArmPublisher)
        self.gripperBasePub = self.createPublisher(self.gripperBaseName, Int32, self.gripperBaseTopic, self.pollRate, \
                                            self.gripperBasePublisher) 
        self.gripperMainPub = self.createPublisher(self.gripperMainName, Int32, self.gripperMainTopic, self.pollRate, \
                                            self.gripperMainPublisher) 
        self.get_logger().info("Servo Publishers initialized.")

        self.get_logger().info("Servo Controller initialized.")

    
    # --------------------Publishers--------------------------- #
        
    def basePublisher(self):
        
        data = self.servoController.getRawPos(SERVO_ENUM.BASE_SERVO.value)
        deg = self.servoController.pulseToDeg(data)

        # print(f"Value is {data} and degree is {deg}")
        
        msg = Int32()
        msg.data = deg
        # print(msg.data)

        self.basePub.publish(msg)

    def lowerArmPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.LOWER_ARM.value)
        deg = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = deg

        
        # print(f"Lower Arm value is {data} and degree is {deg}")
    
        self.lowerArmPub.publish(msg)

    def middleArmPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.MIDDLE_ARM.value)
        deg = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = deg
        # print(f"Middle Arm value is {data} and degree is {deg}")
    
        self.middleArmPub.publish(msg)

    def upperArmPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.UPPER_ARM.value)
        deg = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = deg

    
        self.upperArmPub.publish(msg)

    def gripperBasePublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.GRIPPER_BASE.value)
        deg = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = deg
        

        self.gripperBasePub.publish(msg)

    def gripperMainPublisher(self):
        data = self.servoController.getRawPos(SERVO_ENUM.GRIPPER_MAIN.value)
        deg = self.servoController.pulseToDeg(data)
        msg = Int32()
        msg.data = deg

        self.gripperMainPub.publish(msg)

    def createPublisher(self, publisherName, msgType, msgTopic, pollRate: int, clbFunc, queueSize=1):
        dataPub = self.create_publisher(msgType, msgTopic, queueSize)
        self.get_logger().info(f"Publisher - {publisherName} Created")
        self.timer = self.create_timer(pollRate, clbFunc) 

        return dataPub

    def _initializeBoard(self):
        try:
            self.board = Board(device=self.uartAddress)  # Initialize the board
            self.board.enable_reception()
            
            self.get_logger().info("Board initialized successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize board: {e}")
            
    def getBoard(self):
        if self._board is not None:
            return self._board
        else: 
            return -1

def main():
    rclpy.init()
    robot_controller = RobotController()

    try:
        rclpy.spin(robot_controller)
    except KeyboardInterrupt:
        robot_controller.get_logger().info("Node interrupted by user keyboard")
    finally:
        robot_controller.get_logger().info("Shutting Down ROS Subscriber Node")
        robot_controller.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
    