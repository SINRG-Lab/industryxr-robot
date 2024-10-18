

from time import sleep
from datetime import datetime, timezone

from std_msgs.msg import Int32

from sinrg_robot_sdk.servo_id_enum import SERVO_ENUM
class ServoController:

    def __init__(self, setFunc, getFunc):

        self.setPosition = setFunc
        self.getPosition = getFunc
        
       

        self.pwm_min = 0
        self.pwm_max = 1000
        self.deg_min = 0
        self.deg_max = 180

    def getRawPos(self, servoID: int):
        if(servoID):

            return self.getPosition(servo_id= servoID)[0]
        else:
            return -1

    def getDegPos(self, servoID: int):
        if(servoID):
            return self.getPosition(servo_id= servoID)[0]
        else:
            return -1

    def setPos(self, servoID:int, pos: int, multiplier: int=1, servoSpeed: int=0.5):
        """This class sets the position of the servo motor.
            Arguments:
                servoID (int) - ID for the servo
                pos (int) - position in PWM
                servoSpeed (int) - servo speed; DEFAULT = 1
        """
        currentPos = self.getRawPos(servoID=servoID)
        stepSize = 100

        if(pos > 0):
            # stepDir = 1
            self.setPosition(servoSpeed, [[servoID, currentPos + stepSize]])
        else:
            # stepDir = -1
            self.setPosition(servoSpeed, [[servoID, currentPos - stepSize]])

        # self.setPosition(servoSpeed, [[servoID, stepSize * stepDir]])

        # for pos in range(currentPos, stepSize, stepDir * stepSize):
        #     currentPos = self.getRawPos(servoID=servoID)
        #     self.setPosition(servoSpeed, [[servoID, stepSize]])
        #     sleep(servoSpeed * 0.01)  # Sleep to control the speed of the movement

    def degToPulse(self, data: float):
        val = ( data / self.deg_max ) * self.pwm_max
        return int(val)

    def pulseToDeg(self, data: int):
        val = (data / self.pwm_max) * self.deg_max
        return int(val)

    def resetArm(self):
        self.setPosition(1, ((10, 500), (5, 500), (4, 500 ), (3, 500), (2, 500), (1, 500)))
        sleep(0.02)

        self.setPosition(1, ((10, 500), (5, 500), (4, 500 ), (3, 500), (2, 500), (1, 500)))
        sleep(0.02)
    
    # --------------------Subscribers--------------------------- #
    
    def resetArmCbk(self, msg):
        self.resetArm() 

    def setBaseCbk(self, msg):
        data = msg.data
        self.setPos(servoID= SERVO_ENUM.BASE_SERVO.value, pos=data, servoSpeed=0.5)

    def setJointLowerCbk(self, msg):
        data = msg.data
        self.setPos(servoID= SERVO_ENUM.LOWER_ARM.value, pos=data, servoSpeed=0.5)

    def setJointMiddleCbk(self, msg):
        data = msg.data
        self.setPos(servoID= SERVO_ENUM.MIDDLE_ARM.value, pos=data, servoSpeed=0.5)

    def setJointUpperCbk(self, msg):
        data = msg.data
        self.setPos(servoID= SERVO_ENUM.UPPER_ARM.value, pos=data, servoSpeed=0.5)

    def setGripperBaseCbk(self, msg):
        data = msg.data
        self.setPos(servoID= SERVO_ENUM.GRIPPER_BASE.value, pos=data, servoSpeed=0.5)

    def setGripperMainCbk(self, msg):
        data = msg.data
        self.setPos(servoID= SERVO_ENUM.GRIPPER_MAIN.value, pos=data, servoSpeed=0.5)


    # # --------------------Publishers--------------------------- #

    # def basePublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.BASE_SERVO.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos
    #     # print(msg.data)

    
    #     self.basePub.publish(msg)

    # def lowerArmPublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.LOWER_ARM.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos

    
    #     self.lowerArmPub.publish(msg)

    # def middleArmPublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.MIDDLE_ARM.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos

    
    #     self.middleArmPub.publish(msg)

    # def upperArmPublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.UPPER_ARM.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos

    
    #     self.upperArmPub.publish(msg)

    # def gripperBasePublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.GRIPPER_BASE.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos
        
        

    #     self.gripperBasePub.publish(msg)

    # def gripperMainPublisher(self):
    #     data = self.getDegPos(SERVO_ENUM.GRIPPER_MAIN.value)[0]
    #     servoPos = self.pulseToDeg(data)
    #     msg = Int32()
    #     msg.data = servoPos


    #     self.gripperMainPub.publish(msg)