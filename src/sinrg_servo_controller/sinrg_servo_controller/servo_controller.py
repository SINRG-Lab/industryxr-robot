
import logging

from sinrg_robot_sdk.sinrg_robot_sdk.robot_board_manager import BoardManager

class ServoController(BoardManager):

    def __init__(self):
        super().__init__()

    def setPos():
        pass
    
    def getPos(self, servoID: int):
        if(servoID):
            return self.getBoard().bus_servo_read_position(servo_id= servoID)

    def getVolt(servoID: int):
        pass

    def getServoTemp(self, servoID: int):
        if(servoID):
            return self.getBoard().bus_servo_read_temp(servo_id=servoID)
        else:
            logging.warning("Servo ID not provided")
    
    def setMotorOff(servoID: int):
        pass

    def setMotorOn(servoID: int):
        pass

    def degToPulse():
        pass

    def pulseToDeg():
        pass

    def resetPos(id: int):
        pass

    def resetArm(self):
        self.getBoard().bus_servo_set_position(1, ((10, 500), (5, 500), (4, 500 ), (3, 500), (2, 500), (1, 500)))



        

    