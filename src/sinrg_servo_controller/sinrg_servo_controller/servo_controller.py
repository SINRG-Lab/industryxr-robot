
import logging

from sinrg_robot_sdk.robot_board_manager import BoardManager

class ServoController(BoardManager):

    def __init__(self):
        super().__init__()

        self.pwm_min = 0
        self.pwm_max = 1000
        self.deg_min = 0
        self.deg_max = 180

    def setPos(self, servoID:int, pos: int, servoSpeed: int=1):
        """This class sets the position of the servo motor.
            Arguments:
                servoID (int) - ID for the servo
                pos (int) - position in PWM
                servoSpeed (int) - servo speed; DEFAULT = 1
        """

        self.getBoard().bus_servo_set_position(servoSpeed, ((servoID, pos)))
        
    
    def getPos(self, servoID: int):
        if(servoID):
            return self.getBoard().bus_servo_read_position(servo_id= servoID)
        else:
            return -1

    def getVolt(servoID: int):
        pass

    def getServoTemp(self, servoID: int):
        if(servoID):
            return self.getBoard().bus_servo_read_temp(servo_id=servoID)
        else:
            logging.warning("Servo ID not provided")
    
    def setServoOff(servoID: int):
        pass

    def setServoOn(servoID: int):
        pass

    def degToPulse(self, data: float):
        val = ( data / self.deg_max ) * self.pwm_max
        return val

    def pulseToDeg(self, data: int):
        val = (data / self.pwm_max) * self.deg_max
        return val

    def resetServoPos(self, servoID: int):
        if(servoID):
            return self.getBoard().bus_servo_set_position(1, [servoID, 500])
        else:
            return -1

    def resetArm(self):
        self.getBoard().bus_servo_set_position(1, ((10, 500), (5, 500), (4, 500 ), (3, 500), (2, 500), (1, 500)))



        

    