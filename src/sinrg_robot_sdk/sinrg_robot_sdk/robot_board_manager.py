import logging

from datetime import datetime, timezone

# from config.ConfigUtil import ConfigUtil

from sinrg_robot_sdk.robot_controller_sdk import Board

class BoardManager():

    def __init__(self):

        # self.configUtil = ConfigUtil() 
        # self.uartAddress = self.configUtil.getValue("communication", "UART_ADDRESS")
        self.uartAddress = "/dev/ttyACM0"
        self.board = Board(device= self.uartAddress)
        self._activateBoard()
        
        self.setTimestamp()

    def getBoard(self):
        return self.board

    def _activateBoard(self):
        self.board.enable_reception()
        logging.info("Robot Controller Initialized.")

    def getBoardBattery(self):
        return self.board.get_battery()
    
    def setBoardLed(onTime: int, offTime: int, cycleCount: int):
        pass

    def setTimestamp(self):
        self.timestamp = str(datetime.now(timezone.utc).isoformat())
        return self.timestamp

    def getTimestamp(self):
        return self.timestamp

    

        
        