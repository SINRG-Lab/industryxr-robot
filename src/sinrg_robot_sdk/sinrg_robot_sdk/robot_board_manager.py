import logging

from datetime import datetime, timezone

# from config.ConfigUtil import ConfigUtil

from sinrg_robot_sdk.robot_controller_sdk import Board

class BoardManager:
    
    _instance = None
    _boardStatus = None

    # def __new__(cls):
    #     if cls._instance is None:
    #         cls._instance = super(BoardManager, cls).__new__(cls)
    #     return cls._instance

    def __init__(self):
        
        # self.isBoardActive = False
        # self.configUtil = ConfigUtil() 
        # self.uartAddress = self.configUtil.getValue("communication", "UART_ADDRESS")
        # if hasattr(self, 'Initialized'):
        if BoardManager._boardStatus is None:
            self.uartAddress = "/dev/ttyACM0"
            # self.setBoard()
            self.board = Board(device= self.uartAddress)
            self._activateBoard()
            BoardManager._boardStatus = "active"
            print("Board is set to Active state")
        else:
            print("Board is already active. using prev instance")
        
    def getBoard(self):
        return self.board
    
    def setBoardStatus(self):
        self._boardStatus = True

    def getBoardStatus(self):
        return self._boardStatus
    
    # def setBoard(self):
    #     self.board = Board(device= self.uartAddress)
    #     self._boardStatus = True

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

    # def __new__(cls):
    #     if cls._instance is None:
    #         cls._instance = super(BoardManager, cls).__new__(cls)
    #     return cls._instance

    

        
        