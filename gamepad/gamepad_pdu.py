import hakopy
import hako_pdu
import os

class GamepadPDU:
    def __init__(self, pdu_manager, robotName, channelId):
        self.robotName = robotName
        self.channelId = channelId
        self.pdu_manager = pdu_manager

    def write(self, pdu_data):
        game_pdu = self.pdu_manager.get_pdu(self.robotName, self.channelId)
        game_pdu.obj = pdu_data
        game_pdu.write()

    def read(self):
        game_pdu = self.pdu_manager.get_pdu(self.robotName, self.channelId)
        return game_pdu.read()
