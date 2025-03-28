import hakopy
import hako_pdu
import os

class GamepadPDU:
    def __init__(self, config_path, robotName, channelId):
        self.robotName = robotName
        self.channelId = channelId
        hako_binary_path = os.getenv('HAKO_BINARY_PATH', '/usr/local/lib/hakoniwa/hako_binary/offset')
        self.pdu_manager = hako_pdu.HakoPduManager(hako_binary_path, config_path)

    def write(self, pdu_data):
        game_pdu = self.pdu_manager.get_pdu(self.robotName, self.channelId)
        game_pdu.obj = pdu_data
        game_pdu.write()

    def read(self):
        game_pdu = self.pdu_manager.get_pdu(self.robotName, self.channelId)
        return game_pdu.read()
