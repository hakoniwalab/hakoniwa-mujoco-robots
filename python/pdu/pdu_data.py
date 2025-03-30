import hakopy
import hako_pdu
import os

class PduData:
    def __init__(self, pdu_manager, robotName, channelId):
        self.robotName = robotName
        self.channelId = channelId
        self.pdu_manager = pdu_manager

    def write(self, pdu_data_obj):
        pdu_data = self.pdu_manager.get_pdu(self.robotName, self.channelId)
        pdu_data.obj = pdu_data_obj
        pdu_data.write()

    def read(self):
        pdu_data = self.pdu_manager.get_pdu(self.robotName, self.channelId)
        return pdu_data.read()
