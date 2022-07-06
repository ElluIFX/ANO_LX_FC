from time import sleep, time

from FlightController import FC_Client, FC_Controller, logger
from FlightController.Components import LD_Radar, Map_360, Point_2D
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar):
        self.fc = fc
        self.radar = radar

    def run(self):
        pass
