from time import sleep, time

from FlightController import FC_Client, FC_Controller, logger
from FlightController.Components import LD_Radar, Map_360, Point_2D
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from simple_pid import PID


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar):
        self.fc = fc
        self.radar = radar
        ############### PID #################
        self.height_pid = PID(0.1, 0.0, 0.0, setpoint=100, output_limits=(-20, 20))
        self.height_pid.set_auto_mode(False)

    def run(self):
        ############### 参数 #################
        camera_down_pwm = 32.5
        camera_down_45_pwm = 52.25
        camera_up_pwm = 72
        camera_up_45_pwm = 91.75
        set_buzzer = lambda x: self.fc.set_digital_output(0, x)
        fc = self.fc
        radar = self.radar
        fc.set_PWM_output(0, camera_up_pwm)

        logger.info("[MISSION] Mission Started")
        fc.unlock()
        sleep(2)
        fc.take_off()
        fc.wait_for_takeoff_done()
        sleep(2)
        fc.set_height(1, 100, 30)
        fc.wait_for_hovering()
        radar.start_find_point(
            timeout=1, type=0, from_=-45, to=45, num=1, range_limit=1500
        )
        ######## 已起飞
        sleep(2)
