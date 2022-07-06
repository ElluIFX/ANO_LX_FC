import threading
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
        self.height_pid = PID(0.1, 0.0, 0.05, setpoint=100, output_limits=(-20, 20))
        self.height_pid.set_auto_mode(False)
        #####################################
        self.keep_height_flag = False
        self.running = False
        self.thread_list = []

    def run(self):
        ############### 参数 #################
        camera_down_pwm = 32.5
        camera_down_45_pwm = 52.25
        camera_up_pwm = 72
        camera_up_45_pwm = 91.75
        set_buzzer = lambda x: self.fc.set_digital_output(0, x)
        ################ 启动线程 ################
        self.running = True
        self.thread_list.append(threading.Thread(target=self.keep_height_task))
        self.thread_list[-1].start()
        logger.info("[MISSION] Threads started")
        ################ 初始化 ################
        fc = self.fc
        radar = self.radar
        fc.set_PWM_output(0, camera_up_pwm)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        ################ 初始化完成 ################

        logger.info("[MISSION] Mission Started")
        fc.unlock()
        sleep(2)
        fc.take_off()
        fc.wait_for_takeoff_done()
        sleep(2)
        ######## 已起飞
        radar.start_find_point(
            timeout=1, type=0, from_=-45, to=45, num=1, range_limit=1500
        )
        fc.set_height(1, 100, 30)
        fc.wait_for_hovering()
        fc.set_flight_mode(fc.HOLD_POS_MODE)
        fc.start_realtime_control()
        self.keep_height_flag = True
        sleep(2)
        ######## 闭环定高

    def stop(self):
        self.running = False

    def keep_height_task(self):
        paused = False
        while self.running:
            sleep(0.1)
            if self.fc.state.mode.value != self.fc.HOLD_POS_MODE:
                self.keep_height_flag = False
            if self.keep_height_flag:
                if paused:
                    paused = False
                    self.height_pid.set_auto_mode(True, last_output=0)
                    logger.info("[MISSION] Keep Height resumed")
                out = self.height_pid(self.fc.state.alt_add.value)
                self.fc.update_realtime_control(vel_z=out)
            else:
                if not paused:
                    paused = True
                    self.height_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_z=0)
                    logger.info("[MISSION] Keep height paused")
