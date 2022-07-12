import threading
from time import sleep, time

import numpy as np
from FlightController import FC_Client, FC_Controller, logger
from FlightController.Components import LD_Radar, Map_360, Point_2D
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from simple_pid import PID


def deg_360_180(deg):
    if deg > 180:
        deg = deg - 360
    return deg


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar):
        self.fc = fc
        self.radar = radar
        ############### PID #################
        self.height_pid = PID(0.8, 0.0, 0.1, setpoint=86, output_limits=(-30, 30))  # 86
        self.height_pid.set_auto_mode(False)
        #####################################
        self.keep_height_flag = False
        self.running = False
        self.thread_list = []
        # vision_debug()

    def run(self):
        return
        ############### 参数 #################
        camera_down_pwm = 32.5
        camera_up_pwm = 72
        ################ 启动线程 ################
        self.running = True
        self.thread_list.append(
            threading.Thread(target=self.keep_height_task, daemon=True)
        )
        self.thread_list[-1].start()
        logger.info("[MISSION] Threads started")
        ################ 初始化 ################
        fc = self.fc
        radar = self.radar
        cam = cv2.VideoCapture(0)
        change_cam_resolution(cam, 800, 600)
        fc.set_PWM_output(0, camera_up_pwm)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission Started")
        fc.unlock()
        sleep(2)
        fc.take_off(60)
        fc.wait_for_takeoff_done()
        ######## 已起飞
        fc.set_flight_mode(fc.HOLD_POS_MODE)
        self.keep_height_flag = True
        fc.start_realtime_control()
        ######## 闭环定高

    def stop(self):
        self.running = False
        self.fc.stop_realtime_control()

    def keep_height_task(self):
        paused = False
        while self.running:
            sleep(0.1)
            if (
                self.keep_height_flag
                and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
            ):
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
