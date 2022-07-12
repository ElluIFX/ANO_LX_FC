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


# 进入点 (A)
start_point = (200, 200)
# 任务点
waypoints = [(100, 100)]
# 回航点
return_points = []
# 基地点
base_point = (100, 300)


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.inital_yaw = self.fc.state.yaw.value
        ############### PID #################
        self.height_pid = PID(
            0.8, 0.0, 0.1, setpoint=80, output_limits=(-15, 15), auto_mode=False
        )
        self.navi_x_pid = PID(
            0.5,
            0.0,
            0.05,
            setpoint=start_point[0],
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.navi_y_pid = PID(
            0.5,
            0.0,
            0.05,
            setpoint=start_point[1],
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.yaw_pid = PID(
            1.0,
            0.0,
            0.05,
            setpoint=self.inital_yaw,
            output_limits=(-45, 45),
            auto_mode=False,
        )
        #####################################
        self.keep_height_flag = False
        self.navigation_flag = False
        self.running = False
        self.thread_list = []
        # vision_debug()

    def run(self):
        ############### 参数 #################
        camera_down_pwm = 32.5
        camera_up_pwm = 72
        navigation_speed = 35
        ################ 启动线程 ################
        self.running = True
        self.thread_list.append(
            threading.Thread(target=self.keep_height_task, daemon=True)
        )
        self.thread_list[-1].start()
        self.thread_list.append(
            threading.Thread(target=self.navigation_task, daemon=True)
        )
        self.thread_list[-1].start()
        logger.info("[MISSION] Threads started")
        ################ 初始化 ################
        fc = self.fc
        radar = self.radar
        cam = self.cam
        change_cam_resolution(cam, 800, 600)
        fc.set_PWM_output(0, camera_up_pwm)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        self.set_navigation_speed(navigation_speed)
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission-1 Started")
        fc.unlock()
        sleep(2)  # 等待电机启动
        fc.take_off(60)
        fc.wait_for_takeoff_done()
        ######## 闭环定高
        fc.set_flight_mode(fc.HOLD_POS_MODE)
        self.keep_height_flag = True
        fc.start_realtime_control()
        self.radar.start_resolve_pose()
        sleep(2)
        ######## 飞进入点
        logger.info("[MISSION] Navigation to start point")
        self.navigation_to_waypoint(start_point)
        self.navigation_flag = True
        fc.set_PWM_output(0, camera_down_pwm)
        self.wait_for_waypoint()
        self.sow()
        ######## 遍历路径
        for n, waypoint in enumerate(waypoints):
            logger.info(f"[MISSION] Navigation to Waypoint-{n:02d}: {waypoint}")
            self.navigation_to_waypoint(waypoint)
            self.wait_for_waypoint()
            self.sow()
        ######## 返航
        for n, return_point in enumerate(return_points):
            logger.info(f"[MISSION] Navigation to Return-{n:02d}: {return_point}")
            self.navigation_to_waypoint(return_point)
            self.wait_for_waypoint()
        ######## 精准着陆
        logger.info("[MISSION] Landing")
        self.height_pid.setpoint = 30
        self.navigation_to_waypoint(base_point)
        self.wait_for_waypoint()
        self.height_pid.setpoint = 20
        sleep(1)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.stop_realtime_control()
        fc.land()
        fc.wait_for_lock()
        logger.info("[MISSION] Mission-1 Finished")

    def sow(self):  # 播撒
        if not self.check_green_ground():
            logger.info("[MISSION] No green ground, skipped")
            sleep(0.5)
            return
        logger.info("[MISSION] Green ground detected")
        for _ in range(2):
            self.fc.set_digital_output(2, 1)
            sleep(0.6)
            self.fc.set_digital_output(2, 0)
            sleep(0.4)
        logger.info("[MISSION] Sow Done")

    def check_green_ground(self):
        LOWER = HSV.GREEN_LOWER
        UPPER = HSV.GREEN_UPPER
        THRESHOLD = 0.4  # 颜色判断阈值
        ROI = (0.45, 0.45, 0.1, 0.1)  # 根据高度调整
        img = self.cam.read()[1]
        img = get_ROI(img, ROI)
        return hsv_checker(img, LOWER, UPPER, THRESHOLD)

    def stop(self):
        self.running = False
        self.fc.stop_realtime_control()

    def keep_height_task(self):
        paused = False
        while self.running:
            sleep(1 / 10)
            if (
                self.keep_height_flag
                and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
            ):
                if paused:
                    paused = False
                    self.height_pid.set_auto_mode(True, last_output=0)
                    self.yaw_pid.set_auto_mode(True, last_output=0)
                    logger.info("[MISSION] Keep Height resumed")
                out_hei = int(self.height_pid(self.fc.state.alt_add.value))
                out_yaw = int(self.yaw_pid(self.fc.state.yaw.value))
                self.fc.update_realtime_control(vel_z=out_hei, yaw=out_yaw)
            else:
                if not paused:
                    paused = True
                    self.height_pid.set_auto_mode(False)
                    self.yaw_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_z=0, yaw=0)
                    logger.info("[MISSION] Keep height paused")

    def navigation_task(self):
        paused = False
        while self.running:
            sleep(1 / 15)
            if (
                self.navigation_flag
                and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
                and self.radar._rtpose_flag
            ):
                if paused:
                    paused = False
                    self.navi_x_pid.set_auto_mode(True, last_output=0)
                    self.navi_y_pid.set_auto_mode(True, last_output=0)
                    logger.info("[MISSION] Navigation resumed")
                current_x = self.radar.rt_pose[0]
                current_y = self.radar.rt_pose[1]
                if current_x > 0:  # 0 为无效值
                    out_x = int(self.navi_x_pid(current_x))
                if current_y > 0:
                    out_y = int(self.navi_y_pid(current_y))
                self.fc.update_realtime_control(vel_x=out_x, vel_y=out_y)
            else:
                if not paused:
                    paused = True
                    self.navi_x_pid.set_auto_mode(False)
                    self.navi_y_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_x=0, vel_y=0)
                    logger.info("[MISSION] Navigation paused")

    def navigation_to_waypoint(self, waypoint):
        self.navi_x_pid.setpoint = waypoint[0]
        self.navi_y_pid.setpoint = waypoint[1]

    def set_navigation_speed(self, speed):
        speed = abs(speed)
        self.navi_x_pid.output_limits = (-speed, speed)
        self.navi_y_pid.output_limits = (-speed, speed)

    def reached_waypoint(self):
        THRESHOLD = 15
        return (
            abs(self.radar.rt_pose[0] - self.navi_x_pid.setpoint) < THRESHOLD
            and abs(self.radar.rt_pose[1] - self.navi_y_pid.setpoint) < THRESHOLD
        )

    def wait_for_waypoint(self):
        while not self.reached_waypoint():
            sleep(0.1)
