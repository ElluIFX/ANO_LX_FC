import threading
from re import S
from time import sleep, time

import cv2
import numpy as np
from FlightController import FC_Client, FC_Controller, logger
from FlightController.Components import LD_Radar, Map_360, Point_2D
from FlightController.Solutions.Vision import (
    change_cam_resolution,
    find_QRcode_zbar,
    set_cam_autowb,
    vision_debug,
)
from simple_pid import PID


def deg_360_180(deg):
    if deg > 180:
        deg = deg - 360
    return deg

TARGET_POINT1 = np.array([315, 419])# 任务点
BASE_POINT = np.array([79, 425])# 基地点

class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        ############### PID #################
        self.height_pid = PID(
            0.8, 0.0, 0.1, setpoint=0, output_limits=(-30, 30), auto_mode=False
        )
        self.navi_x_pid = PID(
            0.4,
            0,
            0.08,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.navi_y_pid = PID(
            0.4,
            0,
            0.08,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.navi_yaw_pid = PID(
            0.3,
            0.02,
            0.1,
            setpoint=0,
            output_limits=(-45, 45),
            auto_mode=False,
        )
        #####################################
        self.keep_height_flag = False
        self.navigation_flag = False
        self.avoidance_flag = False
        self.find_obstacle_flag = False
        self.running = False
        self.thread_list = []
        # vision_debug()

    def stop(self):
        self.running = False
        self.fc.stop_realtime_control()

    def run(self):
        fc = self.fc
        radar = self.radar
        cam = self.cam
        ############### 参数 #################
        self.camera_down_pwm = 32.5
        self.camera_up_pwm = 72
        self.navigation_speed = 25  # 导航速度
        self.cruise_height = 125  # 巡航高度
        self.set_buzzer = lambda x: fc.set_digital_output(0, x)
        self.pid_tunings = {
            "default": (0.4, 0, 0.08),  # 导航
            "landing": (0.3, 0.02, 0.12),  # 降落
            "circle": (0.5, 0, 0.08),  # 钻圈
        }  # PID参数
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
        self.thread_list.append(
            threading.Thread(target=self.avoidance_task, daemon=True)
        )
        self.thread_list[-1].start()
        logger.info("[MISSION] Threads started")
        ################ 初始化 ################
        self.init_avoidance_args(0)
        fc.set_action_log(False)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        self.set_navigation_speed(self.navigation_speed)
        fc.set_rgb_led(255, 0, 0)  # 起飞前警告
        for i in range(10):
            sleep(0.1)
            self.set_buzzer(True)
            sleep(0.1)
            self.set_buzzer(False)
        fc.set_rgb_led(0, 0, 0)
        fc.set_PWM_output(0, self.camera_down_pwm)
        fc.set_action_log(True)
        self.fc.start_realtime_control(20)
        self.switch_pid("default")
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission-1 Started")
        self.pointing_takeoff(BASE_POINT)
        ################ 前往目标点 ################
        self.navigation_to_waypoint(TARGET_POINT1)
        self.avoidance_flag = True
        time_count = 0
        while self.avoidance_flag: # 等待到达同时开启避障
            sleep(0.1)
            if self.reached_waypoint():
                time_count += 0.1
            if time_count >= 1:
                self.avoidance_flag = False
                logger.info("[MISSION] Reached waypoint")
                break
            if self.find_obstacle_flag:
                self.avoidance_obstacle()
                break
        ################ 到达目标点 ################
        self.pointing_landing(TARGET_POINT1)
        logger.info("[MISSION] Misson-2 Finished")


    def pointing_takeoff(self, point):
        """
        定点起飞
        """
        logger.info(f"[MISSION] Takeoff at {point}")
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        self.fc.unlock()
        sleep(2)  # 等待电机启动
        self.fc.take_off(80)
        self.fc.wait_for_takeoff_done()
        ######## 闭环定高
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        self.height_pid.setpoint = self.cruise_height
        self.keep_height_flag = True
        sleep(2)
        self.navigation_to_waypoint(point)  # 初始化路径点
        self.switch_pid("default")
        sleep(0.1)
        self.navigation_flag = True

    def pointing_landing(self, point):
        """
        定点降落
        """
        logger.info(f"[MISSION] Landing at {point}")
        self.navigation_to_waypoint(point)
        self.wait_for_waypoint()
        self.switch_pid("landing")
        sleep(1)
        self.height_pid.setpoint = 60
        sleep(1.5)
        self.wait_for_waypoint()
        self.height_pid.setpoint = 20
        sleep(2)
        self.wait_for_waypoint()
        self.height_pid.setpoint = 0
        self.fc.wait_for_lock(6)
        self.fc.lock()

    def switch_pid(self, pid):
        """
        切换PID参数
        """
        tuning = self.pid_tunings.get(pid, self.pid_tunings["default"])
        self.navi_x_pid.tunings = tuning
        self.navi_y_pid.tunings = tuning
        logger.info(f"[MISSION] PID Tunings set to {pid}: {tuning}")

    def keep_height_task(self):
        paused = False
        while self.running:
            sleep(1 / 10)  # 飞控参数以20Hz回传
            if (
                self.keep_height_flag
                and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
            ):
                if paused:
                    paused = False
                    self.height_pid.set_auto_mode(True, last_output=0)
                    logger.info("[MISSION] Keep Height resumed")
                out_hei = int(self.height_pid(self.fc.state.alt_add.value))
                self.fc.update_realtime_control(vel_z=out_hei)
            else:
                if not paused:
                    paused = True
                    self.height_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_z=0)
                    logger.info("[MISSION] Keep height paused")

    def navigation_task(self):
        ######## 解算参数 ########
        SIZE = 1000
        SCALE_RATIO = 0.5
        LOW_PASS_RATIO = 0.6
        ########################
        paused = False
        while self.running:
            sleep(0.01)
            if (
                self.navigation_flag
                and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
            ):
                if paused:
                    paused = False
                    self.navi_x_pid.set_auto_mode(True, last_output=0)
                    self.navi_y_pid.set_auto_mode(True, last_output=0)
                    self.navi_yaw_pid.set_auto_mode(True, last_output=0)
                    logger.info("[MISSION] Navigation resumed")
                if not self.radar._rtpose_flag:
                    self.radar.start_resolve_pose(
                        size=SIZE,
                        scale_ratio=SCALE_RATIO,
                        low_pass_ratio=LOW_PASS_RATIO,
                    )
                    logger.info("[MISSION] Resolve pose started")
                    sleep(0.01)
                if self.radar.rt_pose_update_event.wait(1):  # 等待地图更新
                    self.radar.rt_pose_update_event.clear()
                    current_x = self.radar.rt_pose[0]
                    current_y = self.radar.rt_pose[1]
                    if current_x > 0 and current_y > 0:
                        self.fc.send_general_position(x=current_x, y=current_y)
                    current_yaw = self.radar.rt_pose[2]
                    out_x = None
                    out_y = None
                    out_yaw = None
                    if current_x > 0:  # 0 为无效值
                        out_x = int(self.navi_x_pid(current_x))
                        if out_x is not None:
                            self.fc.update_realtime_control(vel_x=out_x)
                    if current_y > 0:
                        out_y = int(self.navi_y_pid(current_y))
                        if out_y is not None:
                            self.fc.update_realtime_control(vel_y=out_y)
                    out_yaw = int(self.navi_yaw_pid(current_yaw))
                    if out_yaw is not None:
                        self.fc.update_realtime_control(yaw=out_yaw)
                    if False:  # debug
                        logger.debug(
                            f"[MISSION] Current pose: {current_x}, {current_y}, {current_yaw}; Output: {out_x}, {out_y}, {out_yaw}"
                        )
            else:
                self.radar.stop_resolve_pose()
                if not paused:
                    paused = True
                    self.navi_x_pid.set_auto_mode(False)
                    self.navi_y_pid.set_auto_mode(False)
                    self.navi_yaw_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                    logger.info("[MISSION] Navigation paused")

    def navigation_to_waypoint(self, waypoint):
        self.navi_x_pid.setpoint = waypoint[0]
        self.navi_y_pid.setpoint = waypoint[1]

    def set_navigation_speed(self, speed):
        speed = abs(speed)
        self.navi_x_pid.output_limits = (-speed, speed)
        self.navi_y_pid.output_limits = (-speed, speed)

    def reached_waypoint(self, pos_thres=15):
        return (
            abs(self.radar.rt_pose[0] - self.navi_x_pid.setpoint) < pos_thres
            and abs(self.radar.rt_pose[1] - self.navi_y_pid.setpoint) < pos_thres
        )

    def wait_for_waypoint(self, time_thres=1, pos_thres=15, timeout=30):
        time_count = 0
        time_start = time()
        while True:
            sleep(0.1)
            if self.reached_waypoint(pos_thres):
                time_count += 0.1
            if time_count >= time_thres:
                logger.info("[MISSION] Reached waypoint")
                return
            if time() - time_start > timeout:
                logger.warning("[MISSION] Waypoint overtime")
                return

    def avoidance_task(self):
        """
        实时避障DEMO
        在检测到障碍物后升高巡航速度
        移动指定距离后降低巡航高度
        """
        paused = False
        while self.running:
            sleep(0.1)
            if self.avoidance_flag:
                if paused:
                    paused = False
                    self.radar.start_find_point(
                        2.5, 0, self.fp_from, self.fp_to, 1, 800
                    )
                if self.radar.fp_points[0].distance < 400:
                    self.avoidance_flag = False
                    self.find_obstacle_flag = True
                    logger.info("[MISSION] find obstacle")

            else:
                self.radar.stop_find_point()
                if not paused:
                    paused = True
                    logger.info("[MISSION] Avoidance paused")

    def avoidance_obstacle(self):
        waypoint_x = self.navi_x_pid.setpoint
        waypoint_y = self.navi_y_pid.setpoint
        self.navigation_flag = False
        self.fc.update_realtime_control(0, 0, yaw=0)
        current_x = self.radar.rt_pose[0]
        current_y = self.radar.rt_pose[1]
        self.height_pid.setpoint = self.avoidance_height
        for i in range(3):
            self.set_buzzer(True)
            self.fc.set_rgb_led(255, 0, 0)
            sleep(0.2)
            self.set_buzzer(False)
            self.fc.set_rgb_led(0, 0, 0)
            sleep(0.2)
        sleep(2)  # 等待高度稳定
        self.navigation_to_waypoint(
            np.array([current_x, current_y])
            + np.array([self.avoidance_dist, 0])
        )
        self.wait_for_waypoint()
        sleep(1)
        self.navigation_to_waypoint(np.array([waypoint_x, waypoint_y]))
        self.wait_for_waypoint()

    def init_avoidance_args(self, type: int = 1, height: int = 180, move: int = 50):
        """
        type: 0 -> 起飞后正前方120度范围内找点
        type: 1 -> 过呼啦圈后避障,在前方180读范围内找点
        type: 2 -> 1号点起飞后避障,在右方120度范围内找点
        type: 3 -> 2号,3号点起飞后避障,在左方120度范围内找点
        height: 避障高度
        move: 避障距离
        """
        if type == 0:
            self.fp_from = -60
            self.fp_to = 60
        elif type == 1:
            self.fp_from = -90
            self.fp_to = 90
        elif type == 2:
            self.fp_from = 0
            self.fp_to = 120
        elif type == 3:
            self.fp_from = -120
            self.fp_to = 0
        else:
            raise ValueError("type must be 0, 1, 2 or 3")
        self.avoidance_height = height
        self.avoidance_dist = move