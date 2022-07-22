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


TARGET_POINT1 = np.array([325, 419])  # 任务点
BASE_POINT = np.array([79, 425])  # 基地点


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
            0.0,
            0.2,
            setpoint=0,
            output_limits=(-45, 45),
            auto_mode=False,
        )
        #####################################
        self.keep_height_flag = False
        self.navigation_flag = False
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
        self.navigation_speed = 15  # 导航速度
        self.cruise_height = 115  # 巡航高度
        self.set_buzzer = lambda x: fc.set_digital_output(0, x)
        self.pid_tunings = {
            "default": (0.4, 0, 0.08),  # 导航
            "landing": (0.25, 0.02, 0.06),  # 降落
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
        logger.info("[MISSION] Threads started")
        ################ 初始化 ################
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
        self.set_avoidance_args(0)
        self.wait_for_waypoint_with_avoidance()
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

    def wait_for_waypoint_with_avoidance(self, time_thres=1, pos_thres=15, timeout=60):
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
            self.avoidance_handler()

    def avoidance_handler(self):
        points = self.radar.map.find_nearest(
            self._avd_fp_from, self._avd_fp_to, 1, self._avd_fp_dist
        )
        if len(points) > 0:
            logger.info("[MISSION] Found obstacle")
            waypoint = np.array([self.navi_x_pid.setpoint, self.navi_y_pid.setpoint])
            pos_point = np.array([self.radar.rt_pose[0], self.radar.rt_pose[1]])
            self.navigation_to_waypoint(pos_point)  # 原地停下
            self.height_pid.setpoint = self._avd_height
            self.set_buzzer(True)
            self.fc.set_rgb_led(255, 0, 0)
            sleep(1)
            self.set_buzzer(False)
            self.fc.set_rgb_led(0, 0, 0)
            sleep(1)  # 等待高度稳定
            self.keep_height_flag = False
            self.navigation_flag = False
            self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
            self.fc.horizontal_move(self._avd_move, 25, self._avd_deg)
            self.fc.set_rgb_led(255, 255, 0)
            self.fc.wait_for_last_command_done()
            self.fc.set_rgb_led(0, 0, 0)
            self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
            self.keep_height_flag = True
            self.height_pid.setpoint = self.cruise_height
            sleep(1)  # 等待高度稳定
            self.navigation_flag = True
            self.navigation_to_waypoint(waypoint)

    def set_avoidance_args(
        self,
        deg: int = 0,
        deg_range: int = 30,
        dist: int = 600,
        avd_height: int = 200,
        avd_move: int = 180,
    ):
        """
        fp_deg: 目标避障角度(deg) (0~360)
        fp_deg_range: 目标避障角度范围(deg)
        fp_dist: 目标避障距离(mm)
        avd_height: 避障目标高度(cm)
        avd_move: 避障移动距离(cm)
        """
        self._avd_fp_from = deg - deg_range
        self._avd_fp_to = deg + deg_range
        self._avd_fp_dist = dist
        self._avd_height = avd_height
        self._avd_deg = deg
        self._avd_move = avd_move
