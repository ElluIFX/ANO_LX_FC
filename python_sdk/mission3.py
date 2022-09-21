import threading
from time import sleep, time

import cv2
import numpy as np
from configManager import ConfigManager
from FlightController import FC_Client, FC_Controller, logger
from FlightController.Components import LD_Radar, Map_360, Point_2D
from FlightController.Solutions.Vision import (
    change_cam_resolution,
    set_cam_autowb,
    vision_debug,
)
from FlightController.Solutions.Vision_Net import FastestDetOnnx
from hmi import HMI
from just_playback import Playback
from simple_pid import PID


def deg_360_180(deg):
    while deg > 180:
        deg -= 360
    while deg < -180:
        deg += 360
    return deg


cfg = ConfigManager()

# 基地点
BASE_POINT = np.array([75, 75])

# 左边，正面进入
ENTER_PATH_1 = np.array([[110, 275]])
ENTER_TURN_1 = 0
# 左边，侧面进入
ENTER_PATH_2 = np.array([[270, 125]])
ENTER_TURN_2 = -90
# 中间，正面进入
ENTER_PATH_3 = np.array([[75, 200]])
ENTER_TURN_3 = 0
# 中间，侧面进入
ENTER_PATH_4 = np.array([[200, 70]])
ENTER_TURN_4 = -90
# 右边，正面进入
ENTER_PATH_5 = np.array([[120, 120]])
ENTER_TURN_5 = 0
# 右边，侧面进入
ENTER_PATH_6 = np.array([[110, 275], [270, 270]])
ENTER_TURN_6 = 90

POINT_ID = cfg.get_int("enter-point") - 1
ENTER_PATH = (
    ENTER_PATH_1,
    ENTER_PATH_2,
    ENTER_PATH_3,
    ENTER_PATH_4,
    ENTER_PATH_5,
    ENTER_PATH_6,
)[POINT_ID]
logger.info(f"[MISSION] Loaded enter path {ENTER_PATH}")
ENTER_TURN = (
    ENTER_TURN_1,
    ENTER_TURN_2,
    ENTER_TURN_3,
    ENTER_TURN_4,
    ENTER_TURN_5,
    ENTER_TURN_6,
)[POINT_ID]
logger.info(f"[MISSION] Loaded enter turn {ENTER_TURN}")


class Mission(object):
    def __init__(
        self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture, hmi: HMI
    ):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.hmi = hmi
        self.inital_yaw = self.fc.state.yaw.value
        self.playback = Playback()
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
            0.2,
            0.0,
            0.0,
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
        self.navigation_speed = 35  # 导航速度
        self.precision_speed = 25  # 精确速度
        self.cruise_height = 140  # 巡航高度
        self.goods_height = 80  # 处理物品高度
        self.across_height = 140  # 钻圈高度(待调)
        self.pid_tunings = {
            "default": (0.35, 0, 0.08),  # 导航
            "delivery": (0.4, 0.05, 0.16),  # 配送
            "landing": (0.4, 0.05, 0.16),  # 降落
        }  # PID参数 (仅导航XY使用)
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
        change_cam_resolution(cam, 800, 600)
        set_cam_autowb(cam, False)  # 关闭自动白平衡
        for _ in range(10):
            cam.read()
        fc.set_PWM_output(0, self.camera_up_pwm)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        self.set_navigation_speed(self.navigation_speed)
        # self.playback.loop_at_end(True)
        # self.playback.play()
        for i in range(6):
            sleep(0.25)
            fc.set_rgb_led(255, 0, 0)  # 起飞前警告
            sleep(0.25)
            fc.set_rgb_led(0, 0, 0)
        # self.playback.stop()
        fc.set_PWM_output(0, self.camera_down_pwm)
        fc.set_digital_output(2, True)  # 激光笔开启
        fc.set_action_log(True)
        self.fc.start_realtime_control(20)
        self.switch_pid("default")
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission-3 Started")
        self.pointing_takeoff(BASE_POINT)
        ################ 开始任务 ################
        for point in ENTER_PATH:
            self.navigation_to_waypoint(point)
            self.wait_for_waypoint()
        self.navigation_flag = False  # 暂停导航
        if ENTER_TURN != 0:
            fc.set_flight_mode(fc.PROGRAM_MODE)
            if ENTER_TURN < 0:
                fc.turn_left(abs(ENTER_TURN), 45)
            else:
                fc.turn_right(ENTER_TURN, 45)
            fc.wait_for_hovering()
            fc.set_flight_mode(fc.HOLD_POS_MODE)
        self.across_hoop()
        ##########################################
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.land()
        logger.info("[MISSION] Misson-3 Finished")

    def pointing_takeoff(self, point):
        """
        定点起飞
        """
        logger.info(f"[MISSION] Takeoff at {point}")
        self.navigation_flag = False
        self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
        self.fc.unlock()
        inital_yaw = self.fc.state.yaw.value
        sleep(2)  # 等待电机启动
        self.fc.take_off(80)
        self.fc.wait_for_takeoff_done()
        self.fc.set_yaw(inital_yaw, 25)
        self.fc.wait_for_hovering(2)
        ######## 闭环定高
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)
        self.height_pid.setpoint = self.cruise_height
        self.keep_height_flag = True
        sleep(2)
        self.navigation_to_waypoint(point)  # 初始化路径点
        self.switch_pid("default")
        sleep(0.1)
        self.navigation_flag = True
        self.set_navigation_speed(self.navigation_speed)

    def pointing_landing(self, point):
        """
        定点降落
        """
        logger.info(f"[MISSION] Landing at {point}")
        self.navigation_to_waypoint(point)
        self.wait_for_waypoint()
        self.set_navigation_speed(self.precision_speed)
        self.switch_pid("landing")
        sleep(1)
        self.height_pid.setpoint = 60
        sleep(1.5)
        self.height_pid.setpoint = 30
        sleep(1.5)
        self.height_pid.setpoint = 20
        sleep(2)
        self.wait_for_waypoint()
        self.height_pid.setpoint = 0
        # self.fc.land()
        self.fc.wait_for_lock(5)
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
        logger.info("[MISSION] Navigation to waypoint: %s", waypoint)

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

    def across_hoop(self):
        fc = self.fc
        radar = self.radar
        self.height_pid.setpoint = self.across_height
        sleep(2)  # 等待高度稳定
        radar.start_find_point(1, 0, -45, 45, 1, 3000)
        y_locked = False  # y 是否锁定
        x_locked = False  # x 是否锁定
        y_pid = PID(1.2, 0.02, 0.05, setpoint=0, output_limits=(-15, 15))
        x_pid = PID(
            0.6, 0.02, 0.05, setpoint=50, output_limits=(-15, 15), auto_mode=False
        )
        self.navigation_flag = False  # 暂停导航
        while True:
            sleep(0.1)
            if radar.fp_timeout_flag:
                fc.update_realtime_control(vel_x=0, vel_y=0)
            if len(radar.fp_points) > 0:
                deg = deg_360_180(radar.fp_points[0].degree)
                dis = radar.fp_points[0].distance / 10
                logger.info("[MISSION] find point A: %.2f, %.2f" % (deg, dis))
                out_y = y_pid(deg)
                fc.update_realtime_control(vel_y=out_y)
                if -5 < deg < 5:
                    if not y_locked:
                        y_locked = True
                        logger.info("[MISSION] Y locked")
                        x_pid.set_auto_mode(True)
                if y_locked:
                    out_x = x_pid(dis)
                    fc.update_realtime_control(vel_x=-out_x)
                    if -5 < dis - 50 < 5:
                        if not x_locked:
                            x_locked = True
                            ok_time = time()
                            logger.info("[MISSION] X locked")
                if x_locked:
                    if time() - ok_time > 2:
                        break
            else:
                fc.update_realtime_control(vel_x=0, vel_y=0)
        fc.update_realtime_control(vel_x=0, vel_y=0)
        dis_pid = PID(
            0.6, 0.02, 0.05, setpoint=50, output_limits=(-20, 20), auto_mode=True
        )
        yaw_pid = PID(
            0.6, 0.02, 0.1, setpoint=20, output_limits=(-45, 45), auto_mode=False
        )
        yaw_locked = False  # yaw 是否锁定
        x_locked = False  # x 是否锁定
        radar.update_find_point_args(-30, 30, 1, 3000)
        last_yaw = fc.state.yaw.value
        turn_count = 0
        logger.info("[MISSION] Start across hoop")
        self.keep_height_flag = False  # 暂停高度控制
        while True:
            sleep(0.1)
            if radar.fp_timeout_flag:
                fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
            if len(radar.fp_points) > 0:
                deg = deg_360_180(radar.fp_points[0].degree)
                dis = radar.fp_points[0].distance / 10
                logger.info("[MISSION] find point B: %.2f, %.2f" % (deg, dis))
                out_dis = dis_pid(dis)
                fc.update_realtime_control(vel_x=-out_dis)
                if abs(dis - dis_pid.setpoint) < 10:
                    if not x_locked:
                        x_locked = True
                        yaw_pid.set_auto_mode(True)
                        logger.info("[MISSION] X locked")
                if x_locked:
                    out_yaw = yaw_pid(deg)
                    if out_yaw is not None:
                        fc.update_realtime_control(yaw=-out_yaw)
                        if abs(deg - yaw_pid.setpoint) < 5:
                            if not yaw_locked:
                                yaw_locked = True
                                logger.info("[MISSION] Yaw locked")
                if x_locked and yaw_locked:
                    radar.update_find_point_args(-30, 30, 1, 1000)
                    fc.update_realtime_control(vel_y=-8)
            else:
                # 没有雷达点，x速度置为零
                fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
            current_yaw = fc.state.yaw.value
            diff = abs(current_yaw - last_yaw)
            last_yaw = current_yaw
            if diff > 180:
                diff = 360 - diff
            turn_count += diff
            # if turn_count > 100:
            #     ok_points = radar.map.find_nearest(-135, -45, 1)  # 左边
            #     if len(ok_points) > 0:
            #         deg2 = deg_360_180(ok_points[0].degree)
            #         dis2 = ok_points[0].distance / 10
            #         logger.info("[MISSION] find point C: %.2f, %.2f" % (deg2, dis2))
            #         if 120 < dis2 < 150:
            #             logger.info("[MISSION] Stop hoop")
            #             break
            if turn_count > 300:
                logger.info("[MISSION] Stop hoop")
                break
        fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
        logger.info("[MISSION] Across hoop OK")
