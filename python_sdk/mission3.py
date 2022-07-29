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
    if deg > 180:
        deg = deg - 360
    return deg


cfg = ConfigManager()

# 基地点
BASE_POINT = cfg.get_array("point-0")
logger.info(f"[MISSION] Loaded base point: {BASE_POINT}")

ENTER_POINT_1 = np.array([85, 310])  # 待调
ENTER_POINT_2 = np.array([85, 200])
ENTER_POINT_3 = np.array([85, 90])
DEG_RANGE_1 = (0, 90)
DEG_RANGE_2 = (-45, 45)
DEG_RANGE_3 = (-90, 0)

POINT_ID = cfg.get_int("enter-point") - 1
ENTER_POINT = list([ENTER_POINT_1, ENTER_POINT_2, ENTER_POINT_3])[POINT_ID]
logger.info(f"[MISSION] Loaded enter point: {ENTER_POINT}")
DEG_RANGE = list([DEG_RANGE_1, DEG_RANGE_2, DEG_RANGE_3])[POINT_ID]
logger.info(f"[MISSION] Loaded deg range: {DEG_RANGE}")


class Mission(object):
    def __init__(
        self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture, hmi: HMI
    ):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.hmi = hmi
        self.initial_yaw = self.fc.state.yaw.value
        self.playback = Playback()
        self.playback.load_file("/home/pi/Desktop/prj/python_sdk/door.mp3")
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
        self.navigation_to_waypoint(ENTER_POINT)
        self.wait_for_waypoint()
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
        self.navigation_flag = False
        self.height_pid.setpoint = self.across_height
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)

        ############ 找圈 ############
        from_ = DEG_RANGE[0]
        to_ = DEG_RANGE[1]
        dis_thre = 15
        range_limit = 3000
        ############ 穿越 ############
        x_vel = 20
        timeout = 8
        ###########################

        yaw_flag = False
        x_flag = False
        y_flag = False
        locked = False
        dfrom = from_
        dto = to_
        while True:
            sleep(0.01)
            # 固定扫描角度,指定向右为yaw轴零点
            diff_yaw = deg_360_180(self.fc.state.yaw.value - self.initial_yaw)
            ###############################
            # 未锁定时使用yaw更新范围
            if not locked:
                dfrom = int(from_ - diff_yaw)
                dto = int(to_ - diff_yaw)
            ###############################
            points = self.radar.map.find_two_point_with_given_distance(
                from_=dfrom,
                to_=dto,
                distance=110,
                threshold=dis_thre,
                range_limit=range_limit,
            )
            if len(points) == 0:
                self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                logger.info(f"[MISSION] LOSS :{diff_yaw} dfrom:{dfrom}  dto:{dto}")
            elif len(points) == 1:
                self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                logger.info(f"[Hoop] ERROR :{diff_yaw} dfrom:{dfrom}  dto:{dto}")
            elif len(points) == 2:
                logger.info(f"[hoop] LOCKED :{diff_yaw} dfrom:{dfrom}  dto:{dto}")
                locked = True
                a_point = points[0]
                b_point = points[1]
                a_point_xy = a_point.to_xy()
                b_point_xy = b_point.to_xy()
                mid_point = (a_point_xy + b_point_xy) / 2
                k = (b_point_xy[0] - a_point_xy[0]) / (
                    a_point_xy[1] - b_point_xy[1]
                )  # 换算到笛卡尔坐标系的斜率
                ###############################
                # 锁定后使用扫描到的点更新范围
                a_deg = deg_360_180(a_point.degree)
                b_deg = deg_360_180(b_point.degree)
                dfrom = int(min(a_deg, b_deg) - 15)
                dto = int(max(a_deg, b_deg) + 15)
                ###############################
                if k < -0.1:
                    self.fc.update_realtime_control(yaw=15)
                elif k > 0.1:
                    self.fc.update_realtime_control(yaw=-15)
                if abs(k - 0) < 0.1:
                    self.fc.update_realtime_control(yaw=0)
                    if not yaw_flag:
                        yaw_flag = True
                        logger.info("[MISSION] Hoop yaw finish")
                if yaw_flag:  # 如果yaw轴已经完成,则开始x轴
                    if mid_point[0] > 500:
                        # 应该向前移动
                        self.fc.update_realtime_control(vel_x=10)
                    elif mid_point[0] < 500:
                        # 应该向后移动
                        self.fc.update_realtime_control(vel_x=-10)
                    if abs(mid_point[0] - 500) < 50:
                        self.fc.update_realtime_control(vel_x=0)
                        if not x_flag:
                            x_flag = True
                            logger.info("[MISSION] Hoop x finish")
                if x_flag:  # 如果x轴已经完成,则开始y轴
                    if mid_point[1] > 35:
                        # 向左移动
                        self.fc.update_realtime_control(vel_y=10)
                    elif mid_point[1] < -35:
                        # 向右移动
                        self.fc.update_realtime_control(vel_y=-10)
                    if abs(mid_point[1]) < 35:  # mm
                        self.fc.update_realtime_control(vel_y=0)
                        if not y_flag:
                            y_flag = True
                            logger.info("[MISSION] Hoop y finish")
                if yaw_flag and x_flag and y_flag:
                    self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                    logger.info("[MISSION] Align Hoop finish")
                    break

        # 实时控制钻圈
        self.keep_height_flag = False
        start_time = time()
        logger.info("[MISSION] Start to pass hoop")

        y_fixed = False
        yaw_fixed = False
        while True:
            sleep(0.01)
            if time() - start_time > timeout:
                logger.info("[MISSION] Passing hoop overtime")
                self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                break
            left_point = self.radar.map.find_nearest(
                from_=-179, to_=-1, num=1, range_limit=2000
            )
            right_point = self.radar.map.find_nearest(
                from_=1, to_=179, num=1, range_limit=2000
            )
            if len(left_point) > 0 and len(right_point) > 0:  # 正常
                left_p = left_point[0].to_xy()
                right_p = right_point[0].to_xy()
                y_diff = abs(right_p[1]) - abs(left_p[1])  # 用于修正横向位移
                if y_diff > 60:  # 左偏，向右移动
                    self.fc.update_realtime_control(vel_y=-8)
                elif y_diff < -60:  # 右偏，向左移动
                    self.fc.update_realtime_control(vel_y=8)
                else:
                    self.fc.update_realtime_control(vel_y=0)
                    if not y_fixed:
                        y_fixed = True
                        logger.info("[MISSION] Hoop y fixed")

                x_diff = right_p[0] - left_p[0]  # 用于修正YAW轴，不使用degree修正以避免正反馈
                if x_diff > 30:  # 左偏，向右转动
                    self.fc.update_realtime_control(yaw=5)
                elif x_diff < -30:  # 右偏，向左转动
                    self.fc.update_realtime_control(yaw=-5)
                else:
                    self.fc.update_realtime_control(yaw=0)
                    if not yaw_fixed:
                        yaw_fixed = True
                        logger.info("[MISSION] Hoop yaw fixed")

                if left_p[0] < -800 and right_p[0] < -800:  # 到达安全距离
                    self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                    logger.info("[MISSION] Passing hoop finish")
                    break
            else:  # 无法获取点
                self.fc.update_realtime_control(vel_y=0, yaw=0)
                logger.info("[MISSION] Find no hoop edges")

            if yaw_fixed and y_fixed:
                self.fc.update_realtime_control(vel_x=x_vel)  # 开始前进

        logger.info("[MISSION] Across hoop OK")
