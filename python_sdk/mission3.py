import threading
from time import sleep, time

import cv2
import numpy as np
from configManager import ConfigManager
from FlightController import FC_Client, FC_Controller, logger
from FlightController.Components import LD_Radar, Map_360, Point_2D
from FlightController.Solutions.Vision import (
    change_cam_resolution,
    find_QRcode_zbar,
    set_cam_autowb,
    vision_debug,
)
from FlightController.Solutions.Vision_Net import FastestDetOnnx
from hmi import HMI
from simple_pid import PID


def deg_360_180(deg):
    if deg > 180:
        deg = deg - 360
    elif deg < -180:
        deg = deg + 360
    return deg


# 基地点
# BASE_POINT = np.array([79, 425])
BASE_POINT = np.array([72, 75])
# 降落点
landing_point = BASE_POINT
# 任务点
start_point_a = np.array([110, 275])  # 1 9之间
start_point_b = np.array([270, 125])  # 2 10之间
start_point_c = np.array([75, 200])  # 9后
start_point_d = np.array([200, 70])  # 7右
start_point_e = np.array([120, 120])  # 7后
start_point_f = np.array([270, 270])  # 3 8之间
deg_from = {"a": -30, "b": -120, "c": -60, "d": -135, "e": -30, "f": 60}
deg_to = {"a": 30, "b": -60, "c": 60, "d": -45, "e": 30, "f": 120}
point_range = {"a": 2100, "b": 2200, "c": 2200, "d": 2000, "e": 2000, "f": 2200}
select_point = {
    "a": start_point_a,
    "b": start_point_b,
    "c": start_point_c,
    "d": start_point_d,
    "e": start_point_e,
    "f": start_point_f,
}

start_point_name = "e"

# default_dict = {
#     "start_point_name": "b",
# }
# cfg = ConfigManager(default_setting=default_dict)
# start_point_name = str(cfg.get_array("start_point_name"))

start_point = select_point[start_point_name]


class Mission(object):
    def __init__(
        self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture, hmi: HMI
    ):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.hmi = hmi
        self.initial_yaw = self.fc.state.yaw.value
        self.fd = FastestDetOnnx(drawOutput=True)  # 初始化神经网络
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
            0,
            0.1,
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
        self.camera_down_pwm = 20
        self.camera_up_pwm = 60
        self.navigation_speed = 25  # 导航速度
        self.cruise_height = 135  # 巡航高度
        self.across_height = 135  # 钻圈高度(待调)
        self.set_buzzer = lambda x: fc.set_digital_output(2, x)
        self.pid_tunings = {
            "default": (0.3, 0, 0.08),  # 导航
            "landing": (0.3, 0.01, 0.08),  # 降落
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
        # change_cam_resolution(cam, 800, 600)
        # set_cam_autowb(cam, False)  # 关闭自动白平衡
        # for _ in range(10):
        #     cam.read()
        fc.set_PWM_output(0, self.camera_up_pwm)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        self.set_navigation_speed(self.navigation_speed)
        fc.set_rgb_led(255, 0, 0)  # 起飞前警告
        for i in range(10):
            sleep(0.1)
            self.set_buzzer(True)
            sleep(0.1)
            self.set_buzzer(False)
        fc.set_rgb_led(0, 0, 0)
        fc.set_action_log(True)
        self.fc.start_realtime_control(20)
        self.switch_pid("default")
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission-1 Started")
        self.pointing_takeoff(BASE_POINT)
        ############### 开始呼啦圈任务 ##############
        self.navigation_to_waypoint(start_point)
        self.wait_for_waypoint()
        self.across_hoop(
            deg_from[start_point_name],
            deg_to[start_point_name],
            point_range[start_point_name],
        )
        ######## 原地降落
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.land()
        fc.wait_for_lock()
        logger.info("[MISSION] Misson-1 Finished")

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

    def across_hoop(
        self,
        from_: int = 0,
        to_: int = 359,
        range_limit: int = 1e7,
        dis_thre: int = 15,
        timeout: int = 60,
    ):
        """
        找到任意角度的呼啦圈
        dis_thre: 识别到的两个点的距离阈值 cm
        timeout: 超时时间
        """
        count1 = 0
        sum = np.array([0, 0], dtype = np.float64)
        while True:
            sleep(0.1)
            points = self.radar.map.find_two_different_nearest_point(
                from_, to_, range_limit, dis_thre
            )
            if len(points) == 2:
                count1 += 1
                a_point = points[0].to_xy() / 10
                b_point = points[1].to_xy() / 10
                mid_point = (a_point + b_point) / 2
                sum += mid_point
                if count1 > 3:
                    mid_point = sum / count1
                    logger.info("[MISSION] Find hoop")
                    break
            else:
                logger.info("[MISSION] Not find hoop")
        self.set_navigation_speed(15)
        if start_point_name in ["a", "c", "e"]:
            to_point = np.array([0, 0])
            to_point[0] = start_point[0] + mid_point[0] - 65
            to_point[1] = start_point[1] + mid_point[1]
        elif start_point_name in ["b", "d"]:
            to_point = np.array([0, 0])
            to_point[0] = start_point[0] + mid_point[0]
            to_point[1] = start_point[1] + mid_point[1] - 65
        elif start_point_name == "f":
            to_point = np.array([0, 0])
            to_point[0] = start_point[0] + mid_point[0]
            to_point[1] = start_point[1] + mid_point[1] + 65
        logger.info(f"[MISSION] to_point: {to_point}")
        self.navigation_to_waypoint(to_point)
        self.wait_for_waypoint()
        sleep(0.5)
        self.navigation_flag = False
        self.height_pid.setpoint = self.across_height

        if start_point_name in ["a", "c", "e"]:
            logger.info("[MISSION] no turn")
        elif start_point_name in ["b", "d"]:
            self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
            self.fc.turn_left(90, 20)
            self.fc.wait_for_last_command_done()
            self.fc.horizontal_move(10, 15, 0)
            self.fc.wait_for_last_command_done()
        elif start_point_name == "f":
            self.fc.set_flight_mode(self.fc.PROGRAM_MODE)
            self.fc.turn_right(90, 20)
            self.fc.wait_for_last_command_done()
            self.fc.horizontal_move(10, 15, 0)
            self.fc.wait_for_last_command_done()
        self.fc.set_flight_mode(self.fc.HOLD_POS_MODE)

        yaw_flag = False
        x_flag = False
        count2 = 0
        start_time = time()
        while True:
            sleep(0.1)
            if not yaw_flag:
                if start_point_name in ['a','c','e']:
                    points = self.radar.map.find_two_different_nearest_point(
                        0, 359, 1500, dis_thre
                    )
                else:
                    points = self.radar.map.find_two_different_nearest_point(
                        0, 359, 2000, dis_thre
                    )
            else:
                points = self.radar.map.find_two_different_nearest_point(
                    -90, 90, 1200, dis_thre
                )
            if len(points) == 2:
                a_point = points[0].to_xy()
                b_point = points[1].to_xy()
                k = (b_point[0] - a_point[0]) / (
                    a_point[1] - b_point[1]
                )  # 换算到笛卡尔坐标系的斜率
                if k < 0:
                    self.fc.update_realtime_control(yaw=15)
                elif k > 0:
                    self.fc.update_realtime_control(yaw=-15)
                if abs(k - 0) < 0.1:
                    self.fc.update_realtime_control(yaw=0)
                    yaw_flag = True
                    logger.info("[MISSION] Hoop yaw finish")
                if yaw_flag:
                    mid_point = (a_point + b_point) / 2
                    if mid_point[0] > 500:
                        # 如果中点x坐标大于500mm，则应该向前移动
                        self.fc.update_realtime_control(vel_x=10)
                    elif mid_point[0] < 500:
                        # 如果中点x坐标小于500mm，则应该向后移动
                        self.fc.update_realtime_control(vel_x=-10)
                    if abs(mid_point[0] - 500) < 50:
                        self.fc.update_realtime_control(vel_x=0)
                        x_flag = True
                if x_flag:
                    mid_point = (a_point + b_point) / 2
                    if mid_point[1] > 35:
                        # 如果中点y坐标大于50，则应该向左移动
                        self.fc.update_realtime_control(vel_y=10)
                    elif mid_point[1] < -35:
                        # 如果中点y坐标小于-50，则应该向右移动
                        self.fc.update_realtime_control(vel_y=-10)
                    if abs(mid_point[1]) < 35:  # mm
                        count2 += 1
                        self.fc.update_realtime_control(vel_y=0)
                        logger.info("[MISSION] Y success")
                        if count2 > 3:
                            self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                            logger.info("[MISSION] Align Hoop finish")
                            break
            else:
                self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
                logger.info("[MISSION] Hoop find error")
            if time() - start_time > timeout:
                logger.warning("[MISSION] Align_hoop overtime")
                break

        sleep(0.5)
        self.keep_height_flag = False
        self.fc.update_realtime_control(vel_x=21, vel_y=0, vel_z=0, yaw=0)
        sleep(6)
        self.fc.update_realtime_control(vel_x=0, vel_y=0, vel_z=0, yaw=0)
        self.keep_height_flag = True
        sleep(0.5)
