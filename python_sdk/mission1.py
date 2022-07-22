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
from FlightController.Solutions.Vision_Net import FastestDetOnnx
from simple_pid import PID


def deg_360_180(deg):
    if deg > 180:
        deg = deg - 360
    return deg


TARGET_POINT1 = np.array([325, 419])  # 左上方
TARGET_POINT2 = np.array([325, 71])  # 右上方
TARGET_POINT3 = np.array([73, 73])  # 右下方
# 基地点
BASE_POINT = np.array([79, 425])
# 钻圈点
circle_in = np.array([138, 250])
circle_in_wait = np.array([80, 250])
circle_out = circle_in + np.array([130, 0])
circle_out_wait = np.array([310, 250])
circle_speed = 18
circle_height = 136
# 降落点
landing_point = BASE_POINT

target_pos_dict = {}
target_action_dict = {}
target_list = []
# target_pos_dict = {
#     "car": TARGET_POINT1,
#     "house": TARGET_POINT2,
#     "hospital": TARGET_POINT3,
# }
# target_action_dict = {"hospital": 1, "house": 2, "car": 1}
# target_list = [
#     "house",
#     "car",
#     "hospital",
# ]


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.inital_yaw = self.fc.state.yaw.value
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
        self.navigation_speed = 25  # 导航速度
        self.cruise_height = 125  # 巡航高度
        self.set_buzzer = lambda x: fc.set_digital_output(0, x)
        self.pid_tunings = {
            "default": (0.4, 0, 0.08),  # 导航
            "landing": (0.25, 0.02, 0.06),  # 降落
            "circle": (0.5, 0, 0.08),  # 钻圈
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
        self.read_mission_info()
        fc.set_rgb_led(255, 0, 255)
        fc.event.key_short.wait_clear()
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
        ######## 依次前往三个建筑物
        last_target_pos = BASE_POINT
        for target in target_list:
            target_pos = target_pos_dict[target]
            logger.info(f"[MISSION] Go to {target} at {target_pos}")
            if last_target_pos is BASE_POINT or last_target_pos is TARGET_POINT3:
                if target_pos is TARGET_POINT1 or target_pos is TARGET_POINT2:
                    logger.info("[MISSION] Go to the other side")  # 正向穿洞
                    self.across_hoop(0)
                else:
                    logger.info("[MISSION] Go to the same side")  # 直接前往
            else:
                if target_pos is BASE_POINT or target_pos is TARGET_POINT3:
                    logger.info("[MISSION] Go to the other side")  # 逆向穿洞
                    self.across_hoop(1)
                else:
                    logger.info("[MISSION] Go to the same side")  # 直接前往
            self.navigation_to_waypoint(target_pos)
            self.wait_for_waypoint()
            self.pointing_landing(target_pos)
            self.handling_goods(target_action_dict[target])
            self.pointing_takeoff(target_pos)
            last_target_pos = target_pos
        ######## 回到基地点
        logger.info("[MISSION] Go to base")
        if last_target_pos is TARGET_POINT1 or last_target_pos is TARGET_POINT2:
            logger.info("[MISSION] Go to the other side")  # 逆向穿洞
            self.across_hoop(1)
        else:
            logger.info("[MISSION] Go to the same side")
        self.navigation_to_waypoint(BASE_POINT)
        self.wait_for_waypoint()
        self.pointing_landing(landing_point)
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

    def read_mission_info(self):
        global target_pos_dict
        global target_action_dict
        global target_list
        cam = self.cam
        fd = self.fd
        fc = self.fc
        for i in range(15):
            cam.read()  # 清除缓存
        # 读入路径点顺序
        logger.info("[MISSION] Ready to read targets pos")
        fc.set_rgb_led(255, 255, 0)
        for pos in (TARGET_POINT1, TARGET_POINT2, TARGET_POINT3):
            while True:
                cv2.waitKey(1)
                frame = cam.read()[1]
                cv2.imshow("Origin", frame)
                if frame is None:
                    continue
                get = fd.detect(frame)
                cv2.imshow("Result", frame)
                cv2.waitKey(1)
                if len(get) > 0:
                    _, name, _ = get[0]
                    if name not in target_pos_dict:
                        logger.info(f"[MISSION] Target {name} in {pos}")
                        target_pos_dict[name] = pos
                        fc.set_rgb_led(0, 255, 0)
                        sleep(0.5)
                        fc.set_rgb_led(255, 255, 0)
                        break
        logger.info("[MISSION] Done")
        fc.set_rgb_led(0, 0, 255)
        for i in range(30):
            cam.read()  # 清除缓存
        fc.set_rgb_led(255, 255, 0)
        logger.info("[MISSION] Ready to read targets info")
        for i in range(3):
            while True:
                frame = cam.read()[1]
                cv2.imshow("Origin", frame)
                if frame is None:
                    continue
                get = fd.detect(frame)
                cv2.imshow("Result", frame)
                cv2.waitKey(1)
                if len(get) > 0:
                    _, name, _ = get[0]
                    if name not in target_list:
                        logger.info(f"[MISSION] Target {i} -> {name}")
                        target_list.append(name)
                        fc.set_rgb_led(0, 255, 0)
                        sleep(0.5)
                        fc.set_rgb_led(255, 255, 0)
                        break
            while True:
                frame = cam.read()[1]
                cv2.imshow("Origin", frame)
                if frame is None:
                    continue
                get, _, _, data = find_QRcode_zbar(frame)
                cv2.waitKey(1)
                if get:
                    logger.info(f"[MISSION] Action {i} -> {data}")
                    target_action_dict[name] = int(data)
                    fc.set_rgb_led(0, 255, 0)
                    sleep(0.5)
                    fc.set_rgb_led(255, 255, 0)
                    break
        logger.info("[MISSION] All Done")
        logger.info(f"[MISSION] target_pos_dict: {target_pos_dict}")
        logger.info(f"[MISSION] target_action_dict: {target_action_dict}")
        logger.info(f"[MISSION] target_list: {target_list}")
        fc.set_rgb_led(0, 0, 255)
        sleep(1)
        fc.set_rgb_led(255, 255, 0)

    def across_hoop(self, type: int):
        """
        type: 0: 顺X轴方向穿过, 1: 逆X轴方向穿过
        """
        logger.info(f"[MISSION] Acrossing hoop {type}")
        if type == 0:
            in_point = circle_in
            wait_point = circle_in_wait
            out_point = circle_out
        elif type == 1:
            in_point = circle_out
            wait_point = circle_out_wait
            out_point = circle_in
        else:
            raise ValueError("type must be 0 or 1")
        self.height_pid.setpoint = circle_height
        self.navigation_to_waypoint(wait_point)
        self.wait_for_waypoint()
        self.switch_pid("circle")
        self.set_navigation_speed(circle_speed)
        self.navigation_to_waypoint(in_point)
        self.wait_for_waypoint(time_thres=2)
        sleep(1)
        self.keep_height_flag = False
        sleep(0.5)
        self.navigation_to_waypoint(out_point)
        self.wait_for_waypoint(time_thres=2)
        self.keep_height_flag = True
        self.switch_pid("default")
        self.set_navigation_speed(self.navigation_speed)
        self.height_pid.setpoint = self.cruise_height

    def handling_goods(self, type: int):
        """
        type: 1->取货   2->放货
        """
        if type == 1:
            for i in range(3):
                self.set_buzzer(True)
                self.fc.set_rgb_led(255, 0, 0)
                sleep(0.2)
                self.set_buzzer(False)
                self.fc.set_rgb_led(0, 0, 0)
                sleep(0.2)
        elif type == 2:
            for i in range(3):
                self.set_buzzer(True)
                self.fc.set_rgb_led(0, 0, 255)
                sleep(0.2)
                self.set_buzzer(False)
                self.fc.set_rgb_led(0, 0, 0)
                sleep(0.2)

    def avoidance_task(self):
        """
        开启实时避障,未完成
        """
        paused = False
        while self.running:
            sleep(0.1)
            if self.avoidance_flag:
                if paused:
                    paused = False
                    self.radar.start_find_point(2.5, 0, 0, 359, 1, 800)
                if self.radar.fp_points[0].distance < 400:
                    logger.info("[MISSION] find obstacle")
                    self.height_pid.setpoint = 180

            else:
                self.radar.stop_find_point()
                if not paused:
                    paused = True
                    self.update_realtime_control(vel_z=0)
                    logger.info("[MISSION] Avoidance paused")
