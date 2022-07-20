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


TARGET_POINT1 = np.array([0, 0])  # 左上方
TARGET_POINT2 = np.array([0, 0])  # 右上方
TARGET_POINT3 = np.array([0, 0])  # 右下方

# 任务点
waypoints = np.array([])
# 基地点
base_point = np.array([0, 0])
# 降落点
landing_point = base_point

target_pos_dict = {}

target_action_dict = {}

target_list = []


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.inital_yaw = self.fc.state.yaw.value
        self.fd = FastestDetOnnx(drawOutput=True)  # 初始化神经网络
        ############### PID #################
        self.height_pid = PID(
            0.8, 0.0, 0.1, setpoint=135, output_limits=(-15, 15), auto_mode=False
        )
        self.navi_x_pid = PID(
            0.5,
            0,
            0.02,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.navi_y_pid = PID(
            0.5,
            0,
            0.02,
            setpoint=0,
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.navi_yaw_pid = PID(
            1.0,
            0.0,
            0.05,
            setpoint=0,
            output_limits=(-45, 45),
            auto_mode=False,
        )
        #####################################
        self.keep_height_flag = False
        self.navigation_flag = False
        self.running = False
        self.thread_list = []
        vision_debug()

    def run(self):
        fc = self.fc
        radar = self.radar
        cam = self.cam
        ############### 参数 #################
        camera_down_pwm = 32.5
        camera_up_pwm = 72
        navigation_speed = 40
        set_buzzer = lambda x: fc.set_digital_output(0, x)
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
        fc.set_PWM_output(0, camera_up_pwm)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        self.set_navigation_speed(navigation_speed)
        self.read_mission_info()
        fc.set_rgb_led(255, 0, 255)
        fc.event.key_short.wait_clear()
        fc.set_action_log(True)
        return
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission-1 Started")
        fc.set_rgb_led(255, 0, 0)  # 起飞前警告
        for i in range(10):
            sleep(0.1)
            set_buzzer(True)
            sleep(0.1)
            set_buzzer(False)
        fc.set_rgb_led(0, 0, 0)
        fc.unlock()
        sleep(2)  # 等待电机启动
        fc.take_off(80)
        fc.wait_for_takeoff_done()
        ######## 闭环定高
        fc.set_flight_mode(fc.HOLD_POS_MODE)
        self.keep_height_flag = True
        fc.start_realtime_control(10)
        sleep(2)
        self.navigation_to_waypoint(base_point)  # 初始化路径点
        sleep(0.1)
        self.navigation_flag = True
        fc.set_PWM_output(0, camera_down_pwm)
        ######## 遍历路径
        for n, waypoint in enumerate(waypoints):
            logger.info(f"[MISSION] Navigation to Waypoint-{n:02d}: {waypoint}")
            self.navigation_to_waypoint(waypoint)
            self.wait_for_waypoint()
            self.sow()
        ######## 精准着陆
        logger.info("[MISSION] Landing")
        self.height_pid.setpoint = 60
        self.navigation_to_waypoint(landing_point)
        self.wait_for_waypoint()
        self.height_pid.setpoint = 20
        sleep(2)
        self.wait_for_waypoint()
        self.height_pid.setpoint = 0
        fc.wait_for_lock(4)
        fc.lock()
        logger.info("[MISSION] Mission-1 Finished")

    def stop(self):
        self.running = False
        self.fc.stop_realtime_control()

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
                    self.fc.update_realtime_control(vel_z=0, yaw=0)
                    logger.info("[MISSION] Keep height paused")

    def navigation_task(self):
        ######## 解算参数 ########
        SIZE = 1000
        SCALE_RATIO = 0.5
        LOW_PASS_RATIO = 0.5
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
        TIME_THRESHOLD = 1
        OVERTIME_THRESHOLD = 30
        time_count = 0
        time_start = time()
        while True:
            sleep(0.1)
            if self.reached_waypoint():
                time_count += 0.1
            if time_count >= TIME_THRESHOLD:
                logger.info("[MISSION] Reached waypoint")
                return
            if time() - time_start > OVERTIME_THRESHOLD:
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
