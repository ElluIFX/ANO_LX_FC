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


OFFSET = np.array([18, 25])
CORNER_POINT = np.array([53, 51]) + OFFSET
Y_BOX = np.array([0, 50])
X_BOX = np.array([50, 0])

m_point = lambda x, y: CORNER_POINT + X_BOX * x + Y_BOX * y
# 进入航线
enter_points = np.array([m_point(0, 7.2), m_point(4, 7.2)])
# 进入点 (A)
start_point = m_point(4, 6)
# 任务点
waypoints = np.array(
    [
        m_point(5, 6),
        m_point(5, 5),
        m_point(5, 4),
        m_point(5, 3),
        m_point(5, 2),
        m_point(5, 1),
        m_point(5, 0),
        m_point(4, 0),
        m_point(3, 0),
        m_point(3, 1),
        m_point(4, 1),
        m_point(4, 2),
        m_point(3, 2),
        m_point(2, 2),
        m_point(1, 2),
        m_point(1, 1),
        m_point(1, 0),
        m_point(0, 0),
        m_point(0, 1),
        m_point(0, 2),
        m_point(0, 3),
        m_point(1, 3),
        m_point(2, 3),
        m_point(3, 3),
        m_point(4, 3),
        m_point(4, 4),
        m_point(4, 5),
    ]
)
# 回航点
return_points = np.array([m_point(4, 7)])
# 基地点
base_point = m_point(0, 7)  # np.array([82, 432])  # m_point(0, 7)
# 降落点
landing_point = base_point


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.inital_yaw = self.fc.state.yaw.value
        ############### PID #################
        self.height_pid = PID(
            0.8, 0.0, 0.1, setpoint=135, output_limits=(-15, 15), auto_mode=False
        )
        self.navi_x_pid = PID(
            0.5,
            0,
            0.02,
            setpoint=start_point[0],
            output_limits=(-0.01, 0.01),
            auto_mode=False,
        )
        self.navi_y_pid = PID(
            0.5,
            0,
            0.02,
            setpoint=start_point[1],
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
        # vision_debug()

    def run(self):
        ############### 参数 #################
        camera_down_pwm = 32.5
        camera_up_pwm = 72
        navigation_speed = 25
        space_distance = 50
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
        set_cam_autowb(cam, False)  # 关闭自动白平衡
        for _ in range(10):
            cam.read()
        fc.set_PWM_output(0, camera_up_pwm)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        self.set_navigation_speed(navigation_speed)
        ################ 初始化完成 ################
        logger.info("[MISSION] Mission-1 Started")
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
        ########进入
        for n, enter_point in enumerate(enter_points):
            logger.info(f"[MISSION] Navigation to Enter-{n:02d}: {enter_point}")
            self.navigation_to_waypoint(enter_point)
            self.wait_for_waypoint()
        ######## 飞进入点
        logger.info("[MISSION] Navigation to Start point")
        self.navigation_to_waypoint(start_point)
        self.wait_for_waypoint()
        self.sow()
        ######## 遍历路径
        for n, waypoint in enumerate(waypoints):
            logger.info(f"[MISSION] Navigation to Waypoint-{n:02d}: {waypoint}")
            self.navigation_to_waypoint(waypoint)
            self.wait_for_waypoint()
            self.sow()
        ######## 寻杆，找条形码
        fc.set_PWM_output(0, camera_up_pwm)
        self.navigation_flag = False
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.turn_right(180, 90)
        fc.wait_for_hovering()
        fc.set_flight_mode(fc.HOLD_POS_MODE)
        self.navigation_to_waypoint((125, 175))
        self.navigation_flag = True
        self.set_navigation_speed(navigation_speed * 2)
        self.wait_for_waypoint()
        self.navigation_flag = False
        radar.start_find_point(2.5, 0, -30, 30, 1, 2000)
        x_locked = False
        y_locked = False
        find_QR = False
        time_out = 10
        start_time = time()
        while True:
            sleep(0.01)
            if self.radar.fp_timeout_flag:
                self.fc.update_realtime_control(vel_x=0, vel_y=0)
            if len(self.radar.fp_points) > 0:
                self.radar.fp_points.sort(key=lambda x: abs(deg_360_180(x.degree)))
                deg = self.radar.fp_points[0].degree
                deg = deg_360_180(deg)
                dis = self.radar.fp_points[0].distance / 10
                logger.info("[MISSION] find point 1: %.2f, %.2f" % (deg, dis))
                if deg > 1:
                    fc.update_realtime_control(vel_y=-6)
                elif deg < -1:
                    fc.update_realtime_control(vel_y=6)
                else:
                    fc.update_realtime_control(vel_y=0)
                    y_locked = True
                if y_locked:
                    if dis > space_distance + 5:
                        fc.update_realtime_control(vel_x=8)
                    elif dis < space_distance - 5:
                        fc.update_realtime_control(vel_x=-8)
                    else:
                        fc.update_realtime_control(vel_x=0)
                        x_locked = True
                if x_locked and y_locked:
                    fc.update_realtime_control(vel_x=0, vel_y=0)
                    for n in range(5):
                        if self.find_barcode():
                            find_QR = True
                            break
                        else:
                            fc.set_PWM_output(0, camera_up_pwm - n * 0.5)
                            logger.info(f"[MISSON] QR not found :{n}")
                if find_QR or time() - start_time > time_out:
                    break
        radar.stop_find_point()
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.turn_right(180, 60)
        fc.wait_for_last_command_done()
        fc.horizontal_move(25, 15, 270)
        fc.wait_for_last_command_done()
        fc.set_flight_mode(fc.HOLD_POS_MODE)
        self.navigation_flag = True
        ######## 返航
        for n, return_point in enumerate(return_points):
            logger.info(f"[MISSION] Navigation to Return-{n:02d}: {return_point}")
            self.navigation_to_waypoint(return_point)
            self.wait_for_waypoint()
        ######## 精准着陆
        logger.info("[MISSION] Landing")
        self.height_pid.setpoint = 80
        self.navigation_to_waypoint(landing_point)
        self.wait_for_waypoint()
        self.height_pid.setpoint = 40
        sleep(1)
        self.wait_for_waypoint()
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.stop_realtime_control()
        fc.land()
        fc.wait_for_lock()
        logger.info("[MISSION] Mission-1 Finished")

    def sow(self):  # 播撒
        if not self.check_green_ground():
            logger.info("[MISSION] No green ground, skipped")
            return
        logger.info("[MISSION] Green ground detected")
        self.fc.set_action_log(False)
        self.fc.set_digital_output(2, 1)
        self.fc.set_digital_output(0, 1)
        sleep(1)
        self.fc.set_digital_output(2, 0)
        self.fc.set_digital_output(0, 0)
        sleep(0.5)
        self.fc.set_action_log(True)
        logger.info("[MISSION] Sow Done")

    def check_green_ground(self):
        LOWER = HSV.GREEN_LOWER
        UPPER = HSV.GREEN_UPPER
        THRESHOLD = 0.7  # 颜色判断阈值
        ROI = (0.45, 0.6, 0.1, 0.1)  # 根据高度调整
        CHECK_NUM = 10  # 检测次数
        # results = []
        # for _ in range(CHECK_NUM):
        #     img = self.cam.read()[1]
        #     img = get_ROI(img, ROI)
        #     results.append(hsv_checker(img, LOWER, UPPER, THRESHOLD))
        # logger.info(f"[MISSION] Green ground: {results}")
        # return all(results)
        for _ in range(CHECK_NUM):
            img = self.cam.read()[1]
        img = self.cam.read()[1]
        cv2.imshow("Origin", img)
        img = get_ROI(img, ROI)
        result = hsv_checker(img, LOWER, UPPER, THRESHOLD)
        cv2.waitKey(10)
        return result

    def find_barcode(self):
        global landing_point
        img = self.cam.read()[1]
        get, dx, dy, data = find_QRcode_zbar(img)
        if get:
            try:
                num = int(data)
                logger.info(f"[MISSION] Barcode detected: {num}")
                for _ in range(num):  # 声光报警
                    self.fc.set_rgb_led(255, 255, 0)
                    self.fc.set_digital_output(0, 1)
                    sleep(0.6)
                    self.fc.set_rgb_led(0, 0, 0)
                    self.fc.set_digital_output(0, 0)
                    sleep(0.4)
                landing_point += np.array([int(num * 10), 0])
            except Exception as e:
                logger.error(f"[MISSION] Barcode error: {e}")

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
            sleep(0.05)
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
                # if self.radar.rt_pose_update_event.wait(1):  # 等待地图更新
                #     self.radar.rt_pose_update_event.clear()
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
