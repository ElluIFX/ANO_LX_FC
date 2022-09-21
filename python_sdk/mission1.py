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
# 降落点
landing_point = BASE_POINT
# 任务坐标
NULL_PT = np.array([np.NaN, np.NaN])
POINT = lambda x: cfg.get_array(f"point-{x}")
POINTS_ARR = np.array(
    [
        [POINT(1), NULL_PT, POINT(11), NULL_PT, POINT(5)],
        [NULL_PT, POINT(8), NULL_PT, POINT(3), NULL_PT],
        [POINT(9), NULL_PT, POINT(2), NULL_PT, POINT(12)],
        [NULL_PT, POINT(7), NULL_PT, POINT(6), NULL_PT],
        [NULL_PT, NULL_PT, POINT(10), NULL_PT, POINT(4)],
    ]
)
logger.info(f"[MISSION] Loaded points: {POINTS_ARR}")
# 任务点
RED_TRIANGLES = [(0, 0), (2, 2)]
RED_RECTANGLES = [(0, 2), (2, 4)]
RED_CIRCLES = [(4, 0), (3, 3)]
BLUE_TRIANGLES = [(3, 1), (4, 4)]
BLUE_RECTANGLES = [(2, 0), (4, 2)]
BLUE_CIRCLES = [(1, 1), (1, 3)]
target_points = [cfg.get_array("target-1"), cfg.get_array("target-2")]
logger.info(f"[MISSION] Loaded target points: {target_points}")


class Mission(object):
    def __init__(
        self, fc: FC_Controller, radar: LD_Radar, camera: cv2.VideoCapture, hmi: HMI
    ):
        self.fc = fc
        self.radar = radar
        self.cam = camera
        self.hmi = hmi
        self.inital_yaw = self.fc.state.yaw.value
        self.fd = FastestDetOnnx(drawOutput=True)  # 初始化神经网络
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
        # self.recognize_targets()
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
        logger.info("[MISSION] Mission-1 Started")
        self.pointing_takeoff(BASE_POINT)
        ################ 开始任务 ################
        for target_point in target_points:
            x, y = target_point
            target_point_pos = POINTS_ARR[y, x]
            self.navigation_to_waypoint(target_point_pos)
            self.wait_for_waypoint()
            self.handle_goods()
        ######## 回到基地点
        logger.info("[MISSION] Go to base")
        self.navigation_to_waypoint(BASE_POINT)
        self.wait_for_waypoint()
        self.pointing_landing(landing_point)
        logger.info("[MISSION] Misson-1 Finished")

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

    def handle_goods(self):
        """
        处理物品
        """
        logger.info(f"[MISSION] Handle goods")
        self.height_pid.setpoint = self.goods_height
        self.set_navigation_speed(self.precision_speed)
        self.switch_pid("delivery")
        sleep(4)  # 等待高度稳定
        #####################################
        self.fc.set_pod(1, 8500)
        sleep(8.5)
        ####################################
        # self.playback.loop_at_end(True)
        # self.playback.play()
        self.fc.set_rgb_led(0, 255, 0)
        self.fc.set_digital_output(0, True)
        sleep(5)
        self.fc.set_digital_output(0, False)
        # self.playback.stop()
        self.fc.set_rgb_led(0, 0, 0)
        #####################################
        self.fc.set_pod(2, 10000)
        sleep(8.5)
        ####################################
        self.switch_pid("default")
        self.height_pid.setpoint = self.cruise_height
        sleep(4)  # 等待高度稳定
        ####################################
        self.set_navigation_speed(self.navigation_speed)

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
            self.fc.set_rgb_led(255, 0, 0)
            sleep(1)
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
        avd_move: int = 150,
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

    def recognize_targets(self):
        global target_points
        target_points = []
        logger.info("[MISSION] Recognizing targets")
        self.hmi.info("正在识别, 请保持目标在视野内\n尚未识别到目标")
        self.fc.set_rgb_led(255, 255, 0)
        rec_dict = {}
        start = False
        last_scan_time = time()
        while True:
            img = self.cam.read()[1]
            if img is None:
                continue
            get = self.fd.detect(img)
            for res in get:
                name = res[1]
                self.hmi.info(f"正在识别, 请保持目标在视野内\n识别到目标: {name}")
                rec_dict[name] = rec_dict.get(name, 0) + 1
                if not start:
                    start = True
                last_scan_time = time()
            if start and time() - last_scan_time > 2:
                break
        max_idx = max(rec_dict, key=rec_dict.get)
        logger.info(f"[MISSION] Recognized target: {max_idx}")
        self.hmi.info(f"已设定目标: {max_idx}\n等待一键启动")
        self.fc.set_rgb_led(0, 255, 0)
        if max_idx == "r_rec":
            target_points = RED_RECTANGLES
        elif max_idx == "b_rec":
            target_points = BLUE_RECTANGLES
        elif max_idx == "r_tri":
            target_points = RED_TRIANGLES
        elif max_idx == "b_tri":
            target_points = BLUE_TRIANGLES
        elif max_idx == "r_cir":
            target_points = RED_CIRCLES
        elif max_idx == "b_cir":
            target_points = BLUE_CIRCLES
        else:
            raise Exception(f"[MISSION] Unknown target: {max_idx}")
        logger.info("[MISSION] Set target points: {}".format(target_points))
        self.fc.event.key_short.wait_clear()
        self.hmi.info("任务开始, 请远离飞行器")
