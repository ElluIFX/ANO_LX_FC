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


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar):
        self.fc = fc
        self.radar = radar
        ############### PID #################
        self.height_pid = PID(0.8, 0.0, 0.1, setpoint=86, output_limits=(-30, 30))  # 86
        self.height_pid.set_auto_mode(False)
        #####################################
        self.keep_height_flag = False
        self.running = False
        self.thread_list = []
        # vision_debug()

    def run(self):
        ############### 参数 #################
        camera_down_pwm = 32.5
        camera_up_pwm = 72
        camera_up_45_pwm = 79  # 78.5
        set_buzzer = lambda x: self.fc.set_digital_output(0, x)
        space_distance = 52
        yaw_zero_k = 0.064
        dis_threshold = 20
        theta_threshold = 0.1
        yaw_threshold = 1
        bar_x_threshold = 60
        qr_y_threshold = 10
        rod_x_threshold = 40
        initial_yaw = self.fc.state.yaw.value
        arround_yaw_threshold = 25
        line_y_target = -20
        line_y_target_2nd = -10
        ################ 启动线程 ################
        self.running = True
        self.thread_list.append(
            threading.Thread(target=self.keep_height_task, daemon=True)
        )
        self.thread_list[-1].start()
        logger.info("[MISSION] Threads started")
        ################ 初始化 ################
        fc = self.fc
        radar = self.radar
        cam = cv2.VideoCapture(0)
        change_cam_resolution(cam, 800, 600)
        fc.set_PWM_output(0, camera_up_pwm)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.set_digital_output(2, 1)
        ################ 初始化完成 ################

        logger.info("[MISSION] Mission Started")
        fc.unlock()
        sleep(2)
        fc.take_off(60)
        fc.wait_for_takeoff_done()
        ######## 已起飞
        radar.start_find_point(2.5, 0, -30, 30, 1, 1600)
        fc.set_flight_mode(fc.HOLD_POS_MODE)
        self.keep_height_flag = True
        fc.start_realtime_control()
        ######## 闭环定高
        y_locked = False
        x_locked = False
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
                    self.radar.update_find_point_args(-30, 30, 1, 1200)
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
                    break
        fc.set_PWM_output(0, camera_up_45_pwm)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.horizontal_move(20, 10, 90)
        fc.wait_for_last_command_done()
        fc.set_flight_mode(fc.HOLD_POS_MODE)
        sleep(1)
        ######## 到达目标点
        radar.stop_find_point()
        get_picture = False
        warning_time = None
        warned = False
        start_time = time()
        line_pid = PID(
            0.1, 0.002, 0.02, setpoint=line_y_target, output_limits=(-20, 20)
        )
        while True:
            cv2.waitKey(1)
            ret, frame = cam.read()
            if not ret:
                logger.warning("[PICTURE] frame drop ")
                continue
            cv2.imshow("Origin", frame)
            line_is_find, _, line_y_offset, line_t_offset = black_line(frame, 0)
            logger.debug(f"[VISION] {line_is_find}, {line_y_offset}, {line_t_offset}")
            if line_is_find:
                pid_out = line_pid(line_y_offset)
                fc.update_realtime_control(vel_x=-pid_out)
                ######
                # if line_y_offset - line_y_target > dis_threshold:
                #     # 远离黑线
                #     fc.update_realtime_control(vel_x=8)
                # elif line_y_offset - line_y_target < -dis_threshold:
                #     # 靠近黑线
                #     fc.update_realtime_control(vel_x=-8)
                # else:
                #     fc.update_realtime_control(vel_x=0)
                #######
                # if (line_t_offset - yaw_zero_k) > theta_threshold:
                #     # yaw轴向左偏
                #     fc.update_realtime_control(yaw=5)
                # elif (line_t_offset - yaw_zero_k) < -theta_threshold:
                #     # yaw轴向右偏
                #     fc.update_realtime_control(yaw=-5)
                # else:
                #     fc.update_realtime_control(yaw=0)
                if (self.fc.state.yaw.value - initial_yaw) < -yaw_threshold:
                    # yaw轴向左偏
                    fc.update_realtime_control(yaw=2)
                elif (self.fc.state.yaw.value - initial_yaw) > yaw_threshold:
                    # yaw轴向右偏
                    fc.update_realtime_control(yaw=-2)
                else:
                    fc.update_realtime_control(yaw=0)
            if not get_picture and time() - start_time > 3:
                bar_frame = get_ROI(frame, (0, 0.35, 1, 0.3))
                bar_is_find, bar_x_offset, bar_y_offset = find_yellow_code(bar_frame)
                if bar_is_find:
                    if not warned:
                        fc.set_rgb_led(255, 255, 0)
                        set_buzzer(True)
                        warning_time = time()
                        warned = True
                    if bar_x_offset > bar_x_threshold:
                        # 还未抵达条形码处
                        fc.update_realtime_control(vel_y=-4)
                    elif bar_x_offset < -bar_x_threshold:
                        # 超出条形码处
                        fc.update_realtime_control(vel_y=4)
                    else:
                        fc.update_realtime_control(vel_y=0, vel_x=0, yaw=0)
                        logger.info("[MISSION] get yellow bar: %.2f" % (bar_x_offset))
                        change_cam_resolution(cam, 1920, 1080)
                        ret, frame = cam.read()
                        if ret:
                            cv2.imwrite(
                                "/home/pi/Desktop/prj/python_sdk/yellow_bar.png",
                                frame,
                            )
                            logger.info("[MISSION] yellow bar picture saved")
                            get_picture = True
                            change_cam_resolution(cam, 800, 600)
                        else:
                            logger.debug("[PICTURE] take picture is failed")
                    continue
            if warning_time is not None:
                if time() - warning_time > 5:
                    set_buzzer(False)
                    fc.set_rgb_led(0, 0, 0)
                    warning_time = None
            if get_picture:
                line_is_find, line_x_offset, _, line_t_offset = black_line(frame, 1)
                if line_is_find and abs(line_x_offset) < 300:
                    if line_x_offset - 200 > rod_x_threshold:
                        # 还未抵达第二跟杆
                        fc.update_realtime_control(vel_y=-5)
                    elif line_x_offset - 200 < -rod_x_threshold:
                        # 超过第二根杆
                        fc.update_realtime_control(vel_y=5)
                    else:
                        fc.update_realtime_control(0, 0, 0, 0)
                        logger.info(
                            "[MISSION] get the second rod: %.2f" % line_x_offset
                        )
                        break
                    continue
            fc.update_realtime_control(vel_y=-10)  # 默认向前
        fc.set_PWM_output(0, camera_up_pwm)
        sleep(2)
        ####### 到达第二根杆
        get_picture = False
        start_time = time()
        self.keep_height_flag = False  # 暂停高度控制
        radar.start_find_point(2.5, 0, -45, 45, 1, 1600)
        y_locked = False
        x_locked = False
        while True:
            cv2.waitKey(1)
            if self.radar.fp_timeout_flag:
                self.fc.update_realtime_control(vel_x=0, vel_y=0)
            if len(self.radar.fp_points) > 0:
                self.radar.fp_points.sort(key=lambda x: abs(deg_360_180(x.degree)))
                deg = self.radar.fp_points[0].degree
                deg = deg_360_180(deg)
                dis = self.radar.fp_points[0].distance / 10
                logger.info("[MISSION] find point 2: %.2f, %.2f" % (deg, dis))
                if deg > 1:
                    fc.update_realtime_control(vel_y=-6)
                elif deg < -1:
                    fc.update_realtime_control(vel_y=6)
                else:
                    fc.update_realtime_control(vel_y=0)
                    self.radar.update_find_point_args(-30, 30, 1, 1200)
                    y_locked = True
                if y_locked:
                    if dis > space_distance + 5:
                        fc.update_realtime_control(vel_x=4)
                    elif dis < space_distance - 5:
                        fc.update_realtime_control(vel_x=-4)
                    else:
                        fc.update_realtime_control(vel_x=0)
                        x_locked = True
                if x_locked and y_locked:
                    fc.update_realtime_control(vel_x=0, vel_y=0)
                    ret, frame = cam.read()
                    if not ret:
                        continue
                    cv2.imshow("Origin", frame)
                    QR_is_find, _, QR_y_offset, _ = find_QRcode_zbar(frame)
                    logger.info(f"[MISSION] QR offset: {QR_y_offset}")
                    if QR_y_offset > qr_y_threshold:
                        fc.update_realtime_control(vel_z=-5)
                    elif QR_y_offset < -qr_y_threshold:
                        fc.update_realtime_control(vel_z=5)
                    else:
                        fc.update_realtime_control(vel_z=0)
                        logger.info("[MISSION] get QRcode")
                        change_cam_resolution(cam, 1920, 1080)
                        ret, frame = cam.read()
                        if ret:
                            cv2.imwrite(
                                "/home/pi/Desktop/prj/python_sdk/QRcode.png", frame
                            )
                            logger.info("[MISSION] QRcode picture saved")
                            get_picture = True
                            change_cam_resolution(cam, 800, 600)
                        else:
                            logger.debug("[PICTURE] take picture is failed")
                    if get_picture or time() - start_time > 10:
                        break
        self.keep_height_flag = True
        sleep(1)
        ####### 开始绕杆
        # yaw_pid = PID(0.3, 0.0, 0.1, setpoint=-30, output_limits=(-15, 15))
        yaw_pid = PID(0.6, 0.01, 0.1, setpoint=15, output_limits=(-20, 20))
        dis_pid = PID(0.8, 0.04, 0.05, setpoint=45, output_limits=(-40, 40))
        completed_time = None
        last_yaw = fc.state.yaw.value
        turn_count = 0
        while True:
            cv2.waitKey(1)
            ret, frame = cam.read()
            if not ret:
                logger.warning("[PICTURE] frame drop ")
                continue
            frame = get_ROI(frame, (0, 0.2, 1, 0.6))
            # 毙掉的视觉方案：会跟到后面的杆子
            # cv2.imshow("Origin", frame)
            # line_is_find, line_x_offset, _, _ = black_line(frame, 1)
            # out_yaw = yaw_pid(line_x_offset)
            # line_x_offset > 0 -> 向右转    line_x_offset < 0 -> 向左转
            # fc.update_realtime_control(yaw=-out_yaw)

            if self.radar.fp_timeout_flag:
                self.fc.update_realtime_control(vel_x=0, vel_y=0, yaw=0)
            if len(self.radar.fp_points) > 0:
                self.radar.fp_points.sort(key=lambda x: abs(deg_360_180(x.degree)))
                deg = self.radar.fp_points[0].degree
                deg = deg_360_180(deg)
                dis = self.radar.fp_points[0].distance / 10
                logger.info("[MISSION] find point 3: %.2f, %.2f" % (deg, dis))
                out_dis = dis_pid(dis)
                # dis > 60 -> 向前走   dis < 60 -> 向后走
                fc.update_realtime_control(vel_x=-out_dis)
                out_yaw = yaw_pid(deg)
                fc.update_realtime_control(yaw=-out_yaw)
            else:
                # 没有雷达点，x速度置为零
                fc.update_realtime_control(vel_x=0)
            # 判断绕杆结束,将yaw轴保持在初始状态的-180度
            current_yaw = fc.state.yaw.value
            diff = abs(current_yaw - last_yaw)
            last_yaw = current_yaw
            if diff > 180:
                diff = 360 - diff
            turn_count += diff
            if completed_time is not None:
                fc.update_realtime_control(vel_y=0)
                if time() - completed_time > 2:
                    break
                continue
            if 180 - turn_count < arround_yaw_threshold:
                logger.info("[MISSION] Completed around the rod ")
                completed_time = time()
                dis_pid.setpoint += 7
            fc.update_realtime_control(vel_y=-8)
        radar.stop_find_point()
        fc.update_realtime_control(0, 0, 0, 0)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        target = initial_yaw + 190
        if target > 180:
            target = target - 360
        fc.set_yaw(target, 20)
        fc.wait_for_last_command_done()
        ####### 完成绕杆
        fc.set_PWM_output(0, camera_up_45_pwm)
        fc.horizontal_move(40, 25, 90)
        fc.wait_for_last_command_done()
        fc.set_flight_mode(fc.HOLD_POS_MODE)
        radar.stop_find_point()
        ###
        initial_yaw_2nd = fc.state.yaw.value
        get_picture = False
        warning_time = None
        warned = False
        start_time = time()
        line_pid = PID(
            0.1, 0.002, 0.02, setpoint=line_y_target_2nd, output_limits=(-20, 20)
        )
        while True:
            cv2.waitKey(1)
            ret, frame = cam.read()
            if not ret:
                logger.warning("[PICTURE] frame drop ")
                continue
            cv2.imshow("Origin", frame)
            line_is_find, _, line_y_offset, line_t_offset = black_line(frame, 0)
            logger.debug(f"[VISION] {line_is_find}, {line_y_offset}, {line_t_offset}")
            if line_is_find:
                pid_out = line_pid(line_y_offset)
                fc.update_realtime_control(vel_x=-pid_out)
                ######
                # if line_y_offset - line_y_target > dis_threshold:
                #     # 远离黑线
                #     fc.update_realtime_control(vel_x=8)
                # elif line_y_offset - line_y_target < -dis_threshold:
                #     # 靠近黑线
                #     fc.update_realtime_control(vel_x=-8)
                # else:
                #     fc.update_realtime_control(vel_x=0)
                #######
                # if (line_t_offset - yaw_zero_k) > theta_threshold:
                #     # yaw轴向左偏
                #     fc.update_realtime_control(yaw=5)
                # elif (line_t_offset - yaw_zero_k) < -theta_threshold:
                #     # yaw轴向右偏
                #     fc.update_realtime_control(yaw=-5)
                # else:
                #     fc.update_realtime_control(yaw=0)
                #######
                if (self.fc.state.yaw.value - initial_yaw_2nd) < -yaw_threshold:
                    # yaw轴向左偏
                    fc.update_realtime_control(yaw=2)
                elif (self.fc.state.yaw.value - initial_yaw_2nd) > yaw_threshold:
                    # yaw轴向右偏
                    fc.update_realtime_control(yaw=-2)
                else:
                    fc.update_realtime_control(yaw=0)
            if not get_picture and time() - start_time > 6:
                bar_frame = get_ROI(frame, (0, 0.35, 1, 0.3))
                bar_is_find, bar_x_offset, bar_y_offset = find_yellow_code(bar_frame)
                if bar_is_find:
                    if not warned:
                        fc.set_rgb_led(255, 255, 0)
                        set_buzzer(True)
                        warning_time = time()
                        warned = True
                    if bar_x_offset > bar_x_threshold:
                        # 还未抵达条形码处
                        fc.update_realtime_control(vel_y=-4)
                    elif bar_x_offset < -bar_x_threshold:
                        # 超出条形码处
                        fc.update_realtime_control(vel_y=4)
                    else:
                        fc.update_realtime_control(vel_y=0, vel_x=0, yaw=0)
                        logger.info("[MISSION] get yellow bar: %.2f" % (bar_x_offset))
                        change_cam_resolution(cam, 1920, 1080)
                        ret, frame = cam.read()
                        if ret:
                            cv2.imwrite(
                                "/home/pi/Desktop/prj/python_sdk/yellow_bar_2.png",
                                frame,
                            )
                            logger.info("[MISSION] yellow bar 2 picture saved")
                            get_picture = True
                            change_cam_resolution(cam, 800, 600)
                        else:
                            logger.debug("[PICTURE] take picture is failed")
                    continue
            if warning_time is not None:
                if time() - warning_time > 5:
                    set_buzzer(False)
                    fc.set_rgb_led(0, 0, 0)
                    warning_time = None
            if get_picture:
                line_is_find, line_x_offset, _, line_t_offset = black_line(frame, 1)
                if line_is_find and abs(line_x_offset) < 300:
                    if line_x_offset - 200 > rod_x_threshold:
                        # 还未抵达第二跟杆
                        fc.update_realtime_control(vel_y=-5)
                    elif line_x_offset - 200 < -rod_x_threshold:
                        # 超过第二根杆
                        fc.update_realtime_control(vel_y=5)
                    else:
                        fc.update_realtime_control(0, 0, 0, 0)
                        logger.info(
                            "[MISSION] get the second rod: %.2f" % line_x_offset
                        )
                        break
                    continue
            fc.update_realtime_control(vel_y=-10)  # 默认向前
        fc.set_PWM_output(0, camera_up_pwm)
        ###
        fc.update_realtime_control(0, 0, 0, 0)
        fc.set_PWM_output(0, camera_up_pwm)
        sleep(2)
        # 到达第二杆
        radar.start_find_point(2.5, 0, -45, 45, 1, 1600)
        y_locked = False
        x_locked = False
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
                    self.radar.update_find_point_args(-30, 30, 1, 1200)
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
                    break
        ####### 任务1完成

    def stop(self):
        self.running = False
        self.fc.stop_realtime_control()

    def keep_height_task(self):
        paused = False
        while self.running:
            sleep(0.1)
            if (
                self.keep_height_flag
                and self.fc.state.mode.value == self.fc.HOLD_POS_MODE
            ):
                if paused:
                    paused = False
                    self.height_pid.set_auto_mode(True, last_output=0)
                    logger.info("[MISSION] Keep Height resumed")
                out = self.height_pid(self.fc.state.alt_add.value)
                self.fc.update_realtime_control(vel_z=out)
            else:
                if not paused:
                    paused = True
                    self.height_pid.set_auto_mode(False)
                    self.fc.update_realtime_control(vel_z=0)
                    logger.info("[MISSION] Keep height paused")
