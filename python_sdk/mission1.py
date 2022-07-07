import threading
from time import sleep, time
import numpy as np

from FlightController import FC_Client, FC_Controller, logger
from FlightController.Components import LD_Radar, Map_360, Point_2D
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *
from simple_pid import PID


class Mission(object):
    def __init__(self, fc: FC_Controller, radar: LD_Radar):
        self.fc = fc
        self.radar = radar
        ############### PID #################
        self.height_pid = PID(0.4, 0.0, 0.05, setpoint=100, output_limits=(-30, 30))
        self.height_pid.set_auto_mode(False)
        #####################################
        self.keep_height_flag = False
        self.running = False
        self.thread_list = []

    def run(self):
        ############### 参数 #################
        camera_down_pwm = 32.5
        camera_down_45_pwm = 52.25
        camera_up_pwm = 72
        camera_up_45_pwm = 78.5
        radar_pwm = 50
        set_buzzer = lambda x: self.fc.set_digital_output(0, x)
        space_distance = 60
        yaw_zero_k = 0.064
        dis_threshold = 3
        theta_threshold = 0.1
        bar_x_threshold = 3
        rod_x_threshold = 3
        initial_yaw = self.fc.state.yaw
        arround_yaw_threshold = 0.2
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
        fc.set_PWM_output(1, 40)
        fc.set_flight_mode(fc.PROGRAM_MODE)
        ################ 初始化完成 ################

        logger.info("[MISSION] Mission Started")
        fc.unlock()
        sleep(2)
        fc.take_off()
        fc.wait_for_takeoff_done()
        ######## 已起飞
        radar.start_find_point(2.5, 1, -60, 60, 3, 2000)
        fc.set_flight_mode(fc.HOLD_POS_MODE)
        self.keep_height_flag = True
        fc.start_realtime_control()
        ######## 闭环定高
        def deg_360_180(deg):
            if deg > 180:
                deg = deg - 360
            return deg

        while True:
            sleep(0.01)
            if self.radar.fp_timeout_flag:
                self.fc.update_realtime_control(vel_x=0, vel_y=0)
            if len(self.radar.fp_points) > 0:
                self.radar.fp_points.sort(key=lambda x: abs(deg_360_180(x.degree)))
                deg = self.radar.fp_points[0].degree
                deg = deg_360_180(deg)
                dis = self.radar.fp_points[0].distance / 10
                logger.info("[MISSION] Find point: %.2f, %.2f" % (deg, dis))
                if dis < 100:
                    self.radar.update_find_point_args(2.5, 1, -45, 45, 1, 1200)
                ok = 0
                if deg > 2:
                    fc.update_realtime_control(vel_y=-8)
                elif deg < -2:
                    fc.update_realtime_control(vel_y=8)
                else:
                    fc.update_realtime_control(vel_y=0)
                    ok += 1
                if dis > space_distance + 5:
                    fc.update_realtime_control(vel_x=10)
                elif dis < space_distance - 5:
                    fc.update_realtime_control(vel_x=-10)
                else:
                    fc.update_realtime_control(vel_x=0)
                    ok += 1
                if ok == 2:
                    fc.update_realtime_control(vel_x=0, vel_y=0)
                    break
        set_buzzer(True)
        sleep(1)
        set_buzzer(False)
        ######## 到达目标点
        radar.stop_find_point()
        get_picture = False
        while True:
            sleep(0.01)
            ret, frame = cam.read()
            if not ret:
                logger.warning("[PICTURE] frame drop ")
                continue
            # TODO: 相机角度还未调整
            line_is_find, line_x_offset, line_y_offset, line_t_offset = black_line(frame, 0)
            if line_is_find:
                if line_y_offset > dis_threshold:
                    # 远离黑线
                    fc.update_realtime_control(vel_x = 5)
                elif line_y_offset < -dis_threshold:
                    # 靠近黑线
                    fc.update_realtime_control(vel_x = -5)
                else:
                    fc.update_realtime_control(vel_x = 0)

                if (line_t_offset - yaw_zero_k) > theta_threshold:
                    # yaw轴向左偏
                    fc.update_realtime_control(yaw = 5)
                elif (line_t_offset - yaw_zero_k) < -theta_threshold:
                    # yaw轴向右偏
                    fc.update_realtime_control(yaw = -5)
                else:
                    fc.update_realtime_control(yaw = 0)
            if not get_picture:
                bar_is_find, bar_x_offset, bar_y_offset = find_yellow_code(frame)
                if bar_is_find:
                    if bar_x_offset > bar_x_threshold:
                        # 还未抵达条形码处
                        fc.update_realtime_control(vel_y = 5)
                        continue
                    elif bar_x_offset < -bar_x_threshold:
                        # 超出条形码处
                        fc.update_realtime_control(vel_y = -5)
                        continue
                    else:
                        fc.update_realtime_control(0, 0, 0, 0)
                        logger.info("[MISSION] get yellow bar: %.2f" % (bar_x_offset))
                        change_cam_resolution(cam, 1920, 1080)
                        # TODO: 拍照+储存
                        ret, frame = cam.read()
                        if ret:
                            cv2.imwrite('yellow_bar.jpg',frame)
                            get_picture = True
                            change_cam_resolution(cam, 800, 600)
                            continue
                        else:
                            logger.debug("[PICTURE] take picture is failed")
                        ####### 成功拍到条形码
            if get_picture:
                line_is_find, line_x_offset, line_y_offset, line_t_offset = black_line(frame, 1)
                if line_is_find:
                    if line_x_offset > rod_x_threshold:
                        # 还未抵达第二跟杆
                        fc.update_realtime_control(vel_y = 5)
                        continue
                    elif line_x_offset < -rod_x_threshold:
                        # 超过第二根杆
                        fc.update_realtime_control(vel_y = -5)
                        continue
                    else:
                        fc.update_realtime_control(0, 0, 0, 0)
                        logger.info("[MISSION] get the second rod: %.2f" % line_x_offset)
                        break
            fc.update_realtime_control(vel_y = 10)
        set_buzzer(True)
        sleep(1)
        set_buzzer(False)
        ####### 到达第二根杆
        radar.start_find_point(2.5, 1, -30, 30, 3, 2000)
        yaw_pid = PID(0.4, 0.0, 0.05, setpoint=0, output_limits=(-10, 10))
        yaw_pid.set_auto_mode(False)
        dis_pid = PID(0.4, 0.0, 0.05, setpoint=60, output_limits=(-10, 10))
        dis_pid.set_auto_mode(False)
        get_picture = False
        while True:
            sleep(0.01)
            ret, frame = cam.read()
            if not ret:
                logger.warning("[PICTURE] frame drop ")
                continue
            # TODO: 相机角度还未调整
            line_is_find, line_x_offset, line_y_offset, line_t_offset = black_line(frame, 1)
            out_yaw = yaw_pid(line_x_offset)
            # line_x_offset > 0 -> 向右转    line_x_offset < 0 -> 向左转
            fc.update_realtime_control(yaw = -out_yaw)

            if self.radar.fp_timeout_flag:
                self.fc.update_realtime_control(vel_x=0, vel_y=0)
            if len(self.radar.fp_points) > 0:
                self.radar.fp_points.sort(key=lambda x: abs(deg_360_180(x.degree)))
                deg = self.radar.fp_points[0].degree
                deg = deg_360_180(deg)
                dis = self.radar.fp_points[0].distance / 10
                logger.info("[MISSION] Find point-2: %.2f, %.2f" % (deg, dis))
                out_dis = dis_pid(dis)
                # dis > 60 -> 向前走   dis < 60 -> 向后走
                fc.update_realtime_control(vel_x = -out_dis)
            else:
                # 没有雷达点，x速度置为零
                fc.update_realtime_control(vel_x = 0)
            if not get_picture:
                QR_is_find, QR_x_offset, QR_y_offset = find_QRcode_zbar()
                if QR_is_find:
                    if QR_x_offset > bar_x_threshold :
                        fc.update_realtime_control(vel_y = 5)
                    elif QR_x_offset < -bar_x_threshold:
                        fc.update_realtime_control(vel_y = -5)
                    else:
                        fc.update_realtime_control(0, 0, 0, 0)
                        logger.info("[MISSION] get QRcode : %.2f" % (QR_x_offset))
                        change_cam_resolution(cam, 1920, 1080)
                        # TODO: 拍照+储存
                        ret, frame = cam.read()
                        if ret:
                            cv2.imwrite('QRcode.jpg',frame)
                            get_picture = True
                            change_cam_resolution(cam, 800, 600)
                            continue
                        else:
                            logger.debug("[PICTURE] take picture is failed")
            # 判断绕杆结束,将yaw轴保持在初始状态的-180度
            if abs(abs(fc.state.yaw - initial_yaw) - np.pi()) < arround_yaw_threshold :
                fc.update_realtime_control(0, 0, 0, 0)
                logger.info("[MISSION] Completed around the rod ")
                break
            fc.update_realtime_control(vel_y = 10)
        set_buzzer(True)
        sleep(1)
        set_buzzer(False)        
        ####### 完成绕杆
        while True:
            sleep(0.01)
            ret, frame = cam.read()
            if not ret:
                logger.warning("[PICTURE] frame drop ")
                continue
            # TODO: 相机角度还未调整
            line_is_find, line_x_offset, line_y_offset, line_t_offset = black_line(frame, 1)
            if line_is_find:
                # 直接开环向右移动，超过第一根杆后悬停
                if line_x_offset < -10:
                    logger.info("[MISSION] Mission1 completed. Ready to land ")
                    fc.update_realtime_control(0, 0, 0, 0)
                    break
            fc.update_realtime_control(vel_y = 15)
        set_buzzer(True)
        sleep(1)
        set_buzzer(False) 
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
