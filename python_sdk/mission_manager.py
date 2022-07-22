import os
import sys

####### 清理日志 #######
path = os.path.dirname(os.path.abspath(__file__))
log_path = os.path.join(path, "fc_log.log")
if os.path.exists(log_path):
    try:
        os.remove(log_path)
    except:
        pass
####################
from time import sleep, time

import cv2
from FlightController import FC_Client, FC_Controller, logger
from FlightController.Components import LD_Radar, Map_360, Point_2D


def self_reboot():
    logger.info("[MISSION] Manager Restarting")
    os.execl(sys.executable, sys.executable, *sys.argv)


try:
    fc = FC_Client()
    fc.connect()
    fc.start_sync_state(False)
except:
    logger.warning("[MISSION] Manager Connecting Failed, Switching to Local Mode")
    try:
        fc = FC_Controller()
        fc.start_listen_serial("/dev/serial0", print_state=False)
    except:
        logger.error("[MISSION] Local Mode Failed, Restarting")
        sleep(1)
        self_reboot()

fc.event.key_short.clear()
fc.event.key_double.clear()
fc.event.key_long.clear()
fc.set_rgb_led(0, 0, 0)

try:
    radar = LD_Radar()
    radar.start("/dev/ttyUSB0", "LD06")
except:
    logger.warning("[MISSION] Radar Connecting Failed")
    fc.set_rgb_led(255, 0, 0)
    while True:
        sleep(1)
        if fc.event.key_short.is_set():
            self_reboot()
try:
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        cam.open(0)
    assert cam.isOpened()
except:
    logger.warning("[MISSION] Camera Opening Failed")
    fc.set_rgb_led(255, 255, 0)
    while True:
        sleep(1)
        if fc.event.key_short.is_set():
            self_reboot()

############################## 参数 ##############################
camera_down_pwm = 32.5
camera_down_45_pwm = 52.25
camera_up_pwm = 72
camera_up_45_pwm = 91.75
set_button_led = lambda x: fc.set_digital_output(1, x)
set_buzzer = lambda x: fc.set_digital_output(0, x)
############################## 初始化 ##############################
logger.info("[MISSION] Self-Checking Passed")
fc.set_rgb_led(0, 255, 0)
sleep(0.7)
fc.set_rgb_led(0, 0, 0)

fc.wait_for_connection()
fc.set_action_log(False)

fc.set_PWM_output(0, camera_up_pwm)
fc.set_rgb_led(0, 0, 0)
fc.set_flight_mode(fc.PROGRAM_MODE)

target_mission = None
total_mission = 3
_light_cnt = 0

logger.info("[MISSION] Selecting mission...")
if target_mission is None:
    target_mission = 1
    while True:
        sleep(0.1)
        _light_cnt += 1
        set_button_led(_light_cnt % 2 == 0)
        if fc.event.key_short.is_set():
            fc.event.key_short.clear()
            target_mission += 1
            target_mission %= total_mission + 1
            if target_mission == 0:
                target_mission = 1
            for i in range(target_mission):
                fc.set_rgb_led(0, 0, 255)
                sleep(0.15)
                fc.set_rgb_led(0, 0, 0)
                sleep(0.15)
        elif fc.event.key_long.is_set():
            fc.event.key_long.clear()
            for i in range(target_mission):
                fc.set_rgb_led(255, 255, 255)
                sleep(0.2)
                fc.set_rgb_led(0, 0, 0)
                sleep(0.2)
            break
set_button_led(False)
############################## 开始任务 ##############################
logger.info(f"[MISSION] Target Mission: {target_mission}")
fc.set_action_log(True)
mission = None
try:
    if target_mission == 1:
        from mission1 import Mission

        mission = Mission(fc, radar, cam)
    elif target_mission == 2:
        from mission2 import Mission

        mission = Mission(fc, radar, cam)
    elif target_mission == 3:
        from mission3 import Mission

        mission = Mission(fc, radar, cam)
    logger.info("[MISSION] Calling Mission")

    mission.run()

    logger.info("[MISSION] Mission Finished")
except Exception as e:
    import traceback

    logger.error(f"[MISSION] Mission Failed: {traceback.format_exc()}")
finally:
    if mission is not None:
        mission.stop()
    if fc.state.unlock.value:
        logger.warning("[MISSION] Auto Landing")
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.stablize()
        fc.land()
        ret = fc.wait_for_lock()
        if not ret:
            fc.lock()

############################## 结束任务 ##############################
fc.set_action_log(False)
fc.set_PWM_output(1, 0)
fc.set_rgb_led(0, 255, 0)
set_buzzer(True)
sleep(0.5)
set_buzzer(False)
fc.set_rgb_led(0, 0, 0)
fc.quit()
cam.release()

########################## 重启自身 #############################
