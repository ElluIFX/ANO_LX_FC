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

fc = FC_Client()
while True:
    try:
        fc.connect()
        break
    except:
        logger.warning("Connection failed, retrying...")
        sleep(1)
fc.start_sync_state(False)
radar = LD_Radar()
radar.start("/dev/ttyUSB0", "LD06")
cam = cv2.VideoCapture(0)
while not cam.isOpened():
    cam.open(0)
    if not cam.isOpened():
        raise Exception("Camera open failed")

############################## 参数 ##############################
camera_down_pwm = 32.5
camera_down_45_pwm = 52.25
camera_up_pwm = 72
camera_up_45_pwm = 91.75
set_button_led = lambda x: fc.set_digital_output(1, x)
set_buzzer = lambda x: fc.set_digital_output(0, x)
############################## 初始化 ##############################

fc.wait_for_connection()
fc.set_action_log(False)

fc.set_PWM_output(0, camera_up_pwm)

fc.set_flight_mode(fc.PROGRAM_MODE)

target_mission = None
_light_cnt = 0

logger.info("[MISSION] Waiting for input")
fc.event.key_short.clear()
fc.event.key_double.clear()
fc.event.key_long.clear()
while target_mission is None:
    sleep(0.1)
    _light_cnt += 1
    set_button_led(_light_cnt % 2 == 0)
    if fc.event.key_short.is_set():
        fc.event.key_short.clear()
        target_mission = 1
    elif fc.event.key_double.is_set():
        fc.event.key_double.clear()
        target_mission = 2
    elif fc.event.key_long.is_set():
        fc.event.key_long.clear()
        target_mission = 3
logger.info(f"[MISSION] Target Mission: {target_mission}")
set_button_led(False)
sleep(1)
fc.set_rgb_led(
    (target_mission == 1) * 255,
    (target_mission == 2) * 255,
    (target_mission == 3) * 255,
)
for i in range(10):
    sleep(0.1)
    set_buzzer(True)
    sleep(0.1)
    set_buzzer(False)
fc.set_rgb_led(0, 0, 0)

############################## 开始任务 ##############################
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

# sleep(1)
# logger.info("[MISSION] Manager Restarting")
# os.execl(sys.executable, sys.executable, *sys.argv)
