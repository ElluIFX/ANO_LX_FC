import os
import sys

####### 清理日志 #######
# path = os.path.dirname(os.path.abspath(__file__))
# log_path = os.path.join(path, "fc_log.log")
# try:
#     os.remove(log_path)
# except:
#     pass
####################
from time import sleep, time

import cv2
import numpy as np
from configManager import ConfigManager
from FlightController import FC_Client, FC_Controller, logger
from FlightController.Components import LD_Radar, Map_360, Point_2D
from hmi import HMI


def self_reboot():
    logger.info("[MANAGER] Manager Restarting")
    os.execl(sys.executable, sys.executable, *sys.argv)


try:
    fc = FC_Client()
    fc.connect()
    fc.start_sync_state(False)
    fc.wait_for_connection(5)
except:
    logger.warning("[MANAGER] Manager Connecting Failed, Switching to Local Mode")
    try:
        fc = FC_Controller()
        fc.start_listen_serial("/dev/serial0", print_state=False)
        fc.wait_for_connection(5)
    except:
        logger.error("[MANAGER] Local Mode Failed, Restarting")
        sleep(1)
        self_reboot()

fc.event.key_short.clear()
fc.event.key_double.clear()
fc.event.key_long.clear()
fc.set_rgb_led(0, 0, 0)
fc.set_action_log(False)

try:
    radar = LD_Radar()
    radar.start(
        "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0",
        "LD06",
    )
except:
    logger.warning("[MANAGER] Radar Connecting Failed")
    while True:
        fc.set_rgb_led(255, 0, 0)
        sleep(0.5)
        fc.set_rgb_led(0, 0, 0)
        sleep(0.5)
        if fc.event.key_short.is_set():
            fc.quit()
            self_reboot()
try:
    cam = cv2.VideoCapture(0)
    if not cam.isOpened():
        cam.open(0)
    assert cam.isOpened()
except:
    logger.warning("[MANAGER] Camera Opening Failed")
    while True:
        fc.set_rgb_led(255, 255, 0)
        sleep(0.5)
        fc.set_rgb_led(0, 0, 0)
        sleep(0.5)
        if fc.event.key_short.is_set():
            fc.quit()
            radar.stop()
            self_reboot()

try:
    hmi = HMI("/dev/serial/by-id/usb-1a86_USB2.0-Serial-if00-port0")
    hmi.command("page init")
    sleep(1)
except:
    logger.warning("[MANAGER] HMI Connecting Failed")
    while True:
        fc.set_rgb_led(255, 0, 255)
        sleep(0.5)
        fc.set_rgb_led(0, 0, 0)
        sleep(0.5)
        if fc.event.key_short.is_set():
            fc.quit()
            cam.release()
            radar.stop()
            self_reboot()

############################## 参数 ##############################
cfg = ConfigManager()
camera_down_pwm = 32.5
camera_down_45_pwm = 52.25
camera_up_pwm = 72
camera_up_45_pwm = 91.75
set_button_led = lambda x: fc.set_digital_output(1, x)
set_buzzer = lambda x: fc.set_digital_output(0, x)
############################## 初始化 ##############################
logger.info("[MANAGER] Self-Checking Passed")
fc.set_rgb_led(0, 255, 0)
sleep(1)
fc.set_rgb_led(0, 0, 0)
hmi.command("ok.en=1")


fc.set_PWM_output(0, camera_up_pwm)
fc.set_rgb_led(0, 0, 0)
fc.set_flight_mode(fc.PROGRAM_MODE)
set_button_led(False)

target_mission = None
_testing = False

logger.info("[MANAGER] Selecting mission...")

pos_send_flag = False
last_send_time = time()
last_cnt_time = time()
if target_mission is None:
    while True:
        sleep(0.01)
        cmd = hmi.read()
        pos = radar.rt_pose
        if pos_send_flag and time() - last_send_time > 0.2:
            last_send_time = time()
            hmi.command(f'pos.txt="({pos[0]:06.2f}, {pos[1]:06.2f}, {pos[2]:05.1f})"')
        if time() - last_cnt_time > 1:
            last_cnt_time = time()
            hmi.command(f'cnt.txt="已连接 电压: {fc.state.bat.value:.2f}V"')
        if cmd is not None:
            if cmd == "ok":  # 握手
                hmi.command("ok.en=1")
                logger.info("[HMI] HMI connected")
            if cmd == "start_cal":
                pos_send_flag = True
                radar.start_resolve_pose(scale_ratio=1, low_pass_ratio=0.4)
                fc.set_digital_output(2, True)
            if cmd == "stop_cal":
                pos_send_flag = False
                radar.stop_resolve_pose()
                fc.set_digital_output(2, False)
            if cmd.startswith("cal-"):
                try:
                    target = f"point-{int(cmd.removeprefix('cal-'))}"
                    arr = np.array([pos[0], pos[1]])
                    cfg.set(target, arr)
                    logger.info(f"[HMI] Set calibration {target} to {arr}")
                    hmi.info(f"{target} 校正成功")
                except:
                    hmi.info(f"指令出错")
            if cmd.startswith("pos:"):
                try:
                    x1, y1, x2, y2 = cmd.strip().replace("pos:", "").split(",")
                    arr1 = np.array([int(x1), int(y1)])
                    arr2 = np.array([int(x2), int(y2)])
                    cfg.set("target-1", arr1)
                    cfg.set("target-2", arr2)
                    logger.info(f"[HMI] Set targets to {arr1} and {arr2}")
                    hmi.info(f"任务点已设置")
                except:
                    hmi.info(f"指令出错")
            if cmd.startswith("mission-"):
                try:
                    mission = int(cmd.strip().replace("mission-", ""))
                    logger.info(f"[HMI] Get mission {mission}")
                    hmi.info(f"任务 {mission} 已选择, 等待一键启动")
                    target_mission = mission
                    break
                except:
                    hmi.info(f"指令出错")
            if cmd.startswith("spec-"):
                try:
                    it = int(cmd.strip().replace("spec-", ""))
                    epoint = cfg.get_int("enter-point")
                    if it == 1:
                        epoint -= 1
                    elif it == 3:
                        epoint += 1
                    logger.info(f"[HMI] Set enter point {epoint}")
                    hmi.info(f"{epoint}")
                    cfg.set("enter-point", epoint)
                    sleep(0.5)
                    hmi.info("")
                except:
                    logger.warning(f"[HMI] Get enter point error")
else:
    _testing = True
if not _testing:
    set_button_led(True)
    fc.event.key_short.clear()
    while True:
        sleep(0.01)
        if fc.event.key_short.is_set():
            break
        cmd = hmi.read()
        if cmd is not None:
            if "home" in cmd:
                logger.info("[HMI] Cancel mission, auto rebooting...")
                fc.quit()
                cam.release()
                radar.stop()
                hmi.stop()
                self_reboot()
    fc.event.key_short.clear()
    set_button_led(False)
############################## 开始任务 ##############################
logger.info(f"[MANAGER] Target Mission: {target_mission}")
fc.set_action_log(True)
hmi.info(f"任务 {target_mission} 启动")
mission = None
try:
    if target_mission == 1:
        from mission1 import Mission

        mission = Mission(fc, radar, cam, hmi)
    elif target_mission == 2:
        from mission2 import Mission

        mission = Mission(fc, radar, cam, hmi)
    elif target_mission == 3:
        from mission3 import Mission

        mission = Mission(fc, radar, cam, hmi)
    logger.info("[MANAGER] Calling Mission")

    mission.run()

    logger.info("[MANAGER] Mission Finished")
except Exception as e:
    import traceback

    logger.error(f"[MANAGER] Mission Failed: {traceback.format_exc()}")
finally:
    if mission is not None:
        mission.stop()
    if fc.state.unlock.value:
        logger.warning("[MANAGER] Auto Landing")
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.stablize()
        fc.land()
        ret = fc.wait_for_lock()
        if not ret:
            fc.lock()

############################## 结束任务 ##############################
hmi.info(f"任务 {target_mission} 结束")
fc.set_action_log(False)
fc.set_PWM_output(1, 0)
fc.set_rgb_led(0, 255, 0)
set_buzzer(True)
sleep(0.5)
set_buzzer(False)
fc.set_rgb_led(0, 0, 0)
fc.quit()
cam.release()
radar.stop()
hmi.stop()

########################## 重启自身 #############################
if not _testing:
    self_reboot()
