from time import sleep, time

from FlightController import FC_Client, FC_Controller, logger

fc = FC_Client()
fc.connect()
fc.start_sync_state()

############################## 参数 ##############################
camera_down_pwm = 32.5
camera_down_45_pwm = 52.25
camera_up_pwm = 72
camera_up_45_pwm = 91.75
set_button_led = lambda x: fc.set_digital_output(1, x)
set_buzzer = lambda x: fc.set_digital_output(0, x)
############################## 初始化 ##############################

fc.wait_for_connection()
fc.settings.action_log_output = False

fc.set_PWM_output(0, camera_up_pwm)
fc.set_PWM_output(1, 40)

fc.set_flight_mode(fc.PROGRAM_MODE)

target_mission = 1
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
logger.info(f"[MISSION] target_mission: {target_mission}")
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
try:
    if target_mission == 1:
        logger.info("[MISSION] Calling Mission 1")
        from mission1 import mission

        mission(fc)
    elif target_mission == 2:
        logger.info("[MISSION] Calling Mission 2")
        from mission2 import mission

        mission(fc)
    elif target_mission == 3:
        logger.info("[MISSION] Calling Mission 3")
        from mission3 import mission

        mission(fc)

    logger.info("[MISSION] Mission Finished")
except Exception as e:
    logger.error(f"[MISSION] Mission Failed: {e}")
finally:
    if fc.state.unlock.value:
        logger.warning("[MISSION] Auto Landing")
        fc.set_flight_mode(fc.PROGRAM_MODE)
        fc.stablize()
        fc.land()
        ret = fc.wait_for_lock()
        if not ret:
            fc.lock()

############################## 结束任务 ##############################

fc.set_PWM_output(1, 0)
fc.set_rgb_led(0, 255, 0)
set_buzzer(True)
sleep(1)
set_buzzer(False)
fc.set_rgb_led(0, 0, 0)
fc.quit()
