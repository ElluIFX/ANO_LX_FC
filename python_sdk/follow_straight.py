import time

import cv2
import numpy as np
from simple_pid import PID

from FlightController import FlightController
from HoughTest import process as hough_process
from logger import logger

cam = cv2.VideoCapture(0)
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cam.set(cv2.CAP_PROP_FPS, 30)
print(
    f"cam info: {cam.get(cv2.CAP_PROP_FRAME_WIDTH)} x {cam.get(cv2.CAP_PROP_FRAME_HEIGHT)} @ {cam.get(cv2.CAP_PROP_FPS)} fps"
)

fc = FlightController("/dev/serial0")
fc.start_listen_serial(callback=fc.show_state())
fc.wait_for_connect(100)
while not fc.drone_state["rc_mask"]:
    logger.info("waiting for RC")
    time.sleep(1)
fc.rt_stop()
time.sleep(0.1)
fc.emergency()
time.sleep(0.1)
logger.info("Takeoff")
fc.takeoff()
while not fc.drone_state["flying"]:
    time.sleep(1)
logger.info("Hovering")
time.sleep(2)
fc.rt_set_Height(100)
time.sleep(2)
fc.rt_set_Height(100)
time.sleep(2)
logger.info("Go")
fc.flow_go_X(120, 15)
while fc.drone_state["pc_stat"]:
    time.sleep(1)
logger.info("Landing")
fc.rt_stop()
time.sleep(2)
fc.rt_set_Height(50)
time.sleep(0.5)
fc.land()
while not fc.drone_state["flying"]:
    time.sleep(1)
logger.info("landed")

# x_pid = PID(0.15, 0, 0.02, setpoint=0)
# deg_pid = PID(0.5, 0, 0.01, setpoint=0)
# x_pid.output_limits = (-10, 10)
# deg_pid.output_limits = (-30, 30)
# speed = 20
#
# state = 0
#
# logger.info("start")

# while True:
#     ret, frame = cam.read()
#     if frame is None:
#         logger.warning("frame is None")
#         continue
#     window = int((frame.shape[1] - frame.shape[0]) / 2)
#     frame = frame[:, window : frame.shape[1] - window]
#     x_offset, y_offset, deg_offset = hough_process(frame)
#     if x_offset is None:
#         logger.warning("x_offset is None")
#         continue
#     x_output = x_pid(x_offset)
#     deg_output = deg_pid(deg_offset)
#     fc.rt_set_Yaw_speed(deg_output)
#     if state == 0 and abs(deg_offset) < 1:
#         logger.info("state 0 -> 1")
#         state = 1
#     if state == 1:
#         fc.rt_set_XY_speed(0, x_output)
#     if state == 1 and abs(x_offset) < 15:
#         logger.info("state 1 -> 2")
#         state = 2
#     if state == 2:
#         fc.rt_set_XY_speed(speed, x_output)
#     if y_offset is not None:
#         if abs(y_offset) < 4:
#             logger.info("state stop")
#             break
#     key = cv2.waitKey(1)
#     if key == ord("q"):
#         break
#
# fc.rt_stop()
# time.sleep(2)
# fc.rt_set_Height(50)
# time.sleep(0.5)
# fc.land()
# while not fc.drone_state["flying"]:
#     time.sleep(1)
# logger.info("landed")
#
# time.sleep(10)
#
# time.sleep(0.1)
# logger.info("Takeoff")
# fc.rt_stop()
# fc.takeoff()
# while not fc.drone_state["flying"]:
#     time.sleep(1)
# time.sleep(2)
# fc.rt_set_Height(80)
# time.sleep(4)
# logger.info("Hovering")
# fc.flow_turn(180, 30)
# while fc.drone_state["pc_stat"]:
#     time.sleep(1)
# logger.info("done")
# state = 0
#
# logger.info("start")
# while True:
#     ret, frame = cam.read()
#     if frame is None:
#         logger.warning("frame is None")
#         continue
#     window = int((frame.shape[1] - frame.shape[0]) / 2)
#     frame = frame[:, window : frame.shape[1] - window]
#     x_offset, y_offset, deg_offset = hough_process(frame)
#     if x_offset is None:
#         logger.warning("x_offset is None")
#         continue
#     x_output = x_pid(x_offset)
#     deg_output = deg_pid(deg_offset)
#     fc.rt_set_Yaw_speed(deg_output)
#     if state == 0 and abs(deg_offset) < 1:
#         logger.info("state 0 -> 1")
#         state = 1
#     if state == 1:
#         fc.rt_set_XY_speed(0, x_output)
#     if state == 1 and abs(x_offset) < 10:
#         logger.info("state 1 -> 2")
#         state = 2
#     if state == 2:
#         fc.rt_set_XY_speed(speed, x_output)
#     if y_offset is not None:
#         if abs(y_offset) < 10:
#             logger.info("state stop")
#             break
#     key = cv2.waitKey(1)
#     if key == ord("q"):
#         break
# fc.rt_stop()
# time.sleep(2)
# fc.rt_set_Height(50)
# time.sleep(0.5)
# fc.land()
# while not fc.drone_state["flying"]:
#     time.sleep(1)
# logger.info("landed")

fc.quit()
cv2.destroyAllWindows()
