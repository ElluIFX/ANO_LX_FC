import time
from typing import Literal

import cv2
import numpy as np
from FlightController.Components import LD_Radar
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *

radar = LD_Radar()
try:
    radar.start("COM5", "LD06")
except:
    radar.start("/dev/ttyUSB0", "LD06")

KERNAL_DI = 9
KERNAL_ER = 5
HOUGH_THRESHOLD = 120
MIN_LINE_LENGTH = 100

kernel_di = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNAL_DI, KERNAL_DI))
kernel_er = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNAL_ER, KERNAL_ER))

rt_pose = [0, 0, 0]  # x,y,yaw


def process(img):
    global rt_pose
    cv2.imshow("Origin", img)
    img = cv2.dilate(img, kernel_di)
    img = cv2.erode(img, kernel_er)
    cv2.imshow("Close", img)
    lines = cv2.HoughLinesP(
        img,
        1,
        np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=MIN_LINE_LENGTH,
        maxLineGap=200,
    )
    img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    size = img.shape
    # mid point
    x0, y0 = size[0] // 2, size[1] // 2  # 得到中点的整数坐标
    if lines is not None:
        print(f"Total linesP: {len(lines)}")
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if (
                x1 > x0 and x2 > x0 and ((y1 > y0 and y2 < y0) or (y1 < y0 and y2 > y0))
            ):  # 右侧线
                cv2.line(img, (x1, y1), (x2, y2), (255, 255, 0), 2)
                rt_pose[0], rt_pose[2] = get_point_line_distance(
                    x0, y0, x1, y1, x2, y2, 0
                )
                cv2.line(img, (x0, y0), (int(x0 + rt_pose[0]), y0), (0, 0, 255), 2)
            elif (
                y1 > y0 and y2 > y0 and ((x1 > x0 and x2 < x0) or (x1 < x0 and x2 > x0))
            ):  # 下侧线
                cv2.line(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
                rt_pose[1], rt_pose[2] = get_point_line_distance(
                    x0, y0, x1, y1, x2, y2, 1
                )
                cv2.line(img, (x0, y0), (x0, int(y0 + rt_pose[1])), (0, 0, 255), 2)

    cv2.imshow("HoughP", img)


def get_point_line_distance(x1, y1, x2, y2, x3, y3, type: Literal[0, 1] = 0):
    """
    计算点到线的距离
    :param x2: 线的起点x
    :param y2: 线的起点y
    :param x3: 线的终点x
    :param y3: 线的终点y
    :param x1: 点的x
    :param y1: 点的y
    :param type: 0: 右侧线，1: 下侧线
    """
    def deg_180_90(deg):
        if deg > 90:
            deg = deg - 180
        return deg

    distance = abs((y3 - y2) * x1 - (x3 - x2) * y1 + x3 * y2 - y3 * x2) / np.sqrt(
        (y3 - y2) ** 2 + (x3 - x2) ** 2
    )
    if type == 0:
        theta = deg_180_90((np.pi / 2 + np.arctan((y3 - y2) / (x3 - x2))) * 180 / np.pi)
    elif type == 1:
        theta = np.arctan((x3 - x2) / (y3 - y2)) * 180 / np.pi
    return (distance, theta)


while True:
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    t1 = time.perf_counter()
    img = radar.map.output_cloud()
    # img = cv2.imread("radar_map.png")
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    process(img)
    t2 = time.perf_counter()
    print(f"Process time: {t2 - t1}")
