import time

import cv2
import numpy as np
from FlightController.Components import LD_Radar
from FlightController.Solutions.Vision import *
from FlightController.Solutions.Vision_Net import *

radar = LD_Radar()
try:
    radar.start("COM33", "LD06")
except:
    radar.start("/dev/ttyUSB0", "LD06")


def process(img):
    HOUGH_THRESHOLD = 120
    cv2.imshow("Origin", img)

    # 图像膨胀
    KERNAL_SIZE = 7
    kernel = np.ones((KERNAL_SIZE, KERNAL_SIZE), np.uint8)  # 定义卷积核
    img = cv2.dilate(img, kernel)
    cv2.imshow("Dilate", img)

    # HoughLines
    lines = cv2.HoughLines(
        img,
        1,
        np.pi / 180,
        threshold=HOUGH_THRESHOLD,
    )

    image1 = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    size = image1.shape
    cv2.circle(image1, (size[1] // 2, size[0] // 2), 5, (0, 0, 255), 2)
    image2 = image1.copy()
    #
    if lines is not None:
        # print(f"Total lines: {len(lines)}")
        for line in lines:
            r, theta = line[0]
            x0 = r * np.cos(theta)
            y0 = r * np.sin(theta)
            x1 = int(x0 - 2000 * np.sin(theta))
            y1 = int(y0 + 2000 * np.cos(theta))
            x2 = int(x0 + 2000 * np.sin(theta))
            y2 = int(y0 - 2000 * np.cos(theta))
            cv2.line(image1, (x1, y1), (x2, y2), (0, 255, 255), 1)
    cv2.imshow("Hough", image1)
    # HoughLinesP

    lines = cv2.HoughLinesP(
        img, 1, np.pi / 180, threshold=HOUGH_THRESHOLD, minLineLength=1, maxLineGap=100
    )

    if lines is not None:
        # print(f"Total linesP: {len(lines)}")
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(image2, (x1, y1), (x2, y2), (255, 255, 0), 2)

    cv2.imshow("HoughP", image2)


while True:
    t1 = time.perf_counter()
    img = radar.map.output_cloud()
    # img = cv2.imread("radar_map.png")
    # img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break
    process(img)
    t2 = time.perf_counter()
    print(f"Process time: {t2 - t1}")
