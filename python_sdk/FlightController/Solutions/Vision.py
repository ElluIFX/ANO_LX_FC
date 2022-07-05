from typing import List, Optional, Tuple

import cv2
import numpy as np
from pyzbar import pyzbar
from scipy import ndimage


def black_line(
    image, type: int = 1, theta_threshold=0.25
) -> Tuple[bool, float, float, float]:
    """
    寻找画面中的黑线并返回数据
    type: 0:横线 1:竖线
    theta_threshold: 角度容许误差(不能超过45度)
    return: 是否查找到黑线, x偏移值(右正), y偏移值(下正), 弧度偏移值(顺时针正)
    """
    ######### 参数设置 #########
    LOWER = np.array([0, 0, 0])
    UPPER = np.array([180, 255, 50])
    HOUGH_THRESHOLD = 200
    ###########################
    hsv_img = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv_img, LOWER, UPPER)
    # cv2.imshow("Process", mask)

    target_theta = 0 if type == 1 else np.pi / 2
    # lines = cv2.HoughLines(mask, 1, np.pi/180, threshold=400, max_theta=0.1)
    if type == 0:  # 横线
        lines = cv2.HoughLines(
            mask,
            1,
            np.pi / 180,
            threshold=HOUGH_THRESHOLD,
            min_theta=target_theta - theta_threshold,
            max_theta=target_theta + theta_threshold,
        )
    else:  # 竖线
        lines = cv2.HoughLines(
            mask,
            1,
            np.pi / 180,
            threshold=HOUGH_THRESHOLD,
            max_theta=theta_threshold,
        )
        lines2 = cv2.HoughLines(
            mask,
            1,
            np.pi / 180,
            threshold=HOUGH_THRESHOLD,
            min_theta=np.pi - theta_threshold,
        )
        if lines is not None and lines2 is not None:
            lines = np.concatenate((lines, lines2))
        elif lines is None and lines2 is not None:
            lines = lines2

    if lines is not None:
        for line in lines:
            r, theta = line[0]
            # if (
            #     abs(theta - target_theta) < theta_threshold
            #     or abs(theta - target_theta - np.pi) < theta_threshold
            # ):
            x0 = r * np.cos(theta)
            y0 = r * np.sin(theta)
            x1 = int(x0 - 1000 * np.sin(theta))
            y1 = int(y0 + 1000 * np.cos(theta))
            x2 = int(x0 + 1000 * np.sin(theta))
            y2 = int(y0 - 1000 * np.cos(theta))
            # cv2.line(image, (x1, y1), (x2, y2), (0, 0, 255), 2)
            # cv2.imshow("Result", image)
            x = abs((x1 + x2) / 2)
            y = abs((y1 + y2) / 2)
            size = image.shape
            x_offset = x - size[1] / 2
            y_offset = y - size[0] / 2
            if theta > np.pi / 2 and type == 1:
                t_offset = theta - target_theta - np.pi
            else:
                t_offset = theta - target_theta
            return True, x_offset, y_offset, t_offset
    return False, 0, 0, 0


def pass_filter(img, kernel_size=3):
    """
    高/低通滤波器
    kernel_size: 3 / 5 / g (Gaussian)
    """
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if kernel_size == 3:
        kernel_3 = np.array([[-1, -1, -1], [-1, 8, -1], [-1, -1, -1]])
        return ndimage.convolve(img, kernel_3)
    elif kernel_size == 5:
        kernel_5 = np.array(
            [
                [-1, -1, -1, -1, -1],
                [-1, 1, 2, 1, -1],
                [-1, 2, 4, 2, -1],
                [-1, 1, 2, 1, -1],
                [-1, -1, -1, -1, -1],
            ]
        )
        return ndimage.convolve(img, kernel_5)
    elif kernel_size == "g":
        blurred = cv2.GaussianBlur(img, (11, 11), 0)
        g_hpf = img - blurred
        return g_hpf
    else:
        return img


def find_QRcode_zbar(frame):
    """
    使用pyzbar寻找条码
    return: 是否找到条码, x偏移值(右正), y偏移值(下正)
    """
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换成灰度图
    barcodes = pyzbar.decode(image)
    if barcodes != []:
        size = image.shape
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            x_offset = cx - size[1] / 2
            y_offset = cy - size[0] / 2
            # cv2.circle(image, (cx, cy), 2, (0, 255, 0), 8)
            # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # cv2.imshow("Result", image)
            return True, x_offset, y_offset
    return False, 0, 0


def find_QRcode_contour(frame):
    """
    基于形态学轮廓寻找条码
    return: 是否找到条码, x偏移值(右正), y偏移值(下正)
    """
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换成灰度图

    # 提取图像梯度提取二维码区域
    gradX = cv2.Sobel(image, ddepth=cv2.CV_32F, dx=1, dy=0, ksize=-1)
    gradY = cv2.Sobel(image, ddepth=cv2.CV_32F, dx=0, dy=1, ksize=-1)
    gradient = cv2.subtract(gradX, gradY)
    gradient = cv2.convertScaleAbs(gradient)
    # cv2.imshow('sobel', gradient)
    # 去噪并提取兴趣区域
    blurred = cv2.blur(gradient, (9, 9))
    (_, thresh) = cv2.threshold(blurred, 90, 255, cv2.THRESH_BINARY)
    cv2.imshow("thresh", thresh)

    # construct a closing kernel and apply it to the thresholded image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # 进行开运算和闭运算
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)
    cv2.imshow("closed", closed)
    # 处理轮廓，找出最大轮廓
    cnts, _ = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if cnts != []:
        size = image.shape
        c = sorted(cnts, key=cv2.contourArea, reverse=True)[0]
        # compute the rotated bounding box of the largest contour
        rect = cv2.minAreaRect(c)
        box = np.int0(cv2.boxPoints(rect))
        # 找出中心点坐标
        M = cv2.moments(c)
        cx = int(M["m10"] / (M["m00"] + 0.0001))
        cy = int(M["m01"] / (M["m00"] + 0.0001))
        x_offset = cx - size[1] / 2
        y_offset = cy - size[0] / 2
        # 做出轮廓和中心坐标点
        image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        cv2.circle(image, (cx, cy), 2, (0, 255, 0), 8)  # 做出中心坐标
        cv2.drawContours(image, [box], -1, (0, 255, 0), 3)
        cv2.imshow("Result", image)
        return True, x_offset, y_offset
    else:
        return False, 0, 0


hist_scale = 10
hsv_map = None


def _hsv_viewer_set_scale(val):
    global hist_scale
    hist_scale = val


def init_hsv_viewer():
    """
    初始化颜色直方图窗口
    """
    global hsv_map
    hsv_map = np.zeros((180, 256, 3), np.uint8)
    h, s = np.indices(hsv_map.shape[:2])
    hsv_map[:, :, 0] = h
    hsv_map[:, :, 1] = s
    hsv_map[:, :, 2] = 255
    hsv_map = cv2.cvtColor(hsv_map, cv2.COLOR_HSV2BGR)
    cv2.namedWindow("hsv_map", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("hsv_map", hsv_map)
    cv2.namedWindow("hsv_hist", cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar("scale", "hsv_hist", hist_scale, 32, _hsv_viewer_set_scale)


def update_hsv_viewer(img):
    """
    更新颜色直方图
    """
    global hsv_map
    global hist_scale
    # small = cv2.pyrDown(img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 去除孤立的点
    dark = hsv[:, :, 2] < 32
    hsv[dark] = 0
    h = cv2.calcHist([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
    h = np.clip(h * 0.005 * hist_scale, 0, 1)

    # 从一维变成三维
    # 将得到的直方图和颜色直方图相乘
    vis = hsv_map * h[:, :, np.newaxis] / 255.0
    cv2.imshow("hsv_hist", vis)


def rescale_image(image, scale: float, fast: bool = False) -> np.ndarray:
    """
    调整图像大小
    image: 原图像
    scale: 缩放比例(不要超过1,那没意义)
    fast: 是否使用快速算法
    return: 缩放后的图像
    """
    if scale < 1:
        inter_mathod = cv2.INTER_AREA if not fast else cv2.INTER_NEAREST
    else:
        inter_mathod = cv2.INTER_LINEAR
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=inter_mathod)


def init_hsv_selector():
    cv2.namedWindow("Selector", cv2.WINDOW_AUTOSIZE)
    nothing = lambda x: 0
    cv2.createTrackbar("H_l", "Selector", 0, 180, nothing)
    cv2.createTrackbar("H_h", "Selector", 0, 180, nothing)
    cv2.createTrackbar("S_l", "Selector", 0, 255, nothing)
    cv2.createTrackbar("S_h", "Selector", 0, 255, nothing)
    cv2.createTrackbar("V_l", "Selector", 0, 255, nothing)
    cv2.createTrackbar("V_h", "Selector", 0, 255, nothing)

def update_hsv_selector(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (9, 9), 0)
    h_l = cv2.getTrackbarPos("H_l", "Selector")
    h_h = cv2.getTrackbarPos("H_h", "Selector")
    s_l = cv2.getTrackbarPos("S_l", "Selector")
    s_h = cv2.getTrackbarPos("S_h", "Selector")
    v_l = cv2.getTrackbarPos("V_l", "Selector")
    v_h = cv2.getTrackbarPos("V_h", "Selector")
    mask = cv2.inRange(hsv, (h_l, s_l, v_l), (h_h, s_h, v_h))
    cv2.imshow("Result", mask)