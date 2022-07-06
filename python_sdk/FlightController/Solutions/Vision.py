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


def find_yellow_code(image) -> Tuple[bool, float, float]:
    """
    寻找黄色条码
    return: 是否查找到黄色条码, x偏移值(右正), y偏移值(下正)
    """
    ######### 参数设置 #########
    LOWER = np.array([20, 40, 100])
    UPPER = np.array([60, 150, 255])
    ###########################
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # 根据闸值构建掩模
    mask = cv2.inRange(hsv, LOWER, UPPER)
    # 进行开运算和闭运算
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
    closed = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)
    # 找出边界
    conts, hier = cv2.findContours(closed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(image, conts, -1, (0, 255, 0), 3)  # 画出边框
    cv2.imshow("Process", image)
    if conts:
        cnts = conts[0]
        area = cv2.contourArea(cnts)
        # 设定面积闸值，排除黄色小噪点影响
        if area > 150:
            M = cv2.moments(cnts)
            cx = int(M["m10"] / (M["m00"]))
            cy = int(M["m01"] / (M["m00"]))
            cv2.circle(image, (cx, cy), 3, (0, 0, 255), thickness=-1)
            cv2.imshow("Result", image)
            size = image.shape
            return True, cx - size[1] / 2, cy - size[0] / 2
    return False, 0, 0


def find_laser_point(img):
    """
    寻找激光点
    return: 是否查找到激光点, x偏移值(右正), y偏移值(下正)
    """
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = cv2.inRange(img, 230, 255)  # 二值化
    # 图像膨胀
    kernel = np.ones((5, 5), np.uint8)  # 定义卷积核
    img = cv2.dilate(img, kernel)
    # cv2.imshow("Process", img)
    # 设置Blob检测参数
    params = cv2.SimpleBlobDetector_Params()
    # 设置颜色
    params.blobColor = 255
    # 设置闸值
    params.minThreshold = 1
    params.maxThreshold = 20
    # 设置面积
    params.filterByArea = True
    params.minArea = 10
    params.maxArea = 500
    # 设置圆性
    params.filterByCircularity = True
    params.minCircularity = 0.8
    # 设置惯量比
    params.filterByInertia = True
    params.minInertiaRatio = 0.2

    # detector = cv2.SimpleBlobDetector(params) # opencv<4.0
    detector = cv2.SimpleBlobDetector_create(params)

    # Blob检测
    keypoints = detector.detect(img)

    # img_with_keypoints = cv2.drawKeypoints(
    #     img,
    #     keypoints,
    #     np.array([]),
    #     (0, 255, 0),
    #     cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS,
    # )
    # cv2.imshow("Result", img_with_keypoints)
    if keypoints:
        for i in range(0, len(keypoints)):
            x = keypoints[i].pt[0]
            y = keypoints[i].pt[1]
            cx = int(x)
            cy = int(y)
            size = img.shape
            return True, cx - size[1] / 2, cy - size[0] / 2
    else:
        return False, 0, 0


def pass_filter(img, kernel_size=3) -> np.ndarray:
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


def find_QRcode_zbar(frame) -> Tuple[bool, float, float]:
    """
    使用pyzbar寻找条码
    return: 是否找到条码, x偏移值(右正), y偏移值(下正), 条码内容
    """
    image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)  # 转换成灰度图
    barcodes = pyzbar.decode(image)
    if barcodes != []:
        size = image.shape
        for barcode in barcodes:
            (x, y, w, h) = barcode.rect
            data = barcode.data.decode("utf-8")
            cx = int(x + w / 2)
            cy = int(y + h / 2)
            x_offset = cx - size[1] / 2
            y_offset = cy - size[0] / 2
            # image = frame.copy()
            # cv2.circle(image, (cx, cy), 2, (0, 255, 0), 8)
            # cv2.rectangle(image, (x, y), (x + w, y + h), (0, 0, 255), 2)
            # cv2.putText(
            #     image, data, (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2
            # )
            # cv2.imshow("Result", image)
            return True, x_offset, y_offset, data
    return False, 0, 0, ""


def find_QRcode_contour(frame) -> Tuple[bool, float, float]:
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
    # cv2.imshow("thresh", thresh)

    # construct a closing kernel and apply it to the thresholded image
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (21, 7))
    closed = cv2.morphologyEx(thresh, cv2.MORPH_CLOSE, kernel)

    # 进行开运算和闭运算
    closed = cv2.erode(closed, None, iterations=4)
    closed = cv2.dilate(closed, None, iterations=4)
    # cv2.imshow("closed", closed)
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
        # image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
        # cv2.circle(image, (cx, cy), 2, (0, 255, 0), 8)  # 做出中心坐标
        # cv2.drawContours(image, [box], -1, (0, 255, 0), 3)
        # cv2.imshow("Result", image)
        return True, x_offset, y_offset
    else:
        return False, 0, 0


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


def dp_outline_calc(frame):
    """
    D-P算法轮廓面积计算
    return: 最大轮廓面积, 未找到时返回0
    """
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray_frame = cv2.GaussianBlur(gray_frame, (19, 19), 0)  # 高斯滤波进行平滑处理
    # 处理图像轮廓
    ret, thresh = cv2.threshold(gray_frame, 127, 255, 0)
    contours, hierarchy = cv2.findContours(
        thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE
    )
    if len(contours) == 0:
        return 0
    contours = list(contours)
    contours.sort(key=len, reverse=True)  # 进行排序，寻找极大轮廓
    cnt = contours[0]
    # 进行轮廓近似
    epsilon = 0.000001 * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    area = cv2.contourArea(approx)
    # frame = cv2.drawContours(frame, approx, -1, (255, 0, 0), 3)
    # cv2.imshow("Process", frame)
    return area


def FLANN_match(train_img, frame):
    """
    FLANN单应性特征匹配, 最小值匹配
    train_img: 目标查询图像
    frame: 待匹配图像
    return: 匹配点数量, 匹配中点坐标
    """
    ######### 参数设置 #########
    MIN_MATCH_COUNT = 10  # 最小匹配点数量
    FLANN_INDEX_KDTREE = 0  # FLANN索引类型
    ###########################
    train_img = train_img.copy()
    frame = frame.copy()
    sift = cv2.SIFT_create()
    kp1, des1 = sift.detectAndCompute(train_img, None)
    kp2, des2 = sift.detectAndCompute(frame, None)
    index_params = dict(algorithm=FLANN_INDEX_KDTREE, trees=5)
    search_params = dict(checks=50)
    flann = cv2.FlannBasedMatcher(index_params, search_params)
    # matches = flann.knnMatch(des1, des2, k=2) # BUG:数据类型转换错误
    matches = flann.knnMatch(
        np.asarray(des1, np.float32), np.asarray(des2, np.float32), k=2
    )
    # 最小匹配选择
    good = []
    for m, n in matches:
        if m.distance < 0.7 * n.distance:
            good.append(m)

    if len(good) > MIN_MATCH_COUNT:
        src_pts = np.float32([kp1[m.queryIdx].pt for m in good]).reshape(-1, 1, 2)
        dst_pts = np.float32([kp2[m.trainIdx].pt for m in good]).reshape(-1, 1, 2)

        M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)
        h, w = train_img.shape[:2]
        pts = np.float32([[0, 0], [0, h - 1], [w - 1, h - 1], [w - 1, 0]]).reshape(
            -1, 1, 2
        )
        dst = np.int32(cv2.perspectiveTransform(pts, M))
        p1, p2, p3, p4 = dst.reshape(4, 2)
        center_point = (p1 + p2 + p3 + p4) / 4
        ####### 透视关系
        # matchesMask = mask.ravel().tolist()
        # draw_params = dict(
        #     matchColor=(0, 255, 0),  # draw matches in green color
        #     singlePointColor=None,
        #     matchesMask=matchesMask,  # draw only inliers
        #     flags=2,
        # )
        # img3 = cv2.drawMatches(train_img, kp1, frame, kp2, good, None, **draw_params)
        # cv2.imshow("Process", img3)
        ####### 目标匹配
        # frame = cv2.polylines(frame, [dst], True, 255, 3, cv2.LINE_AA)
        # cv2.imshow("Result", frame)
        ###############
        return len(good), center_point
    else:
        return 0, (0, 0)


def contours_match(train_img, frame):
    """
    轮廓匹配
    train_img: 查询图片
    frame: 待匹配图片
    return: 匹配度(越小越匹配)
    """
    # 高斯滤波降噪
    frame_p = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    train_p = cv2.cvtColor(train_img, cv2.COLOR_BGR2GRAY)
    frame_p = cv2.GaussianBlur(frame_p, (5, 5), 0)
    train_p = cv2.GaussianBlur(train_p, (19, 19), 0)
    # 处理轮廓
    ret, thresh = cv2.threshold(frame_p, 127, 255, 0)
    ret, thresh2 = cv2.threshold(train_p, 127, 255, 0)
    contours1, hierarchy = cv2.findContours(thresh, 2, 1)
    contours2, hierarchy = cv2.findContours(thresh2, 2, 1)
    if len(contours1) == 0 or len(contours2) == 0:
        return 1.0
    cnt1 = contours1[0]
    cnt2 = contours2[0]
    # 计算匹配度
    matching_value = cv2.matchShapes(cnt1, cnt2, 1, 0.0)
    return matching_value


class Meanshift(object):
    ######### 参数设置 #########
    LOWER = np.array((99.0, 90.0, 102.0))
    UPPER = np.array((132.0, 212.0, 157.0))
    LOW_PASS_RATIO = 1
    TERM_ITER = 10  # 终止条件: 迭代次数
    TERM_MOVE = 1  # 终止条件: 移动距离
    ###########################
    def __init__(self, frame, init_ROI):
        """
        均值漂移目标跟踪
        frame:当前帧
        init_ROI: 初始兴趣区域, (c, r, w, h) (角点和宽高)
        """
        self.ROI = np.array(init_ROI)
        # c, r, w, h = ROI
        # roi = frame[r : r + h, c : c + w]
        hsv_roi = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 创建包含具有HSV的所有像素的掩码
        mask = cv2.inRange(hsv_roi, self.LOWER, self.UPPER)
        # 计算直方图,参数为 图片(可多)，通道数，蒙板区域，直方图长度，范围
        self.roi_hist = cv2.calcHist([hsv_roi], [0], mask, [180], [0, 180])
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)  # 归一化
        # 设置终止条件，迭代10次或者至少移动1次
        self.term_crit = (
            cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT,
            self.TERM_ITER,
            self.TERM_MOVE,
        )
        self.update(frame)

    def update(self, frame):
        """
        更新目标
        frame: 当前帧
        return: 目标x偏移, 目标y偏移
        """
        size = frame.shape
        # 处理训练图像
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # 直方图反向投影，求得每个像素点的概率
        dst = cv2.calcBackProject([hsv], [0], self.roi_hist, [0, 180], 1)
        # 调用meanShift算法在dst中寻找目标窗口，找到后返回目标窗口
        ret, track_window = cv2.meanShift(dst, self.ROI, self.term_crit)
        x, y, w, h = track_window
        # 更新ROI
        self.ROI[0] += (x - self.ROI[0]) * self.LOW_PASS_RATIO
        self.ROI[1] += (y - self.ROI[1]) * self.LOW_PASS_RATIO
        # 计算中心点坐标
        cx = int(x + w / 2)
        cy = int(y + h / 2)
        output_img = cv2.rectangle(frame, (x, y), (x + w, y + h), 255, 2)
        cv2.imshow("Result", output_img)
        return cx - size[1] / 2, cy - size[0] / 2

    def reset_roi(self, ROI):
        """
        重置ROI
        """
        self.ROI = np.array(ROI)


__bs = None


def mixed_background_sub(frame):
    """
    混合高斯运动检测
    frame: 输入帧
    return: 是否检测到运动物体, 物体中点坐标列表
    """
    global __bs
    if __bs is None:
        __bs = cv2.createBackgroundSubtractorKNN(detectShadows=True)
    fgmask = __bs.apply(frame)
    th = cv2.threshold(fgmask.copy(), 244, 255, cv2.THRESH_BINARY)[1]  # 将非纯白色像素设为0
    th = cv2.erode(
        th, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=2
    )  # 腐蚀图像
    dilated = cv2.dilate(
        th, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (8, 3)), iterations=2
    )  # 膨胀处理
    contours, hier = cv2.findContours(
        dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
    )
    detection_list = []
    for c in contours:
        if cv2.contourArea(c) > 1000:
            (x, y, w, h) = cv2.boundingRect(c)
            detection_list.append((x + w / 2, y + h / 2))
            # cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 255, 0), 2)
    # cv2.imshow("mog", fgmask)
    # cv2.imshow("thresh", th)
    # cv2.imshow("diff", frame & cv2.cvtColor(fgmask, cv2.COLOR_GRAY2BGR))
    # cv2.imshow("detection", frame)
    if len(detection_list) > 0:
        return True, detection_list
    return False, []


__hsv_map = None


def init_hsv_viewer():
    """
    (调试工具) 初始化HSV颜色直方图窗口
    """
    global __hsv_map
    __hsv_map = np.zeros((180, 256, 3), np.uint8)
    h, s = np.indices(__hsv_map.shape[:2])
    __hsv_map[:, :, 0] = h
    __hsv_map[:, :, 1] = s
    __hsv_map[:, :, 2] = 255
    __hsv_map = cv2.cvtColor(__hsv_map, cv2.COLOR_HSV2BGR)
    cv2.namedWindow("hsv_map", cv2.WINDOW_AUTOSIZE)
    cv2.imshow("hsv_map", __hsv_map)
    cv2.namedWindow("hsv_hist", cv2.WINDOW_AUTOSIZE)
    cv2.createTrackbar("scale", "hsv_hist", 10, 32, lambda x: 0)


def update_hsv_viewer(img):
    """
    (调试工具) 更新HSV颜色直方图
    """
    global __hsv_map
    # small = cv2.pyrDown(img)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # 去除孤立的点
    dark = hsv[:, :, 2] < 32
    hsv[dark] = 0
    h = cv2.calcHist([hsv], [0, 1], None, [180, 256], [0, 180, 0, 256])
    hist_scale = cv2.getTrackbarPos("scale", "hsv_hist")
    h = np.clip(h * 0.005 * hist_scale, 0, 1)

    # 从一维变成三维
    # 将得到的直方图和颜色直方图相乘
    vis = __hsv_map * h[:, :, np.newaxis] / 255.0
    cv2.imshow("hsv_hist", vis)


def init_hsv_selector():
    """
    (调试工具) 初始化HSV颜色选择器
    """
    cv2.namedWindow("Selector", cv2.WINDOW_AUTOSIZE)
    cv2.namedWindow("HSV_img", cv2.WINDOW_AUTOSIZE)
    nothing = lambda x: 0
    cv2.createTrackbar("H_l", "Selector", 0, 180, nothing)
    cv2.createTrackbar("H_h", "Selector", 0, 180, nothing)
    cv2.createTrackbar("S_l", "Selector", 0, 255, nothing)
    cv2.createTrackbar("S_h", "Selector", 0, 255, nothing)
    cv2.createTrackbar("V_l", "Selector", 0, 255, nothing)
    cv2.createTrackbar("V_h", "Selector", 0, 255, nothing)


def update_hsv_selector(img):
    """
    (调试工具) 更新HSV颜色选择器
    """
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    hsv = cv2.GaussianBlur(hsv, (9, 9), 0)
    h_l = cv2.getTrackbarPos("H_l", "Selector")
    h_h = cv2.getTrackbarPos("H_h", "Selector")
    s_l = cv2.getTrackbarPos("S_l", "Selector")
    s_h = cv2.getTrackbarPos("S_h", "Selector")
    v_l = cv2.getTrackbarPos("V_l", "Selector")
    v_h = cv2.getTrackbarPos("V_h", "Selector")
    mask = cv2.inRange(hsv, (h_l, s_l, v_l), (h_h, s_h, v_h))
    cv2.imshow("HSV_img", mask)
