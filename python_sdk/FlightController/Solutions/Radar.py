from typing import Literal

import cv2
import numpy as np

from ..Components.LDRadar_Resolver import Point_2D

_rt_pose = [0, 0, 0]  # x,y,yaw


def radar_resolve_rt_pose(img, _DEBUG=False) -> list[float, float, float]:
    """
    从雷达点云图像中解析出中点位置
    img: 雷达点云图像(灰度图)
    _DEBUG: 显示解析结果
    return: 位姿(x,y,yaw)
    """
    global _rt_pose
    ############参数设置##############
    KERNAL_DI = 9
    KERNAL_ER = 5
    HOUGH_THRESHOLD = 80
    MIN_LINE_LENGTH = 60
    LOW_PASS_RATIO = 0.1  # 平滑滤波系数
    #################################
    kernel_di = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNAL_DI, KERNAL_DI))
    kernel_er = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (KERNAL_ER, KERNAL_ER))
    img = cv2.dilate(img, kernel_di)  # 膨胀
    img = cv2.erode(img, kernel_er)  # 腐蚀
    lines = cv2.HoughLinesP(
        img,
        1,
        np.pi / 180,
        threshold=HOUGH_THRESHOLD,
        minLineLength=MIN_LINE_LENGTH,
        maxLineGap=200,
    )
    size = img.shape
    x0, y0 = size[0] // 2, size[1] // 2
    if _DEBUG:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    right_lines = []
    back_lines = []
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            if (
                x1 > x0 and x2 > x0 and ((y1 > y0 and y2 < y0) or (y1 < y0 and y2 > y0))
            ):  # 右侧线
                if x1 > x2:
                    right_lines.append((x2, y2, x1, y1))
                else:
                    right_lines.append((x1, y1, x2, y2))
            elif (
                y1 > y0 and y2 > y0 and ((x1 > x0 and x2 < x0) or (x1 < x0 and x2 > x0))
            ):  # 下侧线
                if y1 > y2:
                    back_lines.append((x2, y2, x1, y1))
                else:
                    back_lines.append((x1, y1, x2, y2))
    right_lines.sort(key=lambda line: line[0])
    back_lines.sort(key=lambda line: line[1])
    if len(right_lines) > 0:
        x1, y1, x2, y2 = right_lines[0]
        dis, yaw = get_point_line_distance(x0, y0, x1, y1, x2, y2, 0)
        _rt_pose[1] = _rt_pose[1] + LOW_PASS_RATIO * (dis - _rt_pose[1])
        _rt_pose[2] = _rt_pose[2] + LOW_PASS_RATIO * (-yaw - _rt_pose[2])
        if _DEBUG:
            cv2.line(img, (x1, y1), (x2, y2), (255, 255, 0), 2)
            p = Point_2D(-_rt_pose[2] + 90, _rt_pose[1])
            target = p.to_cv_xy() + np.array([x0, y0])
            cv2.line(img, (x0, y0), (int(target[0]), int(target[1])), (0, 0, 255), 1)
    if len(back_lines) > 0:
        x1, y1, x2, y2 = back_lines[0]
        dis, yaw = get_point_line_distance(x0, y0, x1, y1, x2, y2, 1)
        _rt_pose[0] = _rt_pose[0] + LOW_PASS_RATIO * (dis - _rt_pose[0])
        _rt_pose[2] = _rt_pose[2] + LOW_PASS_RATIO * (-yaw - _rt_pose[2])
        if _DEBUG:
            cv2.line(img, (x1, y1), (x2, y2), (0, 255, 255), 2)
            p = Point_2D(-_rt_pose[2] + 180, _rt_pose[0])
            target = p.to_cv_xy() + np.array([x0, y0])
            cv2.line(img, (x0, y0), (int(target[0]), int(target[1])), (0, 0, 255), 1)
    if _DEBUG:
        cv2.putText(
            img,
            f"({_rt_pose[0]:.1f}, {_rt_pose[1]:.1f},{_rt_pose[2]:.1f})",
            (x0, y0 - 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            1,
        )
        cv2.imshow("Map Resolve", img)
    return _rt_pose


def get_point_line_distance(
    x1, y1, x2, y2, x3, y3, type: Literal[0, 1] = 0
) -> tuple[float, float]:
    """
    计算点到线的距离
    p1: 目标点
    p2: 线的一个端点
    p3: 线的另一个端点
    type: 0: 右侧线, 1: 下侧线
    return: 距离, 角度
    """

    def deg_180_90(deg):
        if deg > 90:
            deg = deg - 180
        return deg

    distance = abs((y3 - y2) * x1 - (x3 - x2) * y1 + x3 * y2 - y3 * x2) / np.sqrt(
        (y3 - y2) ** 2 + (x3 - x2) ** 2
    )
    if type == 0:
        theta = deg_180_90(
            (np.pi / 2 + np.arctan((y3 - y2) / (x3 - x2 + 0.0000001))) * 180 / np.pi
        )
    elif type == 1:
        theta = np.arctan((y3 - y2) / (x3 - x2 + 0.0000001)) * 180 / np.pi
    return (distance, theta)
