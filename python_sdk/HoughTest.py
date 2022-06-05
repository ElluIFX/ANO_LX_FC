import os
import time

import cv2
import numpy as np

# destroy cv2 window when any exception occurs


def segment_by_angle_kmeans(lines, k=2, type_p=False, **kwargs):
    from collections import defaultdict

    if len(lines) < k:
        return lines

    default_criteria_type = cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER
    criteria = kwargs.get("criteria", (default_criteria_type, 10, 1.0))
    flags = kwargs.get("flags", cv2.KMEANS_RANDOM_CENTERS)
    attempts = kwargs.get("attempts", 10)
    if not type_p:
        angles = np.array([line[0][1] for line in lines], dtype=np.float32)
    else:
        x1, y1, x2, y2 = lines[:, 0, 0], lines[:, 0, 1], lines[:, 0, 2], lines[:, 0, 3]
        angles = np.arctan2(y2 - y1, x2 - x1)
        angles[angles < 0] += np.pi
    pts = np.array(
        [[np.cos(2 * angle), np.sin(2 * angle)] for angle in angles], dtype=np.float32
    )
    labels, centers = cv2.kmeans(pts, k, None, criteria, attempts, flags)[1:]
    labels = labels.reshape(-1)
    segmented = defaultdict(list)
    for i, line in enumerate(lines):
        segmented[labels[i]].append(line)
    segmented = list(segmented.values())
    return segmented


def intersection(line1, line2):
    rho1, theta1 = line1[0]
    rho2, theta2 = line2[0]
    A = np.array([[np.cos(theta1), np.sin(theta1)], [np.cos(theta2), np.sin(theta2)]])
    b = np.array([[rho1], [rho2]])
    x0, y0 = np.linalg.solve(A, b)
    x0, y0 = int(np.round(x0)), int(np.round(y0))
    return [[x0, y0]]


def intersection_p(line1, line2, find_outside_line):
    try:
        x11, y11, x12, y12 = line1[0]
        x21, y21, x22, y22 = line2[0]
        k1 = (y12 - y11) / (x12 - x11)
        c1 = y11 - k1 * x11
        k2 = (y22 - y21) / (x22 - x21)
        c2 = y21 - k2 * x21
        A = np.array([[k1, -1], [k2, -1]])
        b = np.array([[-c1], [-c2]])
        x0, y0 = np.linalg.solve(A, b)
    except:
        return [[-1, -1]]
    if x0 == x0:
        x0, y0 = int(np.round(x0)), int(np.round(y0))
        if find_outside_line or (
            x0 >= min(x11, x12)
            and x0 <= max(x11, x12)
            and x0 >= min(x21, x22)
            and x0 <= max(x21, x22)
        ):
            return [[x0, y0]]
    return [[-1, -1]]


def segmented_intersections(segs, type_p=False, p_find_outside_line=False):
    intersections = []
    for i, group in enumerate(segs[:-1]):
        for next_group in segs[i + 1 :]:
            for line1 in group:
                for line2 in next_group:
                    if type_p:
                        inter = intersection_p(line1, line2, p_find_outside_line)
                    else:
                        inter = intersection(line1, line2)
                    if inter != [[-1, -1]]:
                        intersections.append(inter)
    return intersections


def merge_lines(lines):
    line = lines[0][0]
    if len(lines) == 1:
        return line
    weight = 1
    for each in lines[1:]:
        add_line = each[0]
        try:
            x1, y1, x2, y2 = add_line
        except:
            continue
        if y1 > y2:
            add_line = np.array([x2, y2, x1, y1])
        line = (weight / (weight + 1)) * line + (1 / (weight + 1)) * add_line
        weight += 1
    return line


def calc_line_angle(line):
    x1, y1, x2, y2 = line
    deg = np.arctan2(y2 - y1, x2 - x1)
    while deg < 0:
        deg += np.pi
    deg %= np.pi
    return deg * 180 / np.pi


def process(img, center_x=0.7, center_y=0.8):
    # img = img.copy()
    width = img.shape[1]
    height = img.shape[0]
    cv2.imshow("source", img)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    blur = cv2.medianBlur(gray, 5)
    # blur = gray
    bin_img = cv2.adaptiveThreshold(
        blur, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY_INV, 15, 13
    )
    # cv2.imshow("bin_img", bin_img)
    lines = cv2.HoughLinesP(
        bin_img, 1, np.pi / 180, 100, minLineLength=60, maxLineGap=5
    )
    draw_img = cv2.cvtColor(bin_img, cv2.COLOR_GRAY2BGR)
    if lines is not None and len(lines) >= 2:
        segs = segment_by_angle_kmeans(lines, 2, True)
        merged = [merge_lines(s) for s in segs]

        def draw_segs(img, segs):
            colors = [(0, 0, 200), (0, 200, 0), (200, 0, 0)]
            n = 0
            for seg in segs:
                for line in seg:
                    try:
                        x1, y1, x2, y2 = line[0]
                    except:
                        return
                    cv2.line(img, (x1, y1), (x2, y2), colors[n % 6], 2)
                n += 1

        draw_segs(draw_img, segs)
        a = 0
        b = 1
        test_deg1 = calc_line_angle(merged[a])
        test_deg2 = calc_line_angle(merged[b])
        if abs(test_deg2) > abs(test_deg1):
            a = 1
            b = 0
        m1_x1, m1_y1, m1_x2, m1_y2 = (
            (merged[a][0]),
            (merged[a][1]),
            (merged[a][2]),
            (merged[a][3]),
        )
        m2_x1, m2_y1, m2_x2, m2_y2 = (
            (merged[b][0]),
            (merged[b][1]),
            (merged[b][2]),
            (merged[b][3]),
        )
        deg1 = calc_line_angle(merged[a])
        deg2 = calc_line_angle(merged[b])
        print(f"deg1: {deg1}")
        print(f"deg2: {deg2}")
        y_offset = None

        center_x = width / 2 if center_x is None else center_x * width
        center_y = height / 2 if center_y is None else center_y * height

        cv2.line(
            draw_img, (int(center_x), height), (int(center_x), 0), (125, 125, 0), 1
        )
        cv2.line(draw_img, (0, int(center_y)), (width, int(center_y)), (125, 125, 0), 1)

        horizon_threshold_deg = 30

        if abs(abs(deg1) - abs(deg2)) % 90 > horizon_threshold_deg:
            print(f"detected crossing")
            line_num = len(segs[a])
            line_num2 = len(segs[b])
            m1_c_x, m1_c_y = (m1_x1 + m1_x2) / 2, (m1_y1 + m1_y2) / 2
            m2_c_x, m2_c_y = (m2_x1 + m2_x2) / 2, (m2_y1 + m2_y2) / 2
            x_offset = m1_c_x - center_x
            y_offset = m2_c_y - center_y
            deg_offset = 90 - deg1
            cv2.line(
                draw_img,
                (int(m1_x1), int(m1_y1)),
                (int(m1_x2), int(m1_y2)),
                (255, 255, 0),
                2,
            )
            cv2.circle(draw_img, (int(m1_c_x), int(m1_c_y)), 20, (255, 255, 0), 2)
            cv2.line(
                draw_img,
                (int(m1_c_x), int(m1_c_y)),
                (int(center_x), int(m1_c_y)),
                (0, 0, 255),
                2,
            )
            cv2.circle(draw_img, (int(center_x), int(m1_c_y)), 3, (0, 0, 255), -1)
            cv2.line(
                draw_img,
                (int(m2_x1), int(m2_y1)),
                (int(m2_x2), int(m2_y2)),
                (0, 255, 255),
                2,
            )
            cv2.circle(draw_img, (int(m2_c_x), int(m2_c_y)), 20, (0, 255, 255), 2)
            cv2.line(
                draw_img,
                (int(m2_c_x), int(m2_c_y)),
                (int(m2_c_x), int(center_y)),
                (0, 255, 0),
                2,
            )
            cv2.circle(draw_img, (int(m2_c_x), int(center_y)), 3, (0, 255, 0), -1)
            print("line_num:", line_num, "line_num2:", line_num2)
            print("center 1:", m1_c_x, m1_c_y)
            print("center 2:", m2_c_x, m2_c_y)
            print("x_offset:", x_offset)
            print("deg1_offset:", deg_offset)
            print("y_offset:", y_offset)
        else:
            print(f"detected straight")
            line_num = len(lines)
            s_merged = merge_lines(lines)
            m_x1, m_y1, m_x2, m_y2 = (
                (s_merged[0]),
                (s_merged[1]),
                (s_merged[2]),
                (s_merged[3]),
            )
            deg = calc_line_angle(s_merged)
            m_c_x, m_c_y = (m_x1 + m_x2) / 2, (m_y1 + m_y2) / 2
            x_offset = m_c_x - center_x
            deg_offset = 90 - deg
            cv2.line(
                draw_img,
                (int(m_x1), int(m_y1)),
                (int(m_x2), int(m_y2)),
                (255, 255, 0),
                2,
            )
            cv2.circle(draw_img, (int(m_c_x), int(m_c_y)), 20, (255, 255, 0), 2)
            cv2.line(
                draw_img,
                (int(m_c_x), int(m_c_y)),
                (int(center_x), int(m_c_y)),
                (0, 0, 255),
                2,
            )
            cv2.circle(draw_img, (int(center_x), int(m_c_y)), 3, (0, 0, 255), -1)
            print("line_num:", line_num)
            print("center:", m_c_x, m_c_y)
            print("x_offset:", x_offset)
            print("deg_offset:", deg_offset)
            print("deg:", deg)
    else:
        return None, None, None
    cv2.imshow("result 1", draw_img)
    return x_offset, y_offset, deg_offset


if __name__ == "__main__":
    # cam = cv2.VideoCapture(1)
    # cam.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    # cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # cam.set(cv2.CAP_PROP_FPS, 30)
    # print(
    #     f"cam info: {cam.get(cv2.CAP_PROP_FRAME_WIDTH)} x {cam.get(cv2.CAP_PROP_FRAME_HEIGHT)} @ {cam.get(cv2.CAP_PROP_FPS)} fps"
    # )
    dir_ = "./cv_saved_images"
    filelist = os.listdir(dir_)
    i = 0
    len_list = len(filelist)
    while True:
        # ret, frame = cam.read()
        # if frame is None:
        # continue
        file = dir_ + "/" + filelist[i]
        frame = cv2.imread(file)
        print(f"loaded: {filelist[i]}")
        i = (i + 1) % len_list

        window = int((frame.shape[1] - frame.shape[0]) / 2)
        frame = frame[:, window : frame.shape[1] - window]
        t0 = time.time()
        process(frame)
        t1 = time.time()
        print(f"cost time: {t1 - t0 : .5f}s\n")
        key = cv2.waitKey(500) & 0xFF
        if key == ord("q"):
            break
        if key == ord("p"):
            cv2.waitKey()
    cv2.destroyAllWindows()
