import struct
import time
from typing import List, Optional, Tuple

import cv2
import numpy as np
from scipy.signal import find_peaks

from ..Logger import logger
from ._Driver_Components import calculate_crc8


class Point_2D(object):
    degree = 0.0  # 0.0 ~ 359.9, 0 指向前方, 顺时针
    distance = 0  # 距离 mm
    confidence = 0  # 置信度 典型值=200

    def __init__(self, degree=0, distance=0, confidence=None):
        self.degree = degree
        self.distance = distance
        self.confidence = confidence

    def __str__(self):
        s = f"Point: deg = {self.degree:>6.2f}, dist = {self.distance:>4.0f}"
        if self.confidence is not None:
            s += f", conf = {self.confidence:>3.0f}"
        return s

    def __repr__(self):
        return self.__str__()

    def to_xy(self):
        """
        转换到匿名坐标系下的坐标
        """
        return np.array(
            [
                self.distance * np.cos(self.degree * np.pi / 180),
                -self.distance * np.sin(self.degree * np.pi / 180),
            ]
        )

    def to_cv_xy(self):
        """
        转换到OpenCV坐标系下的坐标
        """
        return np.array(
            [
                self.distance * np.sin(self.degree * np.pi / 180),
                -self.distance * np.cos(self.degree * np.pi / 180),
            ]
        )

    def from_xy(self, xy: np.ndarray):
        """
        从匿名坐标系下的坐标转换到点
        """
        self.degree = np.arctan2(-xy[1], xy[0]) * 180 / np.pi % 360
        self.distance = np.sqrt(xy[0] ** 2 + xy[1] ** 2)

    def from_cv_xy(self, xy: np.ndarray):
        """
        从OpenCV坐标系下的坐标转换到点
        """
        self.degree = np.arctan2(xy[0], -xy[1]) * 180 / np.pi % 360
        self.distance = np.sqrt(xy[0] ** 2 + xy[1] ** 2)

    def __eq__(self, __o: object) -> bool:
        if not isinstance(__o, Point_2D):
            return False
        return self.degree == __o.degree and self.distance == __o.distance

    def to_180_degree(self):
        """
        转换到-180~180度
        """
        if self.degree > 180:
            return self.degree - 360
        return self.degree

    def __add__(self, other):
        if not isinstance(other, Point_2D):
            raise TypeError("Point_2D can only add with Point_2D")
        return Point_2D().from_xy(self.to_xy() + other.to_xy())

    def __sub__(self, other):
        if not isinstance(other, Point_2D):
            raise TypeError("Point_2D can only sub with Point_2D")
        return Point_2D().from_xy(self.to_xy() - other.to_xy())


class Radar_Package(object):
    """
    解析后的数据包
    """

    rotation_spd = 0  # 转速 deg/s
    start_degree = 0.0  # 扫描开始角度
    points = [Point_2D() for _ in range(12)]  # 12个点的数据
    stop_degree = 0.0  # 扫描结束角度
    time_stamp = 0  # 时间戳 ms 记满30000后重置

    def __init__(self, datas=None):
        if datas is not None:
            self.fill_data(datas)

    def fill_data(self, datas: Tuple[int]):
        self.rotation_spd = datas[0]
        self.start_degree = datas[1] * 0.01
        self.stop_degree = datas[26] * 0.01
        self.time_stamp = datas[27]
        deg_step = (self.stop_degree - self.start_degree) % 360 / 11
        for n, point in enumerate(self.points):
            point.distance = datas[2 + n * 2]
            point.confidence = datas[3 + n * 2]
            point.degree = (self.start_degree + n * deg_step) % 360

    def __str__(self):
        string = (
            f"--- Radar Package < TS = {self.time_stamp:05d} ms > ---\n"
            f"Range: {self.start_degree:06.2f}° -> {self.stop_degree:06.2f}° {self.rotation_spd/360:3.2f}rpm\n"
        )
        for n, point in enumerate(self.points):
            string += f"#{n:02d} {point}\n"
        string += "--- End of Info ---"
        return string

    def __repr__(self):
        return self.__str__()


_radar_unpack_fmt = "<HH" + "HB" * 12 + "HH"  # 雷达数据解析格式


def resolve_radar_data(data: bytes, to_package: Radar_Package = None) -> Radar_Package:
    """
    解析雷达原始数据
    data: bytes 原始数据
    to_package: 传入一个RadarPackage对象, 如果不传入, 则会新建一个
    return: 解析后的RadarPackage对象
    """
    if len(data) != 47:  # fixed length of radar data
        logger.warning(f"[RADAR] Invalid data length: {len(data)}")
        return None
    if calculate_crc8(data[:-1]) != data[-1]:
        logger.warning("[RADAR] Invalid CRC8")
        return None
    if data[:2] != b"\x54\x2C":
        logger.warning(f"[RADAR] Invalid header: {data[:2]:X}")
        return None
    datas = struct.unpack(_radar_unpack_fmt, data[2:-1])
    if to_package is None:
        return Radar_Package(datas)
    else:
        to_package.fill_data(datas)
        return to_package


class Map_360(object):
    """
    将点云数据映射到一个360度的圆上
    """

    data = np.ones(360, dtype=np.int64) * -1  # -1: 未知
    time_stamp = np.zeros(360, dtype=np.float64)  # 时间戳
    ######### 映射方法 ########
    MODE_MIN = 0  # 在范围内选择最近的点更新
    MODE_MAX = 1  # 在范围内选择最远的点更新
    MODE_AVG = 2  # 计算平均值更新
    update_mode = MODE_MIN
    ######### 设置 #########
    confidence_threshold = 140  # 置信度阈值
    distance_threshold = 10  # 距离阈值
    timeout_clear = True  # 超时清除
    timeout_time = 1  # 超时时间 s
    ######### 状态 #########
    rotation_spd = 0  # 转速 rpm
    update_count = 0  # 更新计数
    ####### 辅助计算 #######
    _rad_arr = np.deg2rad(np.arange(0, 360))  # 弧度
    _deg_arr = np.arange(0, 360)  # 角度
    _sin_arr = np.sin(_rad_arr)
    _cos_arr = np.cos(_rad_arr)

    def __init__(self):
        pass

    def update(self, data: Radar_Package):
        """
        映射解析后的点云数据
        """
        deg_values_dict = {}
        for point in data.points:
            if (
                point.distance < self.distance_threshold
                or point.confidence < self.confidence_threshold
            ):
                continue
            base = int(point.degree + 0.5)
            degs = [base - 1, base, base + 1]  # 扩大点映射范围, 加快更新速度, 降低精度
            # degs = [base] # 只映射实际角度
            for deg in degs:
                deg %= 360
                if deg not in deg_values_dict:
                    deg_values_dict[deg] = set()
                deg_values_dict[deg].add(point.distance)
        for deg, values in deg_values_dict.items():
            if self.update_mode == self.MODE_MIN:
                self.data[deg] = min(values)
            elif self.update_mode == self.MODE_MAX:
                self.data[deg] = max(values)
            elif self.update_mode == self.MODE_AVG:
                self.data[deg] = int(sum(values) / len(values))
            if self.timeout_clear:
                self.time_stamp[deg] = time.time()
        if self.timeout_clear:
            self.data[self.time_stamp < time.time() - self.timeout_time] = -1
        self.rotation_spd = data.rotation_spd / 360
        self.update_count += 1

    def in_deg(self, from_: int, to_: int) -> List[Point_2D]:
        """
        截取选定角度范围的点
        """
        return [
            Point_2D(deg, self.data[deg])
            for deg in range(from_, to_ + 1)
            if self.data[deg] != -1
        ]

    def clear(self):
        """
        清空数据
        """
        self.data[:] = -1
        self.time_stamp[:] = 0

    def rotation(self, angle: int):
        """
        旋转整个地图, 正角度代表坐标系顺时针旋转, 地图逆时针旋转
        """
        self.data = np.roll(self.data, angle)
        self.time_stamp = np.roll(self.time_stamp, angle)

    def find_nearest(
        self, from_: int = 0, to_: int = 359, num=1, range_limit: int = 1e7, view=None
    ) -> List[Point_2D]:
        """
        在给定范围内查找给定个数的最近点
        from_: int 起始角度
        to_: int 结束角度(包含)
        num: int 查找点的个数
        view: numpy视图, 当指定时上述参数仅num生效
        """
        if view is None:
            view = (self.data < range_limit) & (self.data != -1)
            from_ %= 360
            to_ %= 360
            if from_ > to_:
                view[to_ + 2 : from_] = False
            else:
                view[to_ + 2 : 360] = False
                view[:from_] = False
        deg_arr = np.where(view)[0]
        data_view = self.data[view]
        p_num = len(deg_arr)
        if p_num == 0:
            return []
        elif p_num <= num:
            sort_view = np.argsort(data_view)
        else:
            sort_view = np.argpartition(data_view, num)[:num]
        points = []
        for index in sort_view:
            points.append(Point_2D(deg_arr[index], data_view[index]))
        return points

    def find_nearest_with_ext_point_opt(
        self, from_: int = 0, to_: int = 359, num=1, range_limit: int = 1e7
    ) -> List[Point_2D]:
        """
        在给定范围内查找给定个数的最近点, 只查找极值点
        from_: int 起始角度
        to_: int 结束角度(包含)
        num: int 查找点的个数
        range_limit: int 距离限制
        """
        view = (self.data < range_limit) & (self.data != -1)
        from_ %= 360
        to_ %= 360
        if from_ > to_:
            view[to_ + 2 : from_] = False
        else:
            view[to_ + 2 : 360] = False
            view[:from_] = False
        data_view = self.data[view]
        deg_arr = np.where(view)[0]
        peak = find_peaks(-data_view)[0]
        if len(data_view) > 2:
            if data_view[-1] < data_view[-2]:
                peak = np.append(peak, len(data_view) - 1)
        peak_deg = deg_arr[peak]
        new_view = np.zeros(360, dtype=bool)
        new_view[peak_deg] = True
        return self.find_nearest(from_, to_, num, range_limit, new_view)

    def draw_on_cv_image(
        self,
        img: np.ndarray,
        scale: float = 1,
        color: tuple = (0, 0, 255),
        point_size: int = 1,
        add_points: List[Point_2D] = [],
        add_points_color: tuple = (0, 255, 255),
        add_points_size: int = 1,
    ):
        img_size = img.shape
        center_point = np.array([img_size[1] / 2, img_size[0] / 2])
        points_pos = (
            np.array(
                [
                    self.data * self._sin_arr,
                    -self.data * self._cos_arr,
                ]
            )
            * scale
        )
        for n in range(360):
            pos = points_pos[:, n] + center_point
            if self.data[n] != -1:
                cv2.circle(img, tuple(pos.astype(int)), point_size, color, -1)
        for point in add_points:
            pos = center_point + point.to_cv_xy() * scale
            cv2.circle(
                img, (int(pos[0]), int(pos[1])), add_points_size, add_points_color, -1
            )
        cv2.putText(
            img,
            f"RPM={self.rotation_spd:.2f} CNT={self.update_count}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.4,
            (255, 255, 255),
        )
        return img

    def output_cloud(self, scale: float = 0.1, size=800) -> np.ndarray:
        black_img = np.zeros((size, size, 1), dtype=np.uint8)
        center_point = np.array([size // 2, size // 2])
        points_pos = (
            np.array(
                [
                    self.data * self._sin_arr,
                    -self.data * self._cos_arr,
                ]
            )
            * scale
        )
        for n in range(360):
            pos = points_pos[:, n] + center_point
            if self.data[n] != -1:
                if 0 <= pos[0] < size and 0 <= pos[1] < size:
                    black_img[int(pos[1]), int(pos[0])] = 255
        return black_img

    def get_distance(self, angle: int) -> int:
        return self.data[int(angle % 360)]

    def get_point(self, angle: int) -> Point_2D:
        return Point_2D(angle, self.data[int(angle % 360)])

    def __str__(self):
        string = "--- 360 Degree Map ---\n"
        invalid_count = 0
        for deg in range(360):
            if self.data[deg] == -1:
                invalid_count += 1
                continue
            string += f"{deg:03d}° = {self.data[deg]} mm\n"
        if invalid_count > 0:
            string += f"Hided {invalid_count:03d} invalid points\n"
        string += "--- End of Info ---"
        return string

    def __repr__(self):
        return self.__str__()


if __name__ == "__main__":
    from ._Test_Driver import TEST_DATA

    map_ = Map_360()
    pack = resolve_radar_data(TEST_DATA)
    map_.update(pack)
    print(pack)
    print(map_)
    find = map_.find_nearest_with_ext_point_opt(0, 359, 4)
    print(find)
