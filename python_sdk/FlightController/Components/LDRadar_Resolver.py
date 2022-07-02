import struct
import time

import cv2
import numpy as np

from ..Logger import logger
from ._Driver_Components import calculate_crc8


class Point_2D(object):
    degree = 0.0  # 0.0 ~ 360.0, 0 指向前方, 顺时针
    distance = 0  # 距离 mm
    confidence = 0  # 置信度 典型值=200

    def __init__(self, degree=0, distance=0, confidence=0):
        self.degree = degree
        self.distance = distance
        self.confidence = confidence

    def __str__(self):
        return f"Point: deg = {self.degree:>6.2f}, dist = {self.distance:>4.0f}, conf = {self.confidence:>3.0f}"

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

    def __eq__(self, __o: object) -> bool:
        if not isinstance(__o, Point_2D):
            return False
        return (
            self.degree == __o.degree
            and self.distance == __o.distance
            and self.confidence == __o.confidence
        )

    def __call__(self):
        return (self.degree, self.distance, self.confidence)


class Radar_Package(object):
    """
    解析后的数据包
    """

    rotation_spd = 0  # 转速 deg/s
    start_degree = 0.0  # 扫描开始角度
    points = [Point_2D() for _ in range(12)]  # 12个点的数据
    stop_degree = 0.0  # 扫描结束角度
    time_stamp = 0  # 时间戳 ms 记满30000后重置

    def __init__(self):
        pass

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


_radar_unpack_fmt = (
    "<H"  # little endian, u16 rotation speed
    "H"  # u16 start degree
    f"{'HB' * 12}"  # 12 points, u16 distance, u8 confidence
    "H"  # u16 stop degree
    "H"  # u16 time stamp
)  # 雷达数据解析格式


def resolve_radar_data(data: bytes, to_package: Radar_Package = None) -> Radar_Package:
    """
    解析雷达原始数据
    data: bytes 原始数据
    to_package: 传入一个RadarPackage对象, 如果不传入, 则会新建一个
    return: 解析后的RadarPackage对象
    """
    if len(data) != 47:  # fixed length of radar data
        logger.error(f"[RADAR] Invalid data length: {len(data)}")
        return None
    if calculate_crc8(data[:-1]) != data[-1]:
        logger.error("[RADAR] Invalid CRC8")
        return None
    if data[:2] != b"\x54\x2C":
        logger.error(f"[RADAR] Invalid header: {data[:2]:X}")
        return None
    datas = struct.unpack(_radar_unpack_fmt, data[2:-1])
    if to_package is None:
        package = Radar_Package()
    else:
        package = to_package
    package.rotation_spd = datas[0]
    package.start_degree = datas[1] * 0.01
    package.time_stamp = datas[-1]
    package.stop_degree = datas[-2] * 0.01
    if package.stop_degree < package.start_degree:
        total_deg = package.stop_degree - package.start_degree + 360.0
    else:
        total_deg = package.stop_degree - package.start_degree
    deg_add = total_deg / 11
    for n, point in enumerate(package.points):
        point.distance = datas[2 + n * 2]
        point.confidence = datas[3 + n * 2]
        point.degree = package.start_degree + n * deg_add
        if point.degree > 360.0:
            point.degree -= 360.0
    return package


class Map_360(object):
    """
    将点云数据映射到一个360度的圆上
    """

    data = [-1] * 360  # -1 for no valid data
    ######### 映射方法 ########
    MODE_MIN = 0  # 在正负0.5度范围内选择最近的点更新
    MODE_MAX = 1  # 在正负0.5度范围内选择最远的点更新
    MODE_AVG = 2  # 计算平均值更新
    ######### 设置 #########
    update_mode = MODE_MIN
    confidence_threshold = 100  # 置信度阈值
    distance_threshold = 10  # 距离阈值
    ######### 状态 #########
    rotation_spd = 0  # 转速 rpm

    def __init__(self):
        pass

    def update(self, data: Radar_Package):
        deg_list = []
        deg_values_dict = {}
        for point in data.points:
            if (
                point.distance < self.distance_threshold
                or point.confidence < self.confidence_threshold
            ):
                continue
            deg = int(point.degree + 0.5)
            deg %= 360
            if deg not in deg_values_dict:
                deg_list.append(deg)
                deg_values_dict[deg] = []
            deg_values_dict[deg].append(point.distance)
        for deg, values in deg_values_dict.items():
            if self.update_mode == self.MODE_MIN:
                self.data[deg] = min(values)
            elif self.update_mode == self.MODE_MAX:
                self.data[deg] = max(values)
            elif self.update_mode == self.MODE_AVG:
                self.data[deg] = int(sum(values) / len(values))
        self.rotation_spd += (data.rotation_spd / 360 - self.rotation_spd) * 0.5

    def in_deg(self, from_: int, to_: int) -> list[Point_2D]:
        return [
            Point_2D(deg, self.data[deg])
            for deg in range(from_, to_)
            if self.data[deg] != -1
        ]

    def in_distance(self, from_: int, to_: int) -> list[Point_2D]:
        return [
            Point_2D(deg, self.data[deg])
            for deg in range(360)
            if from_ <= self.data[deg] <= to_
        ]

    def clear(self):
        """
        清空数据
        """
        self.data = [-1] * 360

    def rotation(self, angle: int):
        """
        旋转整个地图, 正角度代表坐标系顺时针旋转, 地图逆时针旋转
        """
        self.data = [self.data[(deg + angle) % 360] for deg in range(360)]

    def find_nearest(
        self, from_: int = 0, to_: int = 359, num=1, data=None
    ) -> list[Point_2D]:
        """
        在给定范围内查找给定个数的最近点
        from_: int 起始角度
        to_: int 结束角度(包含)
        num: int 查找点的个数
        """
        if not data:
            data = self.data
        if from_ > to_:
            range_ = list(range(from_, 360)) + list(range(0, to_))
        else:
            range_ = range(from_, to_ + 1)
        min_points = [Point_2D(-1, 1e8) for i in range(num)]
        for deg in range_:
            if data[deg] == -1:
                continue
            for n, point in enumerate(min_points):
                if data[deg] < point.distance:
                    k = num - 1
                    while k > n:
                        min_points[k] = min_points[k - 1]
                        k -= 1
                    min_points[n] = Point_2D(deg, data[deg])
                    break
        min_points = [point for point in min_points if point.degree != -1]
        return min_points

    def find_nearest_with_ext_point_opt(
        self, from_: int = 0, to_: int = 359, num=1
    ) -> list[Point_2D]:
        """
        在给定范围内查找给定个数的最近点, 只查找极值点
        from_: int 起始角度
        to_: int 结束角度(包含)
        num: int 查找点的个数
        """
        last = 1e9
        last_deg = -1
        ex_points = []
        state = 0  # 0:falling 1:rising
        data = []
        for deg in range(360):
            if self.data[deg] == -1:
                continue
            if state == 0:
                if self.data[deg] > last:
                    state = 1
                    ex_points.append(deg - 1)
            else:
                if self.data[deg] < last:
                    state = 0
            last = self.data[deg]
            last_deg = deg
        if state == 0 and last_deg != -1:
            ex_points.append(last_deg)
        for deg in range(360):
            if deg in ex_points:
                data.append(self.data[deg])
            else:
                data.append(1e9)
        return self.find_nearest(from_, to_, num, data)

    def draw_on_cv_image(
        self,
        img: np.ndarray,
        scale: float = 1,
        color: tuple = (0, 0, 255),
        point_size: int = 1,
        add_points: list[Point_2D] = [],
        add_points_color: tuple = (0, 255, 255),
        add_points_size: int = 1,
    ):
        img_size = img.shape
        center_point = np.array([img_size[1] / 2, img_size[0] / 2])
        for point in self.in_deg(0, 359):
            pos = center_point + point.to_cv_xy() * scale
            cv2.circle(img, (int(pos[0]), int(pos[1])), point_size, color, -1)
        for point in add_points:
            pos = center_point + point.to_cv_xy() * scale
            cv2.circle(
                img, (int(pos[0]), int(pos[1])), add_points_size, add_points_color, -1
            )
        cv2.putText(
            img,
            f"RPM={self.rotation_spd:.2f}",
            (10, 20),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
        )
        return img

    def __str__(self):
        string = "--- 360 Degree Map ---\n"
        invalid_count = 0
        for deg in range(360):
            if self.data[deg] == -1:
                invalid_count += 1
                continue
            string += f"{deg:03d} = {self.data[deg]:6d}\n"
        if invalid_count > 0:
            string += f"Hided {invalid_count:03d} invalid points\n"
        string += "--- End of Info ---"
        return string


if __name__ == "__main__":
    from ._Test_Driver import TEST_DATA

    map_ = Map_360()
    pack = resolve_radar_data(TEST_DATA)
    map_.update(pack)
    print(pack)
    print(map_)
    img = np.zeros((600, 600, 3), dtype=np.uint8) + 10
    cv2.line(img, (0, 0), (600, 600), (255, 0, 0), 1)
    cv2.line(img, (600, 0), (0, 600), (255, 0, 0), 1)
    cv2.line(img, (300, 0), (300, 600), (255, 0, 0), 1)
    cv2.line(img, (0, 300), (600, 300), (255, 0, 0), 1)
    cv2.circle(img, (300, 300), 100, (255, 0, 0), 1)
    cv2.circle(img, (300, 300), 200, (255, 0, 0), 1)
    cv2.circle(img, (300, 300), 300, (255, 0, 0), 1)
    n_p = map_.find_nearest_with_ext_point_opt(0, 359, 2)
    map_.draw_on_cv_image(img, add_points=n_p)
    cv2.imshow("Map", img)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
