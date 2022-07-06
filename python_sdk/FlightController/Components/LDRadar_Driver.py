import threading
import time

import cv2
import numpy as np
import serial

from ..Logger import logger
from .LDRadar_Resolver import Map_360, Point_2D, Radar_Package, resolve_radar_data


class LD_Radar(object):
    def __init__(self):
        self.running = False
        self._thread_list = []
        self._package = Radar_Package()
        self._serial = None
        self._update_callback = None
        self._fp_flag = False
        self.fp_points = []
        self.map = Map_360()

    def start(self, com_port, radar_type: str = "LD08", updae_callback=None):
        if self.running:
            self.stop()
        if radar_type == "LD08":
            baudrate = 115200
        elif radar_type == "LD06":
            baudrate = 230400
        else:
            raise ValueError("Unknown radar type")
        self._serial = serial.Serial(com_port, baudrate=baudrate)
        self._update_callback = updae_callback
        self.running = True
        thread = threading.Thread(target=self._read_serial_task)
        thread.daemon = True
        thread.start()
        self._thread_list.append(thread)
        logger.info("[RADAR] Listenning thread started")

    def stop(self):
        self.running = False
        for thread in self._thread_list:
            thread.join()
        if self._serial != None:
            self._serial.close()
        logger.info("[RADAR] Stopped all threads")

    def _read_serial_task(self):
        reading_flag = False
        start_bit = b"\x54\x2C"
        package_length = 45
        read_buffer = bytes()
        wait_buffer = bytes()
        while self.running:
            try:
                if not reading_flag:  # 等待包头
                    wait_buffer += self._serial.read(1)
                    if len(wait_buffer) >= 2:
                        if wait_buffer[-2:] == start_bit:
                            reading_flag = True
                            read_count = 0
                            wait_buffer = bytes()
                            read_buffer = start_bit
                else:  # 读取数据
                    read_buffer += self._serial.read(package_length)
                    reading_flag = False
                    resolve_radar_data(read_buffer, self._package)
                    self.map.update(self._package)
                    if self._fp_flag:
                        if self._fp_type == 0:
                            self._update_target_point(
                                self.map.find_nearest(*self._fp_arg)
                            )
                        elif self._fp_type == 1:
                            self._update_target_point(
                                self.map.find_nearest_with_ext_point_opt(*self._fp_arg)
                            )
                    if self._update_callback != None:
                        self._update_callback()
            except Exception as e:
                logger.error(f"[RADAR] Listenning thread error: {e}")

    def _init_radar_map(self):
        self._radar_map_img = np.zeros((600, 600, 3), dtype=np.uint8)
        a = np.sqrt(2) * 600
        b = (a - 600) / 2
        c = a - b
        b = int(b / np.sqrt(2))
        c = int(c / np.sqrt(2))
        cv2.line(self._radar_map_img, (b, b), (c, c), (255, 0, 0), 1)
        cv2.line(self._radar_map_img, (c, b), (b, c), (255, 0, 0), 1)
        cv2.line(self._radar_map_img, (300, 0), (300, 600), (255, 0, 0), 1)
        cv2.line(self._radar_map_img, (0, 300), (600, 300), (255, 0, 0), 1)
        cv2.circle(self._radar_map_img, (300, 300), 100, (255, 0, 0), 1)
        cv2.circle(self._radar_map_img, (300, 300), 200, (255, 0, 0), 1)
        cv2.circle(self._radar_map_img, (300, 300), 300, (255, 0, 0), 1)
        self.__radar_map_img_scale = 1
        self.__radar_map_info_angle = -1
        cv2.namedWindow("Radar Map", cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(
            "Radar Map",
            lambda *args, **kwargs: self._show_radar_map_on_mouse(*args, **kwargs),
        )

    def _show_radar_map_on_mouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_MOUSEWHEEL:
            if flags > 0:
                self.__radar_map_img_scale *= 1.1
            else:
                self.__radar_map_img_scale *= 0.9
            self.__radar_map_img_scale = min(max(0.001, self.__radar_map_img_scale), 2)
        elif event == cv2.EVENT_LBUTTONDOWN or (
            event == cv2.EVENT_MOUSEMOVE and flags & cv2.EVENT_FLAG_LBUTTON
        ):
            self.__radar_map_info_angle = (
                90 - np.arctan2(300 - y, x - 300) * 180 / np.pi
            ) % 360
            self.__radar_map_info_angle = int(self.__radar_map_info_angle)

    def show_radar_map(self):
        self._init_radar_map()
        while True:
            img_ = self._radar_map_img.copy()
            cv2.putText(
                img_,
                f"{100/self.__radar_map_img_scale:.0f}",
                (300, 220),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            cv2.putText(
                img_,
                f"{200/self.__radar_map_img_scale:.0f}",
                (300, 120),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            cv2.putText(
                img_,
                f"{300/self.__radar_map_img_scale:.0f}",
                (300, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.4,
                (255, 255, 0),
            )
            add_p = self.fp_points.copy()
            if self.__radar_map_info_angle != -1:
                cv2.putText(
                    img_,
                    f"Angle: {self.__radar_map_info_angle}",
                    (10, 540),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                cv2.putText(
                    img_,
                    f"Distance: {self.map.get_distance(self.__radar_map_info_angle)}",
                    (10, 560),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                point = self.map.get_point(self.__radar_map_info_angle)
                xy = point.to_xy()
                cv2.putText(
                    img_,
                    f"Position: ( {xy[0]:.2f} , {xy[1]:.2f} )",
                    (10, 580),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 0),
                )
                add_p = [point].extend(self.fp_points)
                pos = point.to_cv_xy() * self.__radar_map_img_scale + np.array(
                    [300, 300]
                )
                cv2.line(img_, (300, 300), (int(pos[0]), int(pos[1])), (255, 255, 0), 1)
            self.map.draw_on_cv_image(
                img_, scale=self.__radar_map_img_scale, add_points=add_p
            )
            cv2.imshow("Radar Map", img_)
            key = cv2.waitKey(int(1000 / 50))
            if key == 27:
                break
            elif key == ord("w"):
                self.__radar_map_img_scale *= 1.1
            elif key == ord("s"):
                self.__radar_map_img_scale *= 0.9

    def start_find_point(
        self,
        num: int,
        timeout: float,
        type: int,
        from_: int,
        to_: int,
        range_limit: int,
    ):
        self._fp_update_time = time.time()
        self._fp_timeout = timeout
        self.fp_timeout_flag = False
        self.fp_points = []
        self._fp_flag = True
        self._fp_type = type
        self._fp_arg = (from_, to_, num, range_limit)

    def stop_find_point(self):
        self._fp_flag = False

    def _update_target_point(self, points: list[Point_2D]):
        """
        更新目标点位置
        目标点超时判断
        """
        if self.fp_timeout_flag and len(points) > 0:
            self.fp_timeout_flag = False
        elif len(points) == 0:
            if (
                not self.fp_timeout_flag
                and time.time() - self._fp_update_time > self._fp_timeout
            ):
                self.fp_timeout_flag = True
                logger.warning("[Radar] lost point!")
            return
        self.fp_points = points
        self._fp_update_time = time.time()


if __name__ == "__main__":
    radar = LD_Radar()
    radar.start("COM8", "LD08")
    radar.show_radar_map()
    cv2.destroyAllWindows()
