import datetime
import os
import socket
import struct
import time

import cv2
import numpy
from logger import Exception_Catcher, logger


class fps_counter:
    def __init__(self, max_sample=40) -> None:
        self.t = time.time()
        self.max_sample = max_sample
        self.t_list = []

    def update(self) -> None:
        self.t_list.append(time.time() - self.t)
        self.t = time.time()
        if len(self.t_list) > self.max_sample:
            self.t_list.pop(0)

    def get(self) -> float:
        length = len(self.t_list)
        sum_t = sum(self.t_list)
        if length == 0:
            return 0.0
        else:
            return length / sum_t


class net_speed_counter:
    def __init__(self, refreshTime=0.5) -> None:
        self.t = time.time()
        self.speed = 0.0
        self.sum = 0.0
        self.refreshTime = refreshTime

    def update(self, value) -> None:
        self.sum += value
        if time.time() - self.t > self.refreshTime:
            self.speed = self.sum / self.refreshTime
            self.t = time.time()
            self.sum = 0.0
            self.count = 0

    def getBps(self) -> float:
        return self.speed

    def getKbps(self) -> float:
        return self.speed / 1024

    def getKbit(self) -> float:
        return self.speed / 1024 / 8

    def getMbps(self) -> float:
        return self.speed / 1024 / 1024

    def getMbit(self) -> float:
        return self.speed / 1024 / 1024 / 8


class RT_Camera_Client:
    def __init__(
        self,
        IP,
        Port,
        Resolution=(640, 480),
        Fps=30,
        Quality=95,
        Paste_FPS=False,
        Terminate_server=False,
    ):
        self.addr_port = (IP, Port)
        self.resolution = Resolution
        self.fps = Fps
        self.quality = Quality
        self.timeout = 20
        self.paste_fps = Paste_FPS
        self.terminate = Terminate_server

    @Exception_Catcher
    def Set_socket(self):
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.client.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    @Exception_Catcher
    def Socket_Connect(self):
        self.Set_socket()
        self.client.connect(self.addr_port)
        logger.info("IP is %s:%d" % (self.addr_port[0], self.addr_port[1]))
        self.client.settimeout(self.timeout)
        self.Init_Reciver()

    def Send_Option(self):
        data = struct.pack(
            "!iiiiii",
            int(self.resolution[0]),
            int(self.resolution[1]),
            int(self.fps),
            int(self.quality),
            int(self.paste_fps),
            int(self.terminate),
        )
        logger.info(
            f"Send option: {self.resolution[0]}x{self.resolution[1]}@{self.fps}, {self.quality}% quality, Paste_FPS={self.paste_fps}, Terminate_server={self.terminate}"
        )
        self.client.send(data)

    @Exception_Catcher
    def Init_Reciver(self):
        # 按照格式打包发送帧数和分辨率
        self.Send_Option()
        if self.terminate:
            logger.info("Terminate server")
            return
        self.ip = self.addr_port[0]
        logger.info("%s is ready" % self.ip)
        self.packet_header = b"\x12\x23\x34\x45\x00\xff"
        self.header_len = len(self.packet_header)
        self.fps = fps_counter()
        self.netC = net_speed_counter(0.4)

    @Exception_Catcher
    def capture_frame(self):
        # 接收图像数据
        counter = 0
        self.header_temp_buf = b""
        self.header_temp_buf += self.client.recv(self.header_len)
        while self.header_temp_buf[-self.header_len :] != self.packet_header:
            self.header_temp_buf += self.client.recv(1)
            counter += 1
            if counter > 1000000:
                raise Exception("Waiting for header timeout")
        info = struct.unpack("!i", self.client.recv(struct.calcsize("!i")))
        data_size = info[0]
        if data_size < 0 or data_size > 1000000:
            logger.error(f"Size Error: {data_size}")
            return None
        self.netC.update(data_size * 8)
        try:
            self.buf = b""  # 代表bytes类型
            counter = 0
            while len(self.buf) < data_size:
                self.buf += self.client.recv(data_size)
                counter += 1
                if counter > 1000000:
                    raise Exception("Waiting for image timeout")
            # self.buf=self.buf[:data_size]
            data = numpy.frombuffer(self.buf, dtype="uint8")
            image = cv2.imdecode(data, 1)
            self.fps.update()
            return image
        except Exception as e:
            logger.error(f"Error: {e}")
            return None

    @Exception_Catcher
    def RT_Image_Recv(self):
        blk = None
        last_res = None
        while 1:
            # wait for header
            time.sleep(0.01)  # reduce CPU usage
            try:
                self.image = self.capture_frame()
                if self.image is None:
                    continue
                self.original_image = self.image.copy()
                width, height = self.image.shape[1], self.image.shape[0]
                if last_res is None:
                    last_res = width * height
                if last_res != width * height:
                    blk = None
                if blk is None:
                    blk = numpy.zeros(self.image.shape, numpy.uint8)
                    cv2.rectangle(
                        blk, (0, height - 50), (220, height), (255, 255, 255), -1
                    )
                    line_color = (255, 255, 255)
                    length = min(height, width) // 3
                    cv2.line(
                        blk,
                        ((width - length) // 2, height // 2),
                        ((width + length) // 2, height // 2),
                        line_color,
                        1,
                    )
                    cv2.line(
                        blk,
                        (width // 2, (height - length) // 2),
                        (width // 2, (height + length) // 2),
                        line_color,
                        1,
                    )
                    length = min(height, width)
                    cv2.rectangle(
                        blk,
                        ((width - length) // 2, (height + length) // 2),
                        ((width + length) // 2, (height - length) // 2),
                        line_color,
                        1,
                    )
                self.image = cv2.addWeighted(self.image, 1.0, blk, 0.5, 1)
                self.image = cv2.putText(
                    self.image,
                    f"{self.ip} {width}x{height}",
                    (10, height - 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (20, 20, 20),
                    1,
                )
                self.image = cv2.putText(
                    self.image,
                    f"Fps:{self.fps.get():05.2f} Net:{self.netC.getMbit():.3f}Mbit",
                    (10, height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (20, 20, 20),
                    1,
                )
                cv2.imshow(self.ip + " Main Monitor", self.image)
            except Exception as e:
                logger.error(e)
            finally:
                key = cv2.waitKey(1)
                if key == 27:  # 按‘ESC’（27）退出
                    self.client.close()
                    cv2.destroyAllWindows()
                    break
                elif key == ord("s"):
                    time_string = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    if not os.path.exists("cv_saved_images"):
                        os.mkdir("cv_saved_images")
                    cv2.imwrite(
                        f"cv_saved_images/{time_string}.jpg", self.original_image
                    )
                    logger.info(f"Saved image {time_string}.jpg")
                    time.sleep(0.1)


class KCF_Tracker:  # OPENCV KCF Tracker
    def __init__(self) -> None:
        self.frame = None
        self.selection = None
        self.drag_start = None
        self.track_window = None
        self.track_start = False
        cv2.namedWindow("KCFTracker", cv2.WINDOW_KEEPRATIO | cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback("KCFTracker", self.onmouse)

    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (x, y)
            self.track_start = False
        if self.drag_start:
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:
            self.drag_start = None
            self.selection = None
            self.track_window = (xmin, ymin, xmax - xmin, ymax - ymin)
            if (
                self.track_window
                and self.track_window[2] > 0
                and self.track_window[3] > 0
            ):
                self.track_start = True
                self.tracker = cv2.TrackerKCF_create()
                self.tracker.init(self.frame, self.track_window)
            else:
                self.track_start = False
                self.track_window = None
                self.tracker = None

    def process(self, frame):
        self.frame = frame.copy()
        if self.selection:
            x0, y0, x1, y1 = self.selection
            cv2.rectangle(self.frame, (x0, y0), (x1, y1), (255, 0, 0), 2, 1)
        self.track_ok = None
        if self.track_start:
            self.track_ok, bbox = self.tracker.update(frame)
        if self.track_ok:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(self.frame, p1, p2, (255, 0, 0), 2, 1)
        elif not self.track_start:
            cv2.putText(
                self.frame,
                "No tracking target selected",
                (0, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )
        elif not self.track_ok:
            cv2.putText(
                self.frame,
                "Tracking failed",
                (0, 20),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.75,
                (0, 0, 255),
                2,
            )
        cv2.imshow("KCFTracker", self.frame)


"""
帧率组合:
640x360@30
800x600@20
1024x768@10
1280x720@10
1920x1080@5
"""

if __name__ == "__main__":
    camera = RT_Camera_Client(
        IP="192.168.137.27",
        # IP="raspberrypi",
        Port=6756,
        Resolution=(800, 600),
        Fps=60,
        Paste_FPS=0,
        Quality=70,
        Terminate_server=0,
    )
    logger.info("Waiting for server...")
    while True:
        try:
            camera.Socket_Connect()
        except:
            continue
        break
    camera.RT_Image_Recv()
