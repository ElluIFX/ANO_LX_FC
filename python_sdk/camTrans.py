# 服务器端

import socket
import struct
import threading
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


class RT_Camera_Server:
    def __init__(self, port=7777):
        self.S_addr_port = ("", port)
        self.option = (0, 0, 0)
        self.timeout = 5
        self.Set_Socket()

    @Exception_Catcher
    def Accept(self):
        self.client, self.addr = self.server.accept()
        self.client.settimeout(self.timeout)
        logger.info(f"Connected by: {str(self.addr)}")

    # 设置套接字
    @Exception_Catcher
    def Set_Socket(self):
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # 端口可复用
        self.server.bind(self.S_addr_port)
        self.server.listen(5)
        logger.info(f"Server online: {str(self.S_addr_port[1])}")

    @Exception_Catcher
    def set_option(self):
        # 按格式解码，确定帧数和分辨率
        info = struct.unpack("!iiiiii", self.client.recv(struct.calcsize("!iiiiii")))
        self.option = {
            "W": int(info[0]),  # 宽度
            "H": int(info[1]),  # 高度
            "F": int(info[2]),  # 帧率
            "Q": int(info[3]),  # 传输质量
            "P": int(info[4]),  # 是否显示帧率
            "T": int(info[5]),  # 是否终止传输
        }
        self.paste_fps = self.option["P"]
        logger.info(f"Recived option: {str(self.option)}")

    @Exception_Catcher
    def RT_Image_Trans(self) -> bool:
        logger.info("Waiting for client...")
        self.Accept()
        self.set_option()
        if self.option["T"]:
            logger.info("Received terminate signal")
            return 1
        logger.info("Initiate the camera")
        cam = cv2.VideoCapture(0)
        cam.set(3, self.option["W"])  # 设置宽度
        cam.set(4, self.option["H"])  # 设置高度
        cam.set(5, self.option["F"])  # 设置帧率
        W, H, F = cam.get(3), cam.get(4), cam.get(5)
        logger.info(f"Camera info: {str(W)}x{str(H)}@{str(F)}")
        img_param = [
            int(cv2.IMWRITE_JPEG_QUALITY),
            self.option["Q"],
        ]  # 设置传送图像格式、帧数
        logger.info("Start to send image")
        packet_header = b"\x12\x23\x34\x45\x00\xff"
        fps = fps_counter()
        while 1:
            time.sleep(0.01)
            self.img = cam.read()[1]
            fps.update()
            if self.paste_fps:
                cv2.putText(
                    self.img,
                    f"{fps.get():.1f}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1,
                    (0, 0, 255),
                    1,
                )
            self.img_encode = cv2.imencode(".jpg", self.img, img_param)[1]
            self.img_code = numpy.array(self.img_encode)  # 转换成矩阵
            self.img_data = self.img_code.tobytes()  # 生成相应的字符串
            try:
                self.data = packet_header + struct.pack("!i", len(self.img_data))
                self.client.send(self.data + self.img_data)
            except:
                logger.info("Exit the connection")
                return 0
        return 0


server = RT_Camera_Server(6756)


def Loop_Thread():
    while 1:
        time.sleep(1)
        ret = server.RT_Image_Trans()
        if ret:
            break


if __name__ == "__main__":
    th = threading.Thread(target=Loop_Thread)
    th.daemon = True
    th.start()
    try:
        while 1:
            time.sleep(1)
            if not th.is_alive():
                break
    finally:
        # th.join()
        logger.warning("Server closed")
