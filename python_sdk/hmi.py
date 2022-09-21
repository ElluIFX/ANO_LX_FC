# 陶晶驰串口屏驱动
import time
import traceback

import serial


class HMI(object):
    def __init__(self, port, baudrate: int = 115200):
        self._port = port
        self._baudrate = baudrate
        self._serial = serial.Serial(port, baudrate)
        self._buffer = bytes()
        self._timer = time.time()
        self._flag = False

    def command(self, data: str):
        try:
            send = data.encode("gbk") + b"\xFF\xFF\xFF"
            self._serial.write(send)
        except Exception as e:
            traceback.print_exc()

    def read(self, timeout: float = 0.01) -> str:
        try:
            if self._serial.in_waiting > 0:
                if not self._flag:
                    self._flag = True
                    self._buffer = bytes()
                self._buffer += self._serial.read(self._serial.in_waiting)
                self._timer = time.time()
            if self._flag and time.time() - self._timer > timeout:
                self._flag = False
                data = self._buffer.decode()
                return data
            return None
        except Exception as e:
            traceback.print_exc()
            return None

    def info(self, data: str):
        data = data.replace("\n", "\r\n")
        self.command(f'info.txt="{data}"')

    def stop(self):
        self._serial.close()
