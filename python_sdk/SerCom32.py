import time
from copy import copy
from sys import byteorder as sysByteorder
from typing import Literal

import serial


class SerCom32:
    def __init__(self, port, baudrate, timeout=0.5, byteOrder=sysByteorder):
        self.ser = serial.Serial(port, baudrate, timeout=timeout)
        self.readTick = time.monotonic()
        self.readBuffer = bytes()
        self.readSaveBuffer = bytes()
        self.readByteGoingFlag = False
        self.readByteBuffer = bytes()
        self.readByteSaveBuffer = bytes()
        self.sendConfig()
        self.readConfig()
        self.packCount = 0
        self.packLength = 0
        self.byteOrder = byteOrder
        self.trash_buffer = []

    def sendConfig(self, startBit=[], stopBit=[], optionBit=[], useSumCheck=False):
        self.startBit = startBit
        self.optionBit = optionBit
        self.stopBit = stopBit
        self.useSumCheck = useSumCheck

    def readConfig(
        self,
        readTimeout=0.01,
        packLengthAdd=0,
        packLengthMulti=1,
        readByteStartBit=[],
        byteDataCheck: Literal["none", "sum"] = "none",
    ):
        self.readOvertime = readTimeout
        self.readByteStartBit = readByteStartBit
        self.byteDataCheck = byteDataCheck
        self.packLengthAdd = packLengthAdd
        self.packLengthMulti = packLengthMulti

    def read_serial(self):
        if (
            time.monotonic() - self.readTick > self.readOvertime
            and len(self.readBuffer) != 0
        ):
            self.readSaveBuffer = copy(self.readBuffer)
            self.readBuffer = bytes()
            return 1
        if self.ser.in_waiting > 0:
            self.readBuffer += self.ser.read(self.ser.in_waiting)
            self.readTick = time.monotonic()
            self.ser.flushInput()
        time.sleep(0.002)
        return 0

    @property
    def rx_data(self) -> bytes:
        return self.readSaveBuffer

    def check_byte_data(self):
        if self.byteDataCheck == "none":
            return 1
        elif self.byteDataCheck == "sum":
            length = len(self.readByteBuffer)
            checksum = 0
            for i in self.readByteStartBit:
                checksum += i
                checksum &= 0xFF
            checksum += self.packLengthBit
            checksum &= 0xFF
            for i in range(0, length):
                checksum += int.from_bytes(
                    self.readByteBuffer[i : i + 1],
                    byteorder=self.byteOrder,
                    signed=False,
                )
                checksum &= 0xFF
            receivedChecksum = int.from_bytes(
                self.ser.read(1), byteorder=self.byteOrder, signed=False
            )
            if receivedChecksum == checksum:
                return 1
            return 0
        return 0

    def read_byte_serial(self):
        tmp = 0x00
        while self.ser.in_waiting > 0:
            tmp = self.ser.read(1)
            if not self.readByteGoingFlag:
                self.trash_buffer.append(tmp)
                if len(self.trash_buffer) > 20:
                    self.trash_buffer.pop(0)
                if len(self.trash_buffer) >= len(self.readByteStartBit):
                    if bytes(
                        [
                            int.from_bytes(x, byteorder=self.byteOrder, signed=False)
                            for x in self.trash_buffer[-len(self.readByteStartBit) :]
                        ]
                    ) == bytes(self.readByteStartBit):
                        self.readByteGoingFlag = True
                        self.readByteBuffer = bytes()
                        self.packCount = 0
                        self.packLength = -1
                        self.trash_buffer = []
                continue
            if self.packLength == -1:
                self.packLengthBit = int.from_bytes(tmp, self.byteOrder, signed=False)
                self.packLength = self.packLengthBit & 0b11111111
                continue
            if self.readByteGoingFlag:
                self.packCount += 1
                self.readByteBuffer += tmp
                if (
                    self.packCount
                    >= self.packLength * self.packLengthMulti + self.packLengthAdd
                ):
                    self.readByteGoingFlag = False
                    if self.check_byte_data():
                        self.readByteSaveBuffer = copy(self.readByteBuffer)
                        self.readByteBuffer = bytes()
                        return 1
                    else:
                        self.readByteBuffer = bytes()
                        return 0
        return 0

    @property
    def rx_byte_data(self):
        return self.readByteSaveBuffer

    def close(self):
        if self.ser != None:
            self.ser.close()
            self.ser = None

    def write_str_serial(self, data: str):
        self.ser.write(data.encode("utf-8"))
        self.ser.flush()

    def write_bytes_serial(self, data: bytes):
        self.ser.write(data)
        self.ser.flush()

    def send_from_data(self, data):
        data_ = copy(data)
        if isinstance(data_, list):
            data_ = bytes(data_)
        if not isinstance(data_, bytes):
            raise TypeError("data must be bytes")
        len_as_byte = len(data_).to_bytes(1, self.byteOrder)
        send_data = (
            bytes(self.startBit)
            + bytes(self.optionBit)
            + len_as_byte
            + data_
            + bytes(self.stopBit)
        )
        if self.useSumCheck:
            checksum = 0
            for i in range(0, len(send_data)):
                checksum += send_data[i]
                checksum &= 0xFF
            send_data += checksum.to_bytes(1, self.byteOrder)
        self.ser.write(send_data)
        self.ser.flush()
        return send_data


if __name__ == "__main__":
    ser = SerCom32("COM23", 115200)
    ser.sendConfig(
        startBit=[0xAA, 0x22],
        optionBit=[0x01],
        stopBit=[],
        useSumCheck=True,
    )
    import struct

    bytes_to_str = lambda data: " ".join([f"{b:02X}" for b in data])
    float_to_byte = lambda value: int(value * 100).to_bytes(2, byteorder="little")[::-1]
    int16_to_byte = lambda value: int(value).to_bytes(2, byteorder="little")[::-1]
    data = b""
    data += b"\x10"
    data += int16_to_byte(0x0001)
    data += int16_to_byte(2345)
    data += int16_to_byte(6789)
    sended = ser.send_from_data(data)
    print(bytes_to_str(sended))
