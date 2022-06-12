# vscode-fold=2
import copy
import re
import threading
import time
import traceback

from logger import Exception_Catcher, logger
from SerCom32 import SerCom32

# small functions


def bytes_to_str(data): return " ".join([f"{b:02X}" for b in data])


def float_to_byte(value): return int(
    value * 100).to_bytes(2, "little", signed=True)[::-1]


def uint16_to_byte(value): return int(
    value).to_bytes(2, "little", signed=False)[::-1]
def int16_to_byte(value): return int(
    value).to_bytes(2, "little", signed=True)[::-1]


def uint8_to_byte(value): return int(value).to_bytes(1, "little", signed=False)
def b_int(b): return int.from_bytes(b, "little", signed=True)
def i_byte(i, length): return i.to_bytes(length, "little", signed=True)


class Byte_Var:
    __value = 0

    def __init__(self, ctype="u8", var_type=int, value_multiplier=1):
        self.reset_type(0, ctype, var_type, value_multiplier)

    def reset_type(self, value, ctype, var_type, value_multiplier=1):
        ctype_word_part = ctype[0]
        ctype_number_part = ctype[1:]
        if ctype_word_part.lower() == "u":
            self.__signed = False
        elif ctype_word_part.lower() == "s":
            self.__signed = True
        else:
            raise ValueError(f"Invalid ctype: {ctype}")
        if int(ctype_number_part) % 8 != 0:
            raise ValueError(f"Invalid ctype: {ctype}")
        if var_type not in [int, float, bool]:
            raise ValueError(f"Invalid var_type: {var_type}")
        self.__byte_length = int(int(ctype_number_part) // 8)
        self.__var_type = var_type
        self.__multiplier = value_multiplier
        self.__value = self.__var_type(value)

    def update_value(self, value):
        self.__value = self.__var_type(value)

    def value(self):
        return self.__value

    def update_byte(self, value):
        self.__value = (
            int.from_bytes(value, "little",
                           signed=self.__signed) * self.__multiplier
        )
        self.__value = self.__var_type(self.__value)

    def bytes(self):
        return int(self.__value / self.__multiplier).to_bytes(
            self.__byte_length, "little", signed=self.__signed
        )

    def byte_length(self):
        return self.__byte_length


class FC_Base_Comunication:
    def __init__(self, serial_port, bit_rate) -> None:
        self.running = False
        self.connected = False
        self.__start_bit = [0xAA, 0x22]
        self.__stop_bit = []
        self.__read_thread = None
        self.__state_update_callback = None
        self.__show_state_flag = False
        self.__ser_32 = SerCom32(serial_port, bit_rate)
        self.__sending_data = False
        logger.info("FC: Serial port opened")
        self.__set_option(0)
        self.__ser_32.readConfig(
            readByteStartBit=[0xAA, 0x55], byteDataCheck="sum")
        self.drone_state = {
            "rol": Byte_Var("s16", float, 0.01),  # deg
            "pit": Byte_Var("s16", float, 0.01),  # deg
            "yaw": Byte_Var("s16", float, 0.01),  # deg
            "alt_fused": Byte_Var("s32", int),  # cm
            "alt_add": Byte_Var("s32", int),  # cm
            "vel_x": Byte_Var("s16", int),  # cm/s
            "vel_y": Byte_Var("s16", int),  # cm/s
            "vel_z": Byte_Var("s16", int),  # cm/s
            "pos_x": Byte_Var("s32", int),  # cm
            "pos_y": Byte_Var("s32", int),  # cm
            "bat": Byte_Var("u16", float, 0.01),  # V
            "mode": Byte_Var("u8", int),
            "unlock": Byte_Var("u8", bool),
            "CID": Byte_Var("u8", int),
        }

    def start_listen_serial(self, show_state=True, callback=None):
        self.running = True
        self.__state_update_callback = callback
        self.__show_state_flag = show_state
        self.__read_thread = threading.Thread(target=self.__read_serial_task)
        self.__read_thread.start()

    def quit(self) -> None:
        self.running = False
        if self.__read_thread:
            self.__read_thread.join()
        self.__ser_32.close()
        logger.info("FC: Threads closed, FC offline")

    def __set_option(self, option: int) -> None:
        self.__ser_32.sendConfig(
            startBit=self.__start_bit,
            optionBit=[option],
            stopBit=self.__stop_bit,
            useSumCheck=True,
        )

    def send_32_from_data(self, data, option: int) -> None:
        while self.__sending_data:
            time.sleep(0.01)  # wait for previous data to be sent
        self.__sending_data = True
        self.__set_option(option)
        _ = self.__ser_32.send_from_data(data)
        self.__sending_data = False
        return _

    def __read_serial_task(self):
        logger.info("FC: Read thread started")
        last_heartbeat_time = time.time()
        while self.running:
            try:
                if self.__ser_32.read_byte_serial():
                    data = self.__ser_32.rx_byte_data
                    # logger.debug(f"FC: Read: {bytes_to_str(data)}")
                    self.__update_state(data)
            except Exception as e:
                logger.error(
                    f"FC: Read thread exception: {traceback.format_exc()}")
            if time.time() - last_heartbeat_time > 0.5:
                self.send_32_from_data([0x01], 0)  # 心跳包

    def __update_state(self, data):
        try:
            index = 0
            for key, value in self.drone_state.items():
                length = value.byte_length()
                self.drone_state[key].update_byte(data[index: index + length])
                index += length
            if not self.connected:
                self.connected = True
                logger.info("FC: Connected")
            if self.__state_update_callback is not None:
                self.__state_update_callback(self.drone_state)
            if self.__show_state_flag:
                self.__show_state()
        except Exception as e:
            logger.error(
                f"FC: Update state exception: {traceback.format_exc()}")

    def __show_state(self):
        RED = "\033[1;31m"
        GREEN = "\033[1;32m"
        YELLOW = "\033[1;33m"
        BLUE = "\033[1;34m"
        CYAN = "\033[1;36m"
        PURPLE = "\033[1;35m"
        RESET = "\033[0m"
        text = ""
        text += (
            " ".join(
                [
                    f"{YELLOW}{((key[:2]+key[-1]))}: {f'{GREEN}√ ' if value.value() else f'{RED}x {RESET}'}"
                    if type(value.value()) == bool
                    else f"{YELLOW}{((key[:2]+key[-1]))}: {CYAN}{value.value():^3d}{RESET}"
                    for key, value in self.drone_state.items()
                    if type(value.value()) != float
                ]
            )
            + " "
            + " ".join(
                [
                    f"{YELLOW}{((key[:2]+key[-1]))}:{CYAN}{value.value():^7.02f}{RESET}"
                    for key, value in self.drone_state.items()
                    if type(value.value()) == float
                ]
            )
        )
        print(f"\r {text}\r", end="")


class FC_protocol(FC_Base_Comunication):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.__byte_temp1 = Byte_Var()
        self.__byte_temp2 = Byte_Var()
        self.__byte_temp3 = Byte_Var()

    def __send_32_command(self, suboption: int, data: bytes = b"") -> None:
        suboption = int(suboption).to_bytes(1, "little")
        data_to_send = suboption + data
        sended = self.send_32_from_data(data_to_send, 0x01)
        logger.debug(f"FC: Send: {bytes_to_str(sended)}")

    def set_rgb_led(self, r: int, g: int, b: int) -> None:
        """
        设置由32控制的RGB LED
        r,g,b: 0-255
        """
        r = int(r).to_bytes(1, "little")
        g = int(g).to_bytes(1, "little")
        b = int(b).to_bytes(1, "little")
        self.__send_32_command(0x01, r + g + b)

    def send_general_position(self, x: int, y: int, z: int) -> None:
        """
        通用数据传感器回传
        x,y,z: cm
        """
        self.__byte_temp1.reset_type(x, "s32", int)
        self.__byte_temp2.reset_type(y, "s32", int)
        self.__byte_temp3.reset_type(z, "s32", int)
        self.__send_32_command(0x02, self.__byte_temp1.bytes() +
                               self.__byte_temp2.bytes() + self.__byte_temp3.bytes())

    def reset_position_prediction(self):
        """
        复位位置融合预测(伪造通用位置传感器)
        """
        self.send_general_position(0, 0, 0)

    def __send_command_frame(self, CID: int, CMD0: int, CMD1: int, CMD_data=b""):
        CID = int(CID).to_bytes(1, "little")
        CMD0 = int(CMD0).to_bytes(1, "little")
        CMD1 = int(CMD1).to_bytes(1, "little")
        CMD_data = bytes(CMD_data)
        if len(CMD_data) < 8:
            CMD_data += b"\x00" * (8 - len(CMD_data))
        data_to_send = CID + CMD0 + CMD1 + CMD_data
        sended = self.send_32_from_data(data_to_send, 0x02)
        logger.debug(f"FC: Send: {bytes_to_str(sended)}")

    def set_flight_mode(self, mode: int) -> None:
        """
        设置飞行模式:
        0: 姿态自稳 (危险,禁用)
        1: 定高
        2: 定点
        3: 程控
        """
        if mode not in [1, 2, 3]:
            raise ValueError("mode must be 1,2,3")
        self.__byte_temp1.reset_type(mode, "u8", int)
        self.__send_command_frame(0x01, 0x01, 0x01, self.__byte_temp1.bytes())

    def unlock(self) -> None:
        """
        解锁电机
        """
        self.__send_command_frame(0x10, 0x00, 0x01)

    def lock(self) -> None:
        """
        锁定电机 / 紧急锁浆
        """
        self.__send_command_frame(0x10, 0x00, 0x02)

    def stablize(self) -> None:
        """
        恢复定点悬停, 将终止正在进行的所有控制
        """
        self.__send_command_frame(0x10, 0x00, 0x04)

    def take_off(self) -> None:
        """
        一键起飞
        """
        self.__send_command_frame(0x10, 0x00, 0x05)

    def land(self) -> None:
        """
        一键降落
        """
        self.__send_command_frame(0x10, 0x00, 0x06)

    def horizontal_move(self, distance: int, speed: int, direction: int) -> None:
        """
        水平移动:
        移动距离:0-10000 cm
        移动速度:10-300 cm/s
        移动方向:0-359 度 (当前机头为0参考,顺时针)
        """
        self.__byte_temp1.reset_type(distance, "u16", int)
        self.__byte_temp2.reset_type(speed, "u16", int)
        self.__byte_temp3.reset_type(direction, "u16", int)
        self.__send_command_frame(
            0x10,
            0x02,
            0x03,
            self.__byte_temp1.bytes()
            + self.__byte_temp2.bytes()
            + self.__byte_temp3.bytes(),
        )

    def go_up(self, distance: int, speed: int) -> None:
        """
        上升:
        上升距离:0-10000 cm
        上升速度:10-300 cm/s
        """
        self.__byte_temp1.reset_type(distance, "u16", int)
        self.__byte_temp2.reset_type(speed, "u16", int)
        self.__send_command_frame(
            0x10, 0x02, 0x01, self.__byte_temp1.bytes() + self.__byte_temp2.bytes()
        )

    def go_down(self, distance: int, speed: int) -> None:
        """
        下降:
        下降距离:0-10000 cm
        下降速度:10-300 cm/s
        """
        self.__byte_temp1.reset_type(distance, "u16", int)
        self.__byte_temp2.reset_type(speed, "u16", int)
        self.__send_command_frame(
            0x10, 0x02, 0x02, self.__byte_temp1.bytes() + self.__byte_temp2.bytes()
        )

    def turn_left(self, deg: int, speed: int) -> None:
        """
        左转:
        左转角度:0-359 度
        左转速度:5-90 deg/s
        """
        self.__byte_temp1.reset_type(deg, "u16", int)
        self.__byte_temp2.reset_type(speed, "u16", int)
        self.__send_command_frame(
            0x10, 0x02, 0x07, self.__byte_temp1.bytes() + self.__byte_temp2.bytes()
        )

    def turn_right(self, deg: int, speed: int) -> None:
        """
        右转:
        右转角度:0-359 度
        右转速度:5-90 deg/s
        """
        self.__byte_temp1.reset_type(deg, "u16", int)
        self.__byte_temp2.reset_type(speed, "u16", int)
        self.__send_command_frame(
            0x10, 0x02, 0x08, self.__byte_temp1.bytes() + self.__byte_temp2.bytes()
        )

    def set_height(self, source: int, height: int, speed: int) -> None:
        """
        设置高度:
        高度源: 0:融合高度 1:激光高度
        高度:0-10000 cm
        速度:10-300 cm/s
        """
        if source == 0:
            current_height = self.drone_state["alt_fused"].value()
        elif source == 1:
            current_height = self.drone_state["alt_add"].value()
        if height < current_height:
            self.go_down(current_height - height, speed)
        elif height > current_height:
            self.go_up(height - current_height, speed)

    def set_yaw(self, yaw: int, speed: int) -> None:
        """
        设置偏航角:
        偏航角:-180-180 度
        偏航速度:5-90 deg/s
        """
        current_yaw = self.drone_state["yaw"].value()
        if yaw < current_yaw:
            left_turn_deg = abs(current_yaw - yaw)
            right_turn_deg = abs(360 - left_turn_deg)
        else:
            right_turn_deg = abs(current_yaw - yaw)
            left_turn_deg = abs(360 - right_turn_deg)
        if left_turn_deg < right_turn_deg:
            self.turn_left(left_turn_deg, speed)
        else:
            self.turn_right(right_turn_deg, speed)

    def set_target_position(self, x: int, y: int) -> None:
        """
        设置目标位置:
        x:+-100000 cm
        y:+-100000 cm
        """
        self.__byte_temp1.reset_type(x, "s32", int)
        self.__byte_temp2.reset_type(y, "s32", int)
        self.__send_command_frame(
            0x10, 0x01, 0x01, self.__byte_temp1.bytes() + self.__byte_temp2.bytes()
        )

    def set_target_height(self, height: int) -> None:
        """
        设置目标高度:
        目标对地高度:+100000 cm
        """
        if height < 0:
            height = 0
        self.__byte_temp1.reset_type(height, "s32", int)
        self.__send_command_frame(0x10, 0x01, 0x02, self.__byte_temp1.bytes())


class FC_Udp_Server(FC_Base_Comunication):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)


class FC_Controller(FC_Udp_Server, FC_protocol):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)


if __name__ == "__main__":
    import random
    fc = FC_Controller("COM23", 500000)
    try:
        fc.start_listen_serial()
        try:
            horizontal_distance = 100
            horizontal_speed = 25
            vertical_distance = 20
            vertical_speed = 20
            yaw_deg = 30
            yaw_speed = 30
            while True:
                k = input()
                if len(k) > 1:
                    k = k[0]
                if k == "t":
                    break
                elif k.isdigit():
                    fc.set_flight_mode(int(k))
                elif k == "o":
                    fc.unlock()
                elif k == "p":
                    fc.lock()
                elif k == "z":
                    fc.stablize()
                elif k == "x":
                    fc.takeoff()
                elif k == "c":
                    fc.land()
                elif k == "w":
                    fc.horizontal_move(horizontal_distance,
                                       horizontal_speed, 0)
                elif k == "s":
                    fc.horizontal_move(horizontal_distance,
                                       horizontal_speed, 180)
                elif k == "a":
                    fc.horizontal_move(horizontal_distance,
                                       horizontal_speed, 270)
                elif k == "d":
                    fc.horizontal_move(horizontal_distance,
                                       horizontal_speed, 90)
                elif k == "q":
                    fc.turn_left(yaw_deg, yaw_speed)
                elif k == "e":
                    fc.turn_right(yaw_deg, yaw_speed)
                elif k == "r":
                    fc.go_up(vertical_distance, vertical_speed)
                elif k == "f":
                    fc.go_down(vertical_distance, vertical_speed)
                elif k == 'i':
                    fc.reset_position_prediction()
                elif k == 'u':
                    r, g, b = random.randint(0, 255), random.randint(
                        0, 255), random.randint(0, 255)
                    logger.info(f"rgb:{r} {g} {b}")
                    fc.set_rgb_led(r, g, b)

        except KeyboardInterrupt:
            logger.info("FC: Keyboard interrupt")
    except Exception as e:
        logger.error(f"FC: Main loop exception: {traceback.format_exc()}")
    finally:
        fc.quit()
