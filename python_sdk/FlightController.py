import threading
import time
import traceback

import numpy as np
from logger import Exception_Catcher, logger
from SerCom32 import SerCom32


def bytes_to_str(data):
    return " ".join([f"{b:02X}" for b in data])


class Byte_Var:
    """
    C-like byte类型变量与python泛型变量的转换类
    使用时直接操作成员bytes和value即可
    """

    __value = 0
    __last_update_time = 0
    __byte_length = 0
    __multiplier = 1
    __var_type = None

    def __init__(self, ctype="u8", var_type=int, value_multiplier=1):
        self.reset(0, ctype, var_type, value_multiplier)

    def reset(self, init_value, ctype: str, py_var_type, value_multiplier=1):
        """重置变量

        Args:
            init_value (_type_): 初始值(浮点值或整数值)
            ctype (str): C-like类型(如u8, u16, u32, s8, s16, s32)
            py_var_type (_type_): python类型(如int, float)
            value_multiplier (int, optional): 值在从byte向python转换时的乘数. Defaults to 1.
        """
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
        if py_var_type not in [int, float, bool]:
            raise ValueError(f"Invalid var_type: {py_var_type}")
        self.__byte_length = int(int(ctype_number_part) // 8)
        self.__var_type = py_var_type
        self.__multiplier = value_multiplier
        self.__value = self.__var_type(init_value)
        self.__last_update_time = time.time()
        return self

    @property
    def value(self):
        return self.__value

    @value.setter
    def value(self, value):
        self.__value = self.__var_type(value)
        self.__last_update_time = time.time()

    @property
    def bytes(self):
        return int(self.__value / self.__multiplier).to_bytes(
            self.__byte_length, "little", signed=self.__signed
        )

    @bytes.setter
    def bytes(self, value):
        self.__value = (
            int.from_bytes(value, "little", signed=self.__signed) * self.__multiplier
        )
        self.__value = self.__var_type(self.__value)
        self.__last_update_time = time.time()

    @property
    def byte_length(self):
        return self.__byte_length

    @byte_length.setter
    def byte_length(self, value):
        raise ValueError("byte_length is read-only")

    @property
    def last_update_time(self):
        return self.__last_update_time

    @last_update_time.setter
    def last_update_time(self, value):
        raise ValueError("last_update_time is read-only")


class FC_State_Struct:
    rol = Byte_Var("s16", float, 0.01)  # deg
    pit = Byte_Var("s16", float, 0.01)  # deg
    yaw = Byte_Var("s16", float, 0.01)  # deg
    alt_fused = Byte_Var("s32", int)  # cm
    alt_add = Byte_Var("s32", int)  # cm
    vel_x = Byte_Var("s16", int)  # cm/s
    vel_y = Byte_Var("s16", int)  # cm/s
    vel_z = Byte_Var("s16", int)  # cm/s
    pos_x = Byte_Var("s32", int)  # cm
    pos_y = Byte_Var("s32", int)  # cm
    bat = Byte_Var("u16", float, 0.01)  # V
    mode = Byte_Var("u8", int)  #
    unlock = Byte_Var("u8", bool)  #
    cid = Byte_Var("u8", int)  #

    alt = alt_add  # alias

    RECV_ORDER = [  # 数据包顺序
        rol,
        pit,
        yaw,
        alt_fused,
        alt_add,
        vel_x,
        vel_y,
        vel_z,
        pos_x,
        pos_y,
        bat,
        mode,
        unlock,
        cid,
    ]


class FC_Base_Comunication:
    def __init__(self, serial_port, bit_rate) -> None:
        self.running = False
        self.connected = False
        self.__start_bit = [0xAA, 0x22]
        self.__listen_thread = None
        self.__state_update_callback = None
        self.__print_state_flag = False
        self.__ser_32 = SerCom32(serial_port, bit_rate)
        self.__sending_data = False
        self.__waiting_ack = False
        self.__recivied_ack = None
        logger.info("FC: Serial port opened")
        self.__set_option(0)
        self.__ser_32.read_config(startBit=[0xAA, 0x55])
        self.state = FC_State_Struct()

    def start_listen_serial(self, print_state=True, callback=None):
        self.running = True
        self.__state_update_callback = callback
        self.__print_state_flag = print_state
        self.__listen_thread = threading.Thread(target=self.__listen_serial_task)
        self.__listen_thread.start()

    def quit(self) -> None:
        self.running = False
        if self.__listen_thread:
            self.__listen_thread.join()
        self.__ser_32.close()
        logger.info("FC: Threads closed, FC offline")

    def __set_option(self, option: int) -> None:
        self.__ser_32.send_config(
            startBit=self.__start_bit,
            optionBit=[option],
        )

    def send_32_from_data(
        self, data: bytes, option: int, need_ack: bool = False, ack_max_retry: int = 3
    ) -> None:
        while self.__sending_data:
            time.sleep(0.01)  # wait for previous data to be sent
        self.__sending_data = True
        if need_ack:
            if ack_max_retry < 0:
                # raise Exception("Wait ACK reached max retry")
                logger.error("Wait ACK reached max retry")
                return None
            self.__waiting_ack = True
            self.__recivied_ack = None
        self.__set_option(option)
        sended = self.__ser_32.write(data)
        self.__sending_data = False
        if need_ack:
            send_time = time.time()
            check_ack = (option + data[0]) & 0xFF
            while self.__waiting_ack:
                if time.time() - send_time > 1:
                    logger.warning("FC: ACK timeout, retrying")
                    return self.send_32_from_data(
                        data, option, need_ack, ack_max_retry - 1
                    )
                time.sleep(0.001)
            if self.__recivied_ack is None or self.__recivied_ack != check_ack:
                logger.warning("FC: ACK not received or invalid, retrying")
                return self.send_32_from_data(data, option, need_ack, ack_max_retry - 1)
        return sended

    def __listen_serial_task(self):
        logger.info("FC: listen serial thread started")
        last_heartbeat_time = time.time()
        while self.running:
            try:
                if self.__ser_32.read():
                    _data = self.__ser_32.rx_data
                    # logger.debug(f"FC: Read: {bytes_to_str(_data)}")
                    cmd = _data[0]
                    data = _data[1:]
                    if cmd == 0x01:  # 状态回传
                        self.__update_state(data)
                    elif cmd == 0x02:  # ACK返回
                        self.__recivied_ack = data[0]
                        self.__waiting_ack = False
            except Exception as e:
                logger.error(f"FC: listen serial exception: {traceback.format_exc()}")
            if time.time() - last_heartbeat_time > 0.25:
                self.send_32_from_data(b"\x01", 0x00)  # 心跳包
                last_heartbeat_time = time.time()

    def __update_state(self, recv_byte):
        try:
            index = 0
            for var in self.state.RECV_ORDER:
                length = var.byte_length
                var.bytes = recv_byte[index : index + length]
                index += length
            if not self.connected:
                self.connected = True
                logger.info("FC: Connected")
            if callable(self.__state_update_callback):
                self.__state_update_callback(self.state)
            if self.__print_state_flag:
                self.__print_state()
        except Exception as e:
            logger.error(f"FC: Update state exception: {traceback.format_exc()}")

    def __print_state(self):
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
                    f"{YELLOW}{((var.__name__[:2]+var.__name__[-1]))}: {f'{GREEN}√ ' if var.value else f'{RED}x {RESET}'}"
                    if type(var.value) == bool
                    else f"{YELLOW}{((var.__name__[:2]+var.__name__[-1]))}: {CYAN}{var.value:^3d}{RESET}"
                    for var in self.state.RECV_ORDER
                    if type(var.value) != float
                ]
            )
            + " "
            + " ".join(
                [
                    f"{YELLOW}{((var.__name__[:2]+var.__name__[-1]))}:{CYAN}{var.value:^7.02f}{RESET}"
                    for var in self.state.RECV_ORDER
                    if type(var.value) == float
                ]
            )
        )
        print(f"\r {text}\r", end="")


class FC_Protocol(FC_Base_Comunication):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.__byte_temp1 = Byte_Var()
        self.__byte_temp2 = Byte_Var()
        self.__byte_temp3 = Byte_Var()
        self.__base_pos_x = 0
        self.__base_pos_y = 0
        self.__base_yaw = 0

    def __send_32_command(self, suboption: int, data: bytes = b"") -> None:
        self.__byte_temp1.reset(suboption, "u8", int)
        data_to_send = self.__byte_temp1.bytes + data
        sended = self.send_32_from_data(data_to_send, 0x01)
        logger.debug(f"FC: Send: {bytes_to_str(sended)}")

    def set_rgb_led(self, r: int, g: int, b: int) -> None:
        """
        设置由32控制的RGB LED
        r,g,b: 0-255
        """
        self.__byte_temp1.reset(r, "u8", int)
        self.__byte_temp2.reset(g, "u8", int)
        self.__byte_temp3.reset(b, "u8", int)
        self.__send_32_command(
            0x01,
            self.__byte_temp1.bytes + self.__byte_temp2.bytes + self.__byte_temp3.bytes,
        )

    def send_general_position(self, x: int, y: int, z: int) -> None:
        """
        通用数据传感器回传
        x,y,z: cm
        """
        self.__byte_temp1.reset(x, "s32", int)
        self.__byte_temp2.reset(y, "s32", int)
        self.__byte_temp3.reset(z, "s32", int)
        self.__send_32_command(
            0x02,
            self.__byte_temp1.bytes + self.__byte_temp2.bytes + self.__byte_temp3.bytes,
        )

    def reset_position_prediction(self):
        """
        复位位置融合预测(伪造通用位置传感器)
        """
        self.send_general_position(0, 0, 0)

    def __send_imu_command_frame(self, CID: int, CMD0: int, CMD1: int, CMD_data=b""):
        self.__byte_temp1.reset(CID, "u8", int)
        self.__byte_temp2.reset(CMD0, "u8", int)
        self.__byte_temp3.reset(CMD1, "u8", int)
        bytes_data = bytes(CMD_data)
        if len(bytes_data) < 8:
            bytes_data += b"\x00" * (8 - len(bytes_data))
        if len(bytes_data) > 8:
            raise Exception("CMD_data length is too long")
        data_to_send = (
            self.__byte_temp1.bytes
            + self.__byte_temp2.bytes
            + self.__byte_temp3.bytes
            + bytes_data
        )
        sended = self.send_32_from_data(data_to_send, 0x02, need_ack=True)
        # logger.debug(f"FC: Send: {bytes_to_str(sended)}")

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
        self.__byte_temp1.reset(mode, "u8", int)
        self.__send_imu_command_frame(0x01, 0x01, 0x01, self.__byte_temp1.bytes)

    def unlock(self) -> None:
        """
        解锁电机
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x01)

    def lock(self) -> None:
        """
        锁定电机 / 紧急锁浆
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x02)

    def stablize(self) -> None:
        """
        恢复定点悬停, 将终止正在进行的所有控制
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x04)

    def take_off(self) -> None:
        """
        一键起飞
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x05)

    def land(self) -> None:
        """
        一键降落
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x06)

    def horizontal_move(self, distance: int, speed: int, direction: int) -> None:
        """
        水平移动:
        移动距离:0-10000 cm
        移动速度:10-300 cm/s
        移动方向:0-359 度 (当前机头为0参考,顺时针)
        """
        self.__byte_temp1.reset(distance, "u16", int)
        self.__byte_temp2.reset(speed, "u16", int)
        self.__byte_temp3.reset(direction, "u16", int)
        self.__send_imu_command_frame(
            0x10,
            0x02,
            0x03,
            self.__byte_temp1.bytes + self.__byte_temp2.bytes + self.__byte_temp3.bytes,
        )

    def cordinate_move(self, x: int, y: int, speed: int) -> None:
        """
        匿名坐标系下的水平移动:
        移动半径在0-10000 cm
        x,y: 匿名坐标系下的坐标
        speed: 移动速度:10-300 cm/s
        """
        div = y / x if x != 0 else np.inf
        target_deg = np.arctan(div) / np.pi * 180
        distance = np.sqrt(x**2 + y**2)
        if target_deg < 0:
            target_deg += 360
        self.horizontal_move(int(distance), speed, int(target_deg))

    def go_up(self, distance: int, speed: int) -> None:
        """
        上升:
        上升距离:0-10000 cm
        上升速度:10-300 cm/s
        """
        self.__byte_temp1.reset(distance, "u16", int)
        self.__byte_temp2.reset(speed, "u16", int)
        self.__send_imu_command_frame(
            0x10, 0x02, 0x01, self.__byte_temp1.bytes + self.__byte_temp2.bytes
        )

    def go_down(self, distance: int, speed: int) -> None:
        """
        下降:
        下降距离:0-10000 cm
        下降速度:10-300 cm/s
        """
        self.__byte_temp1.reset(distance, "u16", int)
        self.__byte_temp2.reset(speed, "u16", int)
        self.__send_imu_command_frame(
            0x10, 0x02, 0x02, self.__byte_temp1.bytes + self.__byte_temp2.bytes
        )

    def turn_left(self, deg: int, speed: int) -> None:
        """
        左转:
        左转角度:0-359 度
        左转速度:5-90 deg/s
        """
        self.__byte_temp1.reset(deg, "u16", int)
        self.__byte_temp2.reset(speed, "u16", int)
        self.__send_imu_command_frame(
            0x10, 0x02, 0x07, self.__byte_temp1.bytes + self.__byte_temp2.bytes
        )

    def turn_right(self, deg: int, speed: int) -> None:
        """
        右转:
        右转角度:0-359 度
        右转速度:5-90 deg/s
        """
        self.__byte_temp1.reset(deg, "u16", int)
        self.__byte_temp2.reset(speed, "u16", int)
        self.__send_imu_command_frame(
            0x10, 0x02, 0x08, self.__byte_temp1.bytes + self.__byte_temp2.bytes
        )

    def set_height(self, source: int, height: int, speed: int) -> None:
        """
        设置高度:
        高度源: 0:融合高度 1:激光高度
        高度:0-10000 cm
        速度:10-300 cm/s
        """
        if source == 0:
            current_height = self.state.alt_fused.value
        elif source == 1:
            current_height = self.state.alt_add.value
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
        current_yaw = self.state.yaw.value
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
        self.__byte_temp1.reset(x, "s32", int)
        self.__byte_temp2.reset(y, "s32", int)
        self.__send_imu_command_frame(
            0x10, 0x01, 0x01, self.__byte_temp1.bytes + self.__byte_temp2.bytes
        )

    def set_target_height(self, height: int) -> None:
        """
        设置目标高度:
        目标对地高度:+100000 cm
        """
        if height < 0:
            height = 0
        self.__byte_temp1.reset(height, "s32", int)
        self.__send_imu_command_frame(0x10, 0x01, 0x02, self.__byte_temp1.bytes)

    def reset_base_position(self) -> None:
        """
        重置基地参考点, 用于按坐标移动时的坐标系计算
        """
        self.__base_pos_x = self.state.pos_x.value
        self.__base_pos_y = self.state.pos_y.value
        self.__base_yaw = self.state.yaw.value
        logger.info(
            f"FC: reset base position to {self.__base_pos_x}, {self.__base_pos_y} @ {self.__base_yaw}"
        )

    def go_to_position_by_base(self, x: int, y: int) -> None:
        """
        以基地坐标为参考的移动:
        x,y:+-100000 cm

        匿名机身参考系(即调用本函数时的参考系):         机头为x+ 机身左侧y+ 机身上方z+
        匿名世界参考系(回传数据中位置偏移采用的参考系):  地磁北(实际上是融合后yaw=0的方向)为x+ 地磁西为y+ 天空为z+
        """
        base_deg = -self.__base_yaw / 180 * np.pi
        div = y / x if x != 0 else np.inf
        target_deg = np.arctan(div)
        deg = base_deg + target_deg
        distance = np.sqrt(x**2 + y**2)
        t_x = int(np.cos(deg) * distance + self.__base_pos_x)
        t_y = int(np.sin(deg) * distance + self.__base_pos_y)
        logger.info(f"FC: go to {t_x} {t_y}")
        self.set_target_position(t_x, t_y)

    def go_to_base(self) -> None:
        """
        回到基地
        """
        self.set_target_position(self.__base_pos_x, self.__base_pos_y)


class FC_Udp_Server(FC_Base_Comunication):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)


class FC_Controller(FC_Udp_Server, FC_Protocol):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)


if __name__ == "__main__":

    fc = FC_Controller("COM23", 500000)
    try:
        fc.start_listen_serial()
        try:
            horizontal_distance = 50
            horizontal_speed = 100
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
                    fc.take_off()
                elif k == "c":
                    fc.land()
                elif k == "w":
                    fc.horizontal_move(horizontal_distance, horizontal_speed, 0)
                elif k == "s":
                    fc.horizontal_move(horizontal_distance, horizontal_speed, 180)
                elif k == "a":
                    fc.horizontal_move(horizontal_distance, horizontal_speed, 270)
                elif k == "d":
                    fc.horizontal_move(horizontal_distance, horizontal_speed, 90)
                elif k == "q":
                    fc.turn_left(yaw_deg, yaw_speed)
                elif k == "e":
                    fc.turn_right(yaw_deg, yaw_speed)
                elif k == "r":
                    fc.go_up(vertical_distance, vertical_speed)
                elif k == "f":
                    fc.go_down(vertical_distance, vertical_speed)
                elif k == "k":
                    fc.reset_base_position()
                    fc.go_to_position(1, -100)
                elif k == "l":
                    fc.reset_base_position()
                    fc.set_target_height(100)
                elif k == "u":
                    fc.set_rgb_led(0x66, 0xCC, 0xFF)

        except KeyboardInterrupt:
            logger.info("FC: Keyboard interrupt")
    except Exception as e:
        logger.error(f"FC: Main loop exception: {traceback.format_exc()}")
    finally:
        fc.quit()
