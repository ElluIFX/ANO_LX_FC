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

    def __init__(self, ctype="u8", var_type=int, value_multiplier=1, name=None):
        self.reset(0, ctype, var_type, value_multiplier)
        self.name = name

    def reset(self, init_value, ctype: str, py_var_type, value_multiplier=1, name=None):
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
        self.name = name
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
    rol = Byte_Var("s16", float, 0.01, name="rol")  # deg
    pit = Byte_Var("s16", float, 0.01, name="pit")  # deg
    yaw = Byte_Var("s16", float, 0.01, name="yaw")  # deg
    alt_fused = Byte_Var("s32", int, name="alt_fused")  # cm
    alt_add = Byte_Var("s32", int, name="alt_add")  # cm
    vel_x = Byte_Var("s16", int, name="vel_x")  # cm/s
    vel_y = Byte_Var("s16", int, name="vel_y")  # cm/s
    vel_z = Byte_Var("s16", int, name="vel_z")  # cm/s
    pos_x = Byte_Var("s32", int, name="pos_x")  # cm
    pos_y = Byte_Var("s32", int, name="pos_y")  # cm
    bat = Byte_Var("u16", float, 0.01, name="bat")  # V
    mode = Byte_Var("u8", int, name="mode")  #
    unlock = Byte_Var("u8", bool, name="unlock")  #
    cid = Byte_Var("u8", int, name="cid")  #
    cmd_0 = Byte_Var("u8", int, name="cmd_0")  #
    cmd_1 = Byte_Var("u8", int, name="cmd_1")  #

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
        cmd_0,
        cmd_1,
    ]

    @property
    def command_now(self):
        return (self.cid.value, self.cmd_0.value, self.cmd_1.value)


class FC_Settings_Struct:
    wait_ack_timeout = 0.1
    wait_sending_timeout = 0.2
    ack_max_retry = 3
    action_log_output = True


class FC_Base_Comunication:
    def __init__(self, serial_port, bit_rate) -> None:
        self.running = False
        self.connected = False
        self.__start_bit = [0xAA, 0x22]
        self.__listen_thread = None
        self.__state_update_callback = None
        self.__print_state_flag = False
        self.__ser_32 = SerCom32(serial_port, bit_rate)
        self.__send_lock = threading.Lock()
        self.__waiting_ack = False
        self.__recivied_ack = None
        logger.info("[FC] Serial port opened")
        self.__set_option(0)
        self.__ser_32.read_config(startBit=[0xAA, 0x55])
        self.state = FC_State_Struct()
        self.settings = FC_Settings_Struct()

    def start_listen_serial(self, print_state=True, callback=None, daemon=True):
        self.running = True
        self.__state_update_callback = callback
        self.__print_state_flag = print_state
        self.__listen_thread = threading.Thread(target=self.__listen_serial_task)
        self.__listen_thread.setDaemon(daemon)
        self.__listen_thread.start()

    def quit(self) -> None:
        self.running = False
        if self.__listen_thread:
            self.__listen_thread.join()
        self.__ser_32.close()
        logger.info("[FC] Threads closed, FC offline")

    def __set_option(self, option: int) -> None:
        self.__ser_32.send_config(
            startBit=self.__start_bit,
            optionBit=[option],
        )

    def send_32_from_data(
        self,
        data: bytes,
        option: int,
        need_ack: bool = False,
        __ack_retry_count: int = None,
    ):
        if need_ack:
            if __ack_retry_count is None:
                __ack_retry_count = self.settings.ack_max_retry
            if __ack_retry_count < 0:
                # raise Exception("Wait ACK reached max retry")
                logger.error("Wait ACK reached max retry")
                return None
            self.__waiting_ack = True
            self.__recivied_ack = None
            send_time = time.time()
            check_ack = option
            for add_bit in data:
                check_ack = (check_ack + add_bit) & 0xFF
        try:
            self.__send_lock.acquire(timeout=self.settings.wait_sending_timeout)
        except:
            logger.error("[FC] Wait sending data timeout")
            return None
        self.__set_option(option)
        sended = self.__ser_32.write(data)
        self.__send_lock.release()
        if need_ack:
            while self.__waiting_ack:
                if time.time() - send_time > self.settings.wait_ack_timeout:
                    logger.warning("[FC] ACK timeout, retrying")
                    return self.send_32_from_data(
                        data, option, need_ack, __ack_retry_count - 1
                    )
                time.sleep(0.001)
            if self.__recivied_ack is None or self.__recivied_ack != check_ack:
                logger.warning("[FC] ACK not received or invalid, retrying")
                return self.send_32_from_data(
                    data, option, need_ack, __ack_retry_count - 1
                )
        return sended

    def __listen_serial_task(self):
        logger.info("[FC] listen serial thread started")
        last_heartbeat_time = time.time()
        while self.running:
            try:
                if self.__ser_32.read():
                    _data = self.__ser_32.rx_data
                    # logger.debug(f"[FC] Read: {bytes_to_str(_data)}")
                    cmd = _data[0]
                    data = _data[1:]
                    if cmd == 0x01:  # 状态回传
                        self.__update_state(data)
                    elif cmd == 0x02:  # ACK返回
                        self.__recivied_ack = data[0]
                        self.__waiting_ack = False
                        # logger.debug(f"[FC] ACK received: {self.__recivied_ack}")
            except Exception as e:
                logger.error(f"[FC] listen serial exception: {traceback.format_exc()}")
            if time.time() - last_heartbeat_time > 0.25:
                self.send_32_from_data(b"\x01", 0x00)  # 心跳包
                last_heartbeat_time = time.time()
            time.sleep(0.001)  # 降低CPU占用

    def __update_state(self, recv_byte):
        try:
            index = 0
            for var in self.state.RECV_ORDER:
                length = var.byte_length
                var.bytes = recv_byte[index : index + length]
                index += length
            if not self.connected:
                self.connected = True
                logger.info("[FC] Connected")
            if callable(self.__state_update_callback):
                self.__state_update_callback(self.state)
            if self.__print_state_flag:
                self.__print_state()
        except Exception as e:
            logger.error(f"[FC] Update state exception: {traceback.format_exc()}")

    def __print_state(self):
        RED = "\033[1;31m"
        GREEN = "\033[1;32m"
        YELLOW = "\033[1;33m"
        BLUE = "\033[1;34m"
        CYAN = "\033[1;36m"
        PURPLE = "\033[1;35m"
        RESET = "\033[0m"
        text = ""
        text += " ".join(
            [
                f"{YELLOW}{((var.name[0]+var.name[-1]))}: {f'{GREEN}√ ' if var.value else f'{RED}x {RESET}'}"
                if type(var.value) == bool
                else (
                    f"{YELLOW}{((var.name[0]+var.name[-1]))}:{CYAN}{var.value:^7.02f}{RESET}"
                    if type(var.value) == float
                    else f"{YELLOW}{((var.name[0]+var.name[-1]))}:{CYAN}{var.value:^4d}{RESET}"
                )
                for var in self.state.RECV_ORDER
            ]
        )
        print(f"\r {text}\r", end="")


class FC_Protocol(FC_Base_Comunication):
    # constants
    HOLD_ALT_MODE = 1
    HOLD_POS_MODE = 2
    PROGRAM_MODE = 3

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.__byte_temp1 = Byte_Var()
        self.__byte_temp2 = Byte_Var()
        self.__byte_temp3 = Byte_Var()
        self.__byte_temp4 = Byte_Var()
        self.last_command = (0, 0, 0)  # (CID,CMD_0,CMD_1)

    def __action_log(self, action: str, data_info: str = None):
        if self.settings.action_log_output:
            string = f"[FC] [ACTION] {action.upper()}"
            if data_info is not None:
                string += f" -> {data_info}"
            logger.info(string)

    ######### 飞控命令 #########

    def __send_32_command(self, suboption: int, data: bytes = b"") -> None:
        self.__byte_temp1.reset(suboption, "u8", int)
        data_to_send = self.__byte_temp1.bytes + data
        sended = self.send_32_from_data(data_to_send, 0x01, need_ack=False)
        # logger.debug(f"[FC] Send: {bytes_to_str(sended)}")

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
        self.__action_log("set rgb led", f"#{r:02X}{g:02X}{b:02X}")

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
            self.__byte_temp1.bytes
            + self.__byte_temp2.bytes
            + self.__byte_temp3.bytes
            + b"\x77",  # 帧结尾
        )

    def reset_position_prediction(self):
        """
        复位位置融合预测(伪造通用位置传感器)
        """
        self.send_general_position(0, 0, 0)
        self.__action_log("reset position prediction")

    def realtime_control(
        self, vel_x: int = 0, vel_y: int = 0, vel_z: int = 0, yaw: int = 0
    ) -> None:
        """
        发送实时控制帧, 仅在定点模式下有效(MODE=2), 切换模式前需要确保遥控器摇杆全部归中
        vel_x,vel_y,vel_z: cm/s 匿名坐标系
        yaw: deg/s 顺时针为正
        """
        self.__byte_temp1.reset(vel_x, "s16", int)
        self.__byte_temp2.reset(vel_y, "s16", int)
        self.__byte_temp3.reset(vel_z, "s16", int)
        self.__byte_temp4.reset(-yaw, "s16", int)  # 实际控制帧是逆时针正向,此处取反以适应标准
        self.__send_32_command(
            0x03,
            self.__byte_temp1.bytes
            + self.__byte_temp2.bytes
            + self.__byte_temp3.bytes
            + self.__byte_temp4.bytes
            + b"\x88",  # 帧结尾
        )

    ######### IMU 命令 #########

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
        self.last_command = (CID, CMD0, CMD1)
        # logger.debug(f"[FC] Send: {bytes_to_str(sended)}")

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
        self.__action_log("set flight mode", f"{mode}")

    def unlock(self) -> None:
        """
        解锁电机
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x01)
        self.__action_log("unlock")

    def lock(self) -> None:
        """
        锁定电机 / 紧急锁浆
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x02)
        self.__action_log("lock")

    def stablize(self) -> None:
        """
        恢复定点悬停, 将终止正在进行的所有控制
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x04)
        self.__action_log("stablize")

    def take_off(self) -> None:
        """
        一键起飞
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x05)
        self.__action_log("take off")

    def land(self) -> None:
        """
        一键降落
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x06)
        self.__action_log("land")

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
        self.__action_log(
            "horizontal move", f"{distance}cm, {speed}cm/s, {direction}deg"
        )

    def cordinate_move(self, x: int, y: int, speed: int) -> None:
        """
        匿名坐标系下的水平移动:
        移动半径在0-10000 cm
        x,y: 匿名坐标系下的坐标
        speed: 移动速度:10-300 cm/s
        """
        div = y / x if x != 0 else np.inf
        target_deg = -np.arctan(div) / np.pi * 180
        distance = np.sqrt(x**2 + y**2)
        if target_deg < 0:
            target_deg += 360
        self.__action_log("cordinate move", f"{x}, {y}, {speed}")
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
        self.__action_log("go up", f"{distance}cm, {speed}cm/s")

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
        self.__action_log("go down", f"{distance}cm, {speed}cm/s")

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
        self.__action_log("turn left", f"{deg}deg, {speed}deg/s")

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
        self.__action_log("turn right", f"{deg}deg, {speed}deg/s")

    def set_height(self, source: int, height: int, speed: int) -> None:
        """
        设置高度:
        高度源: 0:融合高度 1:激光高度
        高度:0-10000 cm
        速度:10-300 cm/s
        """
        self.__action_log(
            "set height",
            f"{'fusion' if source == 0 else 'lidar'}, {height}cm, {speed}cm/s",
        )
        if source == 0:
            alt = self.state.alt_fused
        elif source == 1:
            alt = self.state.alt_add
        while time.time() - alt.last_update_time > 0.5:
            time.sleep(0.1)  # 确保使用的是最新的高度
        if height < alt.value:
            self.go_down(alt.value - height, speed)
        elif height > alt.value:
            self.go_up(height - alt.value, speed)

    def set_yaw(self, yaw: int, speed: int) -> None:
        """
        设置偏航角:
        偏航角:-180-180 度
        偏航速度:5-90 deg/s
        """
        self.__action_log("set yaw", f"{yaw}deg, {speed}deg/s")
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
        self.__action_log("set target position", f"{x}, {y}")

    def set_target_height(self, height: int) -> None:
        """
        设置目标高度:
        目标对地高度:+100000 cm
        """
        if height < 0:
            height = 0
        self.__byte_temp1.reset(height, "s32", int)
        self.__send_imu_command_frame(0x10, 0x01, 0x02, self.__byte_temp1.bytes)
        self.__action_log("set target height", f"{height}cm")

    @property
    def last_command_done(self) -> bool:
        """
        最后一次指令是否完成
        """
        return self.last_command != self.state.command_now

    @property
    def is_stablizing(self) -> bool:
        """
        是否正在悬停
        """
        stable_command = (0x10, 0x00, 0x04)
        return self.state.command_now == stable_command

    def wait_for_last_command_done(self, timeout_s=10) -> bool:
        """
        等待最后一次指令完成
        """
        t0 = time.time()
        time.sleep(0.5)  # 等待数据回传
        while not self.last_command_done:
            time.sleep(0.1)
            if time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for last command done timeout")
                return False
        self.__action_log("wait ok", "last cmd done")
        return True

    def wait_for_stabilizing(self, timeout_s=10) -> bool:
        """
        等待进入悬停状态
        """
        t0 = time.time()
        time.sleep(0.5)  # 等待数据回传
        while not self.is_stablizing:
            time.sleep(0.1)
            if time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for stabilizing timeout")
                return False
        self.__action_log("wait ok", "stabilizing")
        return True

    def wait_for_lock(self, timeout_s=10) -> bool:
        """
        等待锁定
        """
        t0 = time.time()
        while self.state.unlock.value:
            time.sleep(0.1)
            if time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for lock timeout")
                return False
        self.__action_log("wait ok", "locked")
        return True

    def wait_for_takeoff_done(self, z_speed_threshold=4, timeout_s=5) -> bool:
        """
        等待起飞完成
        """
        t0 = time.time()
        time.sleep(1)  # 等待加速完成
        while self.state.vel_z.value < z_speed_threshold:
            time.sleep(0.1)
            if time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for takeoff done timeout")
                return False
        if self.state.alt_add.value < 20:
            logger.warning("[FC] takeoff failed, low altitude")
            return False
        time.sleep(1)  # 等待机身高度稳定
        self.__action_log("wait ok", "takeoff done")
        return True

    def manual_takeoff(self, height: int, speed: int) -> None:
        """
        程控起飞模式,使用前先解锁 (危险,当回传频率过低时容易冲顶,应使用较小的安全速度)
        """
        self.__action_log("manual takeoff start", f"{height}cm")
        last_mode = self.state.mode.value
        hgt = self.state.height
        timeout_value = height / speed  # 安全时间
        self.set_flight_mode(self.HOLD_POS_MODE)
        time.sleep(0.1)  # 等待模式设置完成
        self.realtime_control(vel_z=speed)
        takeoff_time = time.time()
        while hgt.value < height - 20:
            time.sleep(0.1)
            if time.time() - takeoff_time > timeout_value:  # 超时, 终止
                logger.warning("[FC] takeoff timeout, force stop")
                self.stablize()
                return
            else:
                self.realtime_control(vel_z=speed)  # 持续发送
        self.realtime_control(vel_z=10)
        while hgt.value < height - 2:
            time.sleep(0.1)
        self.stablize()
        self.set_flight_mode(last_mode) # 还原模式
        self.__action_log("manual takeoff ok")


class FC_Udp_Server(FC_Base_Comunication):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)


class FC_Controller(FC_Udp_Server, FC_Protocol):
    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
