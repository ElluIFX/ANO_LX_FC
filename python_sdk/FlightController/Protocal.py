import time

import numpy as np

from .Base import Byte_Var, FC_Base_Uart_Comunication
from .Logger import logger


class FC_Protocol(FC_Base_Uart_Comunication):
    """
    协议层, 定义了实际的控制命令
    """

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
        self.__last_realtime_call_time = 0
        self.last_command = (0, 0, 0)  # (CID,CMD_0,CMD_1)

    def __action_log(self, action: str, data_info: str = None):
        if self.settings.action_log_output:
            string = f"[FC] [ACTION] {action.upper()}"
            if data_info is not None:
                string += f" -> {data_info}"
            logger.info(string)

    ######### 飞控命令 #########

    def __send_32_command(
        self, suboption: int, data: bytes = b"", need_ack=False
    ) -> None:
        self.__byte_temp1.reset(suboption, "u8", int)
        data_to_send = self.__byte_temp1.bytes + data
        sended = self.send_data_to_fc(data_to_send, 0x01, need_ack=need_ack)
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
        在飞控内有实时控制帧的安全检查, 每个帧有效时间只有1s, 因此发送频率需要大于1Hz
        注意记得切换模式!!!
        vel_x,vel_y,vel_z: cm/s 匿名坐标系
        yaw: deg/s 顺时针为正
        """
        # 性能优化:因为实时控制帧需要频繁低延迟发送, 所以不做过多次变量初始化
        if time.time() - self.__last_realtime_call_time > 1:  # need init
            self.__byte_temp1.reset(vel_x, "s16", int)
            self.__byte_temp2.reset(vel_y, "s16", int)
            self.__byte_temp3.reset(vel_z, "s16", int)
            self.__byte_temp4.reset(-yaw, "s16", int)  # 实际控制帧是逆时针正向,此处取反以适应标准
        else:
            self.__byte_temp1.value = vel_x
            self.__byte_temp2.value = vel_y
            self.__byte_temp3.value = vel_z
            self.__byte_temp4.value = -yaw
        self.__last_realtime_call_time = time.time()
        self.__send_32_command(
            0x03,
            self.__byte_temp1.bytes
            + self.__byte_temp2.bytes
            + self.__byte_temp3.bytes
            + self.__byte_temp4.bytes
            + b"\x88",  # 帧结尾
        )

    def set_PWM_output(self, channel: int, pwm: float) -> None:
        """
        设置PWM输出
        channel: 0-3
        pwm: 0.00-100.00
        """
        assert channel in [0, 1, 2, 3]
        pwm_int = int(pwm * 100)
        pwm_int = max(0, min(10000, pwm_int))
        self.__byte_temp1.reset(channel, "u8", int)
        self.__byte_temp2.reset(pwm_int, "s16", int)
        self.__send_32_command(
            0x04, self.__byte_temp1.bytes + self.__byte_temp2.bytes + b"\x99", True
        )
        self.__action_log("set pwm output", f"channel {channel} pwm {pwm:.2f}")

    def set_buzzer(self, on: bool) -> None:
        """
        设置蜂鸣器
        on: 开关
        """
        self.__byte_temp1.reset(int(on), "u8", int)
        self.__send_32_command(0x05, self.__byte_temp1.bytes + b"\xAA")
        self.__action_log("set buzzer", f"{on}")

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
        sended = self.send_data_to_fc(data_to_send, 0x02, need_ack=True)
        self.last_command = (CID, CMD0, CMD1)
        # logger.debug(f"[FC] Send: {bytes_to_str(sended)}")

    def __check_mode(self, target_mode) -> bool:
        """
        检查当前模式是否与需要的模式一致
        """
        mode_dict = {1: "HOLD ALT", 2: "HOLD POS", 3: "PROGRAM"}
        if self.state.mode.value != target_mode:
            if self.settings.auto_change_mode:
                self.__action_log("auto mode set", mode_dict[target_mode])
                self.set_flight_mode(target_mode)
                time.sleep(0.1)  # 等待模式改变完成
                return True
            else:
                logger.error(
                    f"[FC] Mode error: action required mode is {mode_dict[target_mode]}"
                    f", but current mode is {mode_dict[self.state.mode.value]}"
                )
                return False
        return True

    def set_flight_mode(self, mode: int) -> None:
        """
        设置飞行模式: (随时有效)
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
        解锁电机 (随时有效)
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x01)
        self.__action_log("unlock")

    def lock(self) -> None:
        """
        锁定电机 / 紧急锁浆 (随时有效)
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x02)
        self.__action_log("lock")

    def stablize(self) -> None:
        """
        恢复定点悬停, 将终止正在进行的所有控制 (随时有效)
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x04)
        self.__action_log("stablize")

    def take_off(self) -> None:
        """
        一键起飞 (除姿态模式外, 随时有效)
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x05)
        self.__action_log("take off")

    def land(self) -> None:
        """
        一键降落 (除姿态模式外, 随时有效)
        """
        self.__send_imu_command_frame(0x10, 0x00, 0x06)
        self.__action_log("land")

    def horizontal_move(self, distance: int, speed: int, direction: int) -> None:
        """
        水平移动: (程控模式下有效)
        移动距离:0-10000 cm
        移动速度:10-300 cm/s
        移动方向:0-359 度 (当前机头为0参考,顺时针)
        """
        self.__check_mode(3)
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
        匿名坐标系下的水平移动: (程控模式下有效)
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
        上升: (程控模式下有效)
        上升距离:0-10000 cm
        上升速度:10-300 cm/s
        """
        self.__check_mode(3)
        self.__byte_temp1.reset(distance, "u16", int)
        self.__byte_temp2.reset(speed, "u16", int)
        self.__send_imu_command_frame(
            0x10, 0x02, 0x01, self.__byte_temp1.bytes + self.__byte_temp2.bytes
        )
        self.__action_log("go up", f"{distance}cm, {speed}cm/s")

    def go_down(self, distance: int, speed: int) -> None:
        """
        下降: (程控模式下有效)
        下降距离:0-10000 cm
        下降速度:10-300 cm/s
        """
        self.__check_mode(3)
        self.__byte_temp1.reset(distance, "u16", int)
        self.__byte_temp2.reset(speed, "u16", int)
        self.__send_imu_command_frame(
            0x10, 0x02, 0x02, self.__byte_temp1.bytes + self.__byte_temp2.bytes
        )
        self.__action_log("go down", f"{distance}cm, {speed}cm/s")

    def turn_left(self, deg: int, speed: int) -> None:
        """
        左转: (程控模式下有效)
        左转角度:0-359 度
        左转速度:5-90 deg/s
        """
        self.__check_mode(3)
        self.__byte_temp1.reset(deg, "u16", int)
        self.__byte_temp2.reset(speed, "u16", int)
        self.__send_imu_command_frame(
            0x10, 0x02, 0x07, self.__byte_temp1.bytes + self.__byte_temp2.bytes
        )
        self.__action_log("turn left", f"{deg}deg, {speed}deg/s")

    def turn_right(self, deg: int, speed: int) -> None:
        """
        右转: (程控模式下有效)
        右转角度:0-359 度
        右转速度:5-90 deg/s
        """
        self.__check_mode(3)
        self.__byte_temp1.reset(deg, "u16", int)
        self.__byte_temp2.reset(speed, "u16", int)
        self.__send_imu_command_frame(
            0x10, 0x02, 0x08, self.__byte_temp1.bytes + self.__byte_temp2.bytes
        )
        self.__action_log("turn right", f"{deg}deg, {speed}deg/s")

    def set_height(self, source: int, height: int, speed: int) -> None:
        """
        设置高度: (程控模式下有效)
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
        设置偏航角: (程控模式下有效)
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
        设置目标位置: (程控模式下有效)
        x:+-100000 cm
        y:+-100000 cm
        """
        self.__check_mode(3)
        self.__byte_temp1.reset(x, "s32", int)
        self.__byte_temp2.reset(y, "s32", int)
        self.__send_imu_command_frame(
            0x10, 0x01, 0x01, self.__byte_temp1.bytes + self.__byte_temp2.bytes
        )
        self.__action_log("set target position", f"{x}, {y}")

    def set_target_height(self, height: int) -> None:
        """
        设置目标高度: (程控模式下有效)
        目标对地高度:+100000 cm
        """
        if height < 0:
            height = 0
        self.__check_mode(3)
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

    def wait_for_connection(self, timeout_s=-1) -> bool:
        """
        等待飞控连接
        """
        t0 = time.time()
        while not self.connected:
            time.sleep(0.1)
            if timeout_s != -1 and time.time() - t0 > timeout_s:
                logger.warning("[FC] wait for fc connection timeout")
                return False
        self.__action_log("wait ok", "fc connection")
        return True

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
        self.set_flight_mode(last_mode)  # 还原模式
        self.__action_log("manual takeoff ok")
