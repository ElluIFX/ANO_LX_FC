import struct
import time

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
        self._byte_temp1 = Byte_Var()
        self._byte_temp2 = Byte_Var()
        self._byte_temp3 = Byte_Var()
        self._byte_temp4 = Byte_Var()
        self.last_sended_command = (0, 0, 0)  # (CID,CMD_0,CMD_1)

    def _action_log(self, action: str, data_info: str = None):
        if self.settings.action_log_output:
            string = f"[FC] [ACTION] {action.upper()}"
            if data_info is not None:
                string += f" -> {data_info}"
            logger.info(string)

    ######### 飞控命令 #########

    def _send_32_command(
        self, suboption: int, data: bytes = b"", need_ack=False
    ) -> None:
        self._byte_temp1.reset(suboption, "u8", int)
        sended = self.send_data_to_fc(
            self._byte_temp1.bytes + data, 0x01, need_ack=need_ack
        )
        # logger.debug(f"[FC] Send: {bytes_to_str(sended)}")

    def set_rgb_led(self, r: int, g: int, b: int) -> None:
        """
        设置由32控制的RGB LED
        r,g,b: 0-255
        """
        self._byte_temp1.reset(r, "u8", int)
        self._byte_temp2.reset(g, "u8", int)
        self._byte_temp3.reset(b, "u8", int)
        self._send_32_command(
            0x01,
            self._byte_temp1.bytes
            + self._byte_temp2.bytes
            + self._byte_temp3.bytes
            + b"\x11",  # 帧结尾
        )
        self._action_log("set rgb led", f"#{r:02X}{g:02X}{b:02X}")

    def send_general_position(
        self, x: int = -2_147_483_648, y: int = -2_147_483_648, z: int = -2_147_483_648
    ) -> None:
        """
        通用数据传感器回传
        x,y,z: cm
        默认值 -2_147_483_648 代表输入无效
        """
        x = x if x is not None else -2_147_483_648
        y = y if y is not None else -2_147_483_648
        z = z if z is not None else -2_147_483_648
        data = struct.pack("<iii", int(x), int(y), int(z))
        self._send_32_command(
            0x02,
            data + b"\x22",  # 帧结尾
        )

    def send_realtime_control_data(
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
        data = struct.pack("<hhhh", int(vel_x), int(vel_y), int(vel_z), int(-yaw))
        self._send_32_command(
            0x03,
            data + b"\x33",  # 帧结尾
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
        self._byte_temp1.reset(channel, "u8", int)
        self._byte_temp2.reset(pwm_int, "s16", int)
        self._send_32_command(
            0x04,
            self._byte_temp1.bytes + self._byte_temp2.bytes + b"\x44",
            True,  # need ack
        )
        self._action_log("set pwm output", f"channel {channel} pwm {pwm:.2f}")

    def set_digital_output(self, channel: int, on: bool) -> None:
        """
        设置数字输出
        channel: 0-3
        on: 开关
        """
        assert channel in [0, 1, 2, 3]
        on = 1 if on else 0
        self._byte_temp1.reset(channel, "u8", int)
        self._byte_temp2.reset(on, "u8", int)
        self._send_32_command(
            0x05, self._byte_temp1.bytes + self._byte_temp2.bytes + b"\x55"
        )
        self._action_log("set digital output", f"channel {channel} on {on}")

    def set_pod(self, state: int, time: int) -> None:
        """
        设置吊舱
        state: 1 放线 2 收线
        time: 动作时间 毫秒 收线触发限位开关时或到达时间会停止, 放线仅超时停止
        """
        self._byte_temp1.reset(state, "u8", int)
        self._byte_temp2.reset(time, "u32", int)
        self._send_32_command(
            0x06, self._byte_temp1.bytes + self._byte_temp2.bytes + b"\x66", True
        )
        self._action_log("set pod", f"state {state} time {time}")

    ######### IMU 命令 #########

    def _send_imu_command_frame(self, CID: int, CMD0: int, CMD1: int, CMD_data=b""):
        self._byte_temp1.reset(CID, "u8", int)
        self._byte_temp2.reset(CMD0, "u8", int)
        self._byte_temp3.reset(CMD1, "u8", int)
        bytes_data = bytes(CMD_data)
        if len(bytes_data) < 8:
            bytes_data += b"\x00" * (8 - len(bytes_data))
        if len(bytes_data) > 8:
            raise Exception("CMD_data length is too long")
        data_to_send = (
            self._byte_temp1.bytes
            + self._byte_temp2.bytes
            + self._byte_temp3.bytes
            + bytes_data
        )
        sended = self.send_data_to_fc(data_to_send, 0x02, need_ack=True)
        self.last_sended_command = (CID, CMD0, CMD1)
        # logger.debug(f"[FC] Send: {bytes_to_str(sended)}")

    def _check_mode(self, target_mode) -> bool:
        """
        检查当前模式是否与需要的模式一致
        """
        mode_dict = {1: "HOLD ALT", 2: "HOLD POS", 3: "PROGRAM"}
        if self.state.mode.value != target_mode:
            if self.settings.auto_change_mode:
                self._action_log("auto mode set", mode_dict[target_mode])
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
        self._byte_temp1.reset(mode, "u8", int)
        self._send_imu_command_frame(0x01, 0x01, 0x01, self._byte_temp1.bytes)
        self._action_log("set flight mode", f"{mode}")

    def unlock(self) -> None:
        """
        解锁电机 (随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x01)
        self._action_log("unlock")

    def lock(self) -> None:
        """
        锁定电机 / 紧急锁浆 (随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x02)
        self._action_log("lock")

    def stablize(self) -> None:
        """
        恢复定点悬停, 将终止正在进行的所有控制 (随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x04)
        self._action_log("stablize")

    def take_off(self, target_height: int = 0) -> None:
        """
        一键起飞 (除姿态模式外, 随时有效)
        目标高度: 0-500 cm, 0为默认高度
        """
        self._byte_temp1.reset(target_height, "u16", int)
        self._send_imu_command_frame(0x10, 0x00, 0x05, self._byte_temp1.bytes)
        self._action_log("take off")

    def land(self) -> None:
        """
        一键降落 (除姿态模式外, 随时有效)
        """
        self._send_imu_command_frame(0x10, 0x00, 0x06)
        self._action_log("land")

    def horizontal_move(self, distance: int, speed: int, direction: int) -> None:
        """
        水平移动: (程控模式下有效)
        移动距离:0-10000 cm
        移动速度:10-300 cm/s
        移动方向:0-359 度 (当前机头为0参考,顺时针)
        """
        self._check_mode(3)
        self._byte_temp1.reset(distance, "u16", int)
        self._byte_temp2.reset(speed, "u16", int)
        self._byte_temp3.reset(direction, "u16", int)
        self._send_imu_command_frame(
            0x10,
            0x02,
            0x03,
            self._byte_temp1.bytes + self._byte_temp2.bytes + self._byte_temp3.bytes,
        )
        self._action_log(
            "horizontal move", f"{distance}cm, {speed}cm/s, {direction}deg"
        )

    def go_up(self, distance: int, speed: int) -> None:
        """
        上升: (程控模式下有效)
        上升距离:0-10000 cm
        上升速度:10-300 cm/s
        """
        self._check_mode(3)
        self._byte_temp1.reset(distance, "u16", int)
        self._byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(
            0x10, 0x02, 0x01, self._byte_temp1.bytes + self._byte_temp2.bytes
        )
        self._action_log("go up", f"{distance}cm, {speed}cm/s")

    def go_down(self, distance: int, speed: int) -> None:
        """
        下降: (程控模式下有效)
        下降距离:0-10000 cm
        下降速度:10-300 cm/s
        """
        self._check_mode(3)
        self._byte_temp1.reset(distance, "u16", int)
        self._byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(
            0x10, 0x02, 0x02, self._byte_temp1.bytes + self._byte_temp2.bytes
        )
        self._action_log("go down", f"{distance}cm, {speed}cm/s")

    def turn_left(self, deg: int, speed: int) -> None:
        """
        左转: (程控模式下有效)
        左转角度:0-359 度
        左转速度:5-90 deg/s
        """
        self._check_mode(3)
        self._byte_temp1.reset(deg, "u16", int)
        self._byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(
            0x10, 0x02, 0x07, self._byte_temp1.bytes + self._byte_temp2.bytes
        )
        self._action_log("turn left", f"{deg}deg, {speed}deg/s")

    def turn_right(self, deg: int, speed: int) -> None:
        """
        右转: (程控模式下有效)
        右转角度:0-359 度
        右转速度:5-90 deg/s
        """
        self._check_mode(3)
        self._byte_temp1.reset(deg, "u16", int)
        self._byte_temp2.reset(speed, "u16", int)
        self._send_imu_command_frame(
            0x10, 0x02, 0x08, self._byte_temp1.bytes + self._byte_temp2.bytes
        )
        self._action_log("turn right", f"{deg}deg, {speed}deg/s")

    def set_height(self, source: int, height: int, speed: int) -> None:
        """
        设置高度: (程控模式下有效)
        高度源: 0:融合高度 1:激光高度
        高度:0-10000 cm
        速度:10-300 cm/s
        """
        self._action_log(
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
        self._action_log("set yaw", f"{yaw}deg, {speed}deg/s")
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
        self._check_mode(3)
        self._byte_temp1.reset(x, "s32", int)
        self._byte_temp2.reset(y, "s32", int)
        self._send_imu_command_frame(
            0x10, 0x01, 0x01, self._byte_temp1.bytes + self._byte_temp2.bytes
        )
        self._action_log("set target position", f"{x}, {y}")

    def set_target_height(self, height: int) -> None:
        """
        设置目标高度: (程控模式下有效)
        目标对地高度:+100000 cm
        """
        if height < 0:
            height = 0
        self._check_mode(3)
        self._byte_temp1.reset(height, "s32", int)
        self._send_imu_command_frame(0x10, 0x01, 0x02, self._byte_temp1.bytes)
        self._action_log("set target height", f"{height}cm")

    @property
    def last_command_done(self) -> bool:
        """
        最后一次指令是否完成
        """
        return self.last_sended_command != self.state.command_now

    @property
    def hovering(self) -> bool:
        """
        是否正在悬停
        """
        stable_command = (0x10, 0x00, 0x04)
        return self.state.command_now == stable_command
